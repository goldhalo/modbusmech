// =============================================================================
//  tcp.hpp —— Modbus TCP 传输（内部头）
// =============================================================================
//  MBAP 头 7 字节：
//      事务ID(2) | 协议ID(2)=0 | 长度(2) | 单元ID(1)
//
//  和 RTU 的三个差别：
//    1. 有**长度字段** → 不用猜帧边界，也不用静默
//    2. 有**事务 ID** → 能匹配请求/响应（天然支持并发）
//    3. 没有 CRC（TCP 自己保证完整性）
//
//  ★ 流水线（多个请求同时在途）
//  协议本身允许在同一连接上同时有多个事务（靠 tid 区分）。本层用
//  `tid → Pending` 表实现：send() 登记一个 Pending 并把 tid 交回去，
//  receive() 在自己的 Pending 上等；解析**统一在 on_bytes（io 线程）**做，
//  解析出整帧后按 tid 分发 —— 这样多个调用线程可以共用一个连接。
//  `max_in_flight` 控制同时在途的上限（=1 就退化成 RTU 式的串行）。
//
//  ★ 短连接设备（现场很常见）
//  有些 PLC 只支持"一问一连接"：回完一个响应就把连接关掉。这**不是错误**，
//  而是常态。默认（`auto_reconnect = false`）语义是"连接断了就报错"；
//  打开 auto_reconnect 后，`send()` 前发现没有连接就先建连，建连超时
//  直接返回失败（不会无限重试）。
// =============================================================================
#pragma once

namespace netmech { class Runtime; }


#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <cstddef>
#include <string>
#include <system_error>

#include <modbusmech/common.hpp>

#include "transport.hpp"

namespace netmech {
class Conn;
class Mechanism;
}

namespace modbusmech {
namespace detail {

/// MBAP 头长度。
constexpr std::size_t kMbapLen = 7;

/// TCP 传输（主站侧，客户端）。
class TcpTransport final : public Transport,
                           public std::enable_shared_from_this<TcpTransport> {
public:
    /// 连接到一个 Modbus TCP 从站。
    static std::shared_ptr<TcpTransport>
    create(const std::string& host, std::uint16_t port, std::error_code& ec,
           std::shared_ptr<netmech::Runtime> runtime = nullptr);

    ~TcpTransport() override;

    [[nodiscard]] bool is_open() const noexcept override;

    /// 发一帧。**调用方传来的 frame 是裸 PDU**，这里负责加 MBAP。
    /// @param[out] tid 本次事务号；receive() 用它取响应
    std::error_code send(span<const std::uint8_t> pdu, std::uint8_t unit,
                         std::uint16_t& tid) override;

    /// 收自己那一笔的响应（按 tid）。
    std::error_code receive(std::vector<std::uint8_t>& out,
                            std::chrono::milliseconds timeout,
                            std::uint8_t unit, std::uint16_t tid) override;

    /// TCP 侧**不能**清接收缓冲：缓冲里可能是别的在途请求的半个响应。
    /// （transaction 的"每笔请求前清残留"是 RTU 的语义，TCP 有 tid 兜底不需要。）
    void flush_input() override {}
    void resync() override {}

    void close() noexcept override;
    [[nodiscard]] bool is_rtu() const noexcept override { return false; }

    /// 连接断了之后是否自动重连。**默认 false**（与改造前一致：断了就报错）。
    void set_auto_reconnect(bool on) noexcept { m_auto_reconnect = on; }

    /// 建连超时（含首次连接与按需重连）。默认 3000ms。
    void set_connect_timeout(std::chrono::milliseconds t) noexcept override
    {
        if (t.count() > 0) {
            m_connect_timeout = t;
        }
    }

    /// 同一连接允许的在途请求上限（流水线）。默认 1（串行）。
    void set_max_in_flight(std::uint32_t n) noexcept override
    {
        m_max_in_flight = n == 0 ? 1u : n;
    }

    /// 已发生的重连次数（诊断用：能看出对端是不是在不停断链）。
    [[nodiscard]] std::uint64_t reconnects() const noexcept override
    {
        return m_reconnects.load(std::memory_order_relaxed);
    }

private:
    /// 一个在途事务。
    struct Pending {
        std::uint16_t             tid = 0;
        std::uint8_t              unit = 0;
        std::mutex                mtx;
        std::condition_variable   cv;
        bool                      done = false;
        std::error_code           ec;
        std::vector<std::uint8_t> pdu;
    };

    TcpTransport();

    class Handler;
    friend class Handler;

    /// 建一次连接并同步等 on_linkup / on_linkfail。
    bool connect_once(std::error_code& ec);

    /// 需要时确保连接可用（仅当 auto_reconnect 开启）。
    bool ensure_connected(std::error_code& ec);

    void on_bytes(const std::uint8_t* data, std::size_t n);

    /// 把一个完整帧交付给对应的等待者（找不到 = 迟到的响应，丢弃）。
    void dispatch_frame(std::uint16_t tid, std::uint8_t unit,
                        std::vector<std::uint8_t>&& pdu);

    /// 让所有在途请求立刻以 @p ec 结束（连接断开 / 关闭）。
    void fail_all(const std::error_code& ec) noexcept;

    /// 对端关闭 / IO 错误：当前连接不可用，但**允许**后续按需重连。
    void mark_dropped() noexcept
    {
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            m_alive = false;
        }
        fail_all(make_error_code(ModbusErrc::not_connected));
    }

    /// connect 的回调：拿 Conn 并交给 Handler。
    class Link;

    std::string                         m_host;
    std::shared_ptr<netmech::Runtime> m_runtime;   ///< 注入运行时（null=默认）
    std::uint16_t                       m_port = 0;
    std::shared_ptr<netmech::Mechanism> m_mech;
    std::shared_ptr<Link>               m_link;
    std::shared_ptr<netmech::Conn>      m_conn;

    mutable std::mutex        m_mtx;      ///< 保护 m_rx / m_alive / m_closed
    std::vector<std::uint8_t> m_rx;
    bool                      m_alive = false;        ///< 当前连接可用
    bool                      m_closed = false;       ///< 用户显式 close()：禁止重连
    bool                      m_auto_reconnect = false;
    std::chrono::milliseconds m_connect_timeout{3000};
    std::atomic<std::uint64_t> m_reconnects{0};

    /// 在途事务表（m_pending_mtx 保护）。
    std::mutex m_pending_mtx;
    std::unordered_map<std::uint16_t, std::shared_ptr<Pending>> m_pending;
    std::size_t m_max_in_flight = 1;
    std::uint16_t m_next_tid = 1;
};

} // namespace detail
} // namespace modbusmech
