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
// =============================================================================
#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
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
    create(const std::string& host, std::uint16_t port, std::error_code& ec);

    ~TcpTransport() override;

    [[nodiscard]] bool is_open() const noexcept override;

    /// 发一帧。**调用方传来的 frame 是裸 PDU**，这里负责加 MBAP。
    std::error_code send(span<const std::uint8_t> pdu, std::uint8_t unit) override;

    /// 收一帧。返回的是**裸 PDU**（MBAP 已剥掉并校验）。
    std::error_code receive(std::vector<std::uint8_t>& out,
                            std::chrono::milliseconds timeout,
                            std::uint8_t unit) override;

    void flush_input() override;
    void close() noexcept override;
    [[nodiscard]] bool is_rtu() const noexcept override { return false; }

private:
    TcpTransport();

    class Handler;
    friend class Handler;

    void on_bytes(const std::uint8_t* data, std::size_t n);

    /// 连接已被对端/netmech 关闭：只标记状态，唤醒等待者。
    /// **不要**再调 close()（那会 close_all 造成递归）。
    void mark_closed() noexcept
    {
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            m_closed = true;
        }
        m_cv.notify_all();
    }

    /// connect 的回调：拿 Conn 并交给 Handler。
    ///
    /// 必须作为成员存活 —— Mechanism 只存 weak_ptr<Linkable>，
    /// 局部变量出作用域后 on_linkup 就不会被调了。
    class Link;

    std::shared_ptr<netmech::Mechanism>  m_mech;
    std::shared_ptr<Link>                m_link;
    std::shared_ptr<netmech::Conn>       m_conn;

    std::mutex                m_mtx;
    std::condition_variable   m_cv;
    std::vector<std::uint8_t> m_rx;
    bool                      m_closed = false;

    std::uint16_t m_next_tid = 1;      ///< 下一次请求用的事务 ID
    std::uint16_t m_pending_tid = 0;   ///< 已发出、正在等响应的事务 ID
};

} // namespace detail
} // namespace modbusmech
