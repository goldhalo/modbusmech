// =============================================================================
//  slave.cpp —— 从站实现（RTU）
// =============================================================================
//  从站和主站最大的不同：**它不知道请求有多长**，所以只能用"线路安静下来"
//  判断一帧结束。这就是 silence 帧边界唯一的用武之地。
//
//      收到字节 → 记时间戳 → 等 silence_ms 没有新字节 → 认为一帧完整
//
//  处理流程：
//      CRC 校验（失败丢弃，不应答）
//      → 地址校验（不是自己且不是广播 → 丢弃）
//      → 解析 PDU → 调 RegisterBank
//      → 组响应 → 写回
// =============================================================================
#include <modbusmech/slave.hpp>

#include "wire.hpp"

#include <modbusmech/pdu.hpp>
#include <netmech/net.hpp>
#include <netmech/serial.hpp>

#include "tcp.hpp"
#include "rtu.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>
#include <memory>
#include <cstddef>
#include <cstdint>
#include <string>
#include <system_error>
#include <utility>

namespace modbusmech {

namespace {

using namespace std::chrono_literals;

/// 每个请求、每个 io 线程的"待回异常码"槽。
///
/// 为什么是 thread_local：TCP 下 bank 被多个会话共享，回调跑在不同 io 线程；
/// 用普通成员会让"A 连接设的 03"被"B 连接的 02"读到（串码）。库在每个请求
/// 的 `handle_pdu` 入口复位它，应用只在失败前用 `set_exception()` 写入。
thread_local Exception g_pending_exception = Exception::none;

} // namespace

void RegisterBank::set_exception(Exception e) const noexcept
{
    g_pending_exception = e;
}

void RegisterBank::clear_exception() const noexcept
{
    g_pending_exception = Exception::none;
}

Exception RegisterBank::last_exception() const
{
    return g_pending_exception;
}

// =============================================================================
//  Impl
// =============================================================================

class Slave::Impl : public std::enable_shared_from_this<Slave::Impl> {
public:
    Config                        cfg;
    std::shared_ptr<RegisterBank> bank;
    /// 内部计数器：跨线程自增（不同客户端的回调在不同 io 线程上），必须原子。
    /// 对外仍然返回可拷贝的 `Slave::Stats`（那是快照）。
    struct StatsAtomic {
        std::atomic<std::uint64_t> requests{0};
        std::atomic<std::uint64_t> exceptions{0};
        std::atomic<std::uint64_t> bad_frames{0};
        std::atomic<std::uint64_t> rejected_clients{0};
        std::atomic<std::uint64_t> send_failures{0};
    };
    StatsAtomic                   stats;

    // ---- 串口 ----
    std::shared_ptr<netmech::SerialPort> port;
    std::uint32_t                        baud_rate = 9600;   ///< RTU 用；算 3.5 字符沉默

    // ---- TCP ----
    std::shared_ptr<netmech::Mechanism> mech;
    /// ★ 必须存住：Mechanism 只持有 weak_ptr<Linkable>，
    ///   局部变量销毁后新连接没人接，netmech 会按"无订阅者"立即关闭。
    std::shared_ptr<netmech::Linkable>  link;
    std::atomic<std::size_t>            clients{0};
    // 连接 id 直接用 netmech::Conn::id()（uint64_t、库保证稳定），
    // connect / disconnect 两个回调带回同一个 id；库不额外维护任何连接状态。

    /// 串口事件：收到字节就追加 + 记时间。
    class SerialSink final : public netmech::SerialHandler {
    public:
        explicit SerialSink(std::shared_ptr<Impl> owner) : m_owner(std::move(owner)) {}

        void on_read(std::error_code ec, netmech::Buf buf, std::size_t n) override
        {
            auto owner = m_owner.lock();
            if (!owner) {
                return;
            }
            if (ec) {
                owner->stop();
                return;
            }
            if (n > 0) {
                owner->on_bytes(static_cast<const std::uint8_t*>(buf.data), n);
            }
            if (owner->port) {
                owner->port->read(netmech::Buf{nullptr, m_chunk.data(), m_chunk.size()});
            }
        }

        void start()
        {
            if (auto owner = m_owner.lock()) {
                owner->port->read(netmech::Buf{nullptr, m_chunk.data(), m_chunk.size()});
            }
        }

        std::array<std::uint8_t, 512> m_chunk{};

    private:
        std::weak_ptr<Impl> m_owner;
    };

    void on_bytes(const std::uint8_t* data, std::size_t n)
    {
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            // ★ 缓冲上限：Modbus RTU 的 ADU 最大 256 字节，512 已经够放下
            //   "两帧粘一起"。超过说明线路在持续出字节却没有静默（波特率配错、
            //   干扰、另一台主站狂发）——那样静默切帧永远不成立，缓冲会无限涨，
            //   长期运行迟早 OOM。这里强制丢弃并计入 bad_frames。
            if (m_rx.size() + n > Limits::max_rtu_adu * 2) {
                m_rx.clear();
                stats.bad_frames.fetch_add(1, std::memory_order_relaxed);
            } else {
                m_rx.insert(m_rx.end(), data, data + n);
            }
            m_last_rx = std::chrono::steady_clock::now();
        }
        m_cv.notify_all();
    }

    /// 帧间"静默"阈值。
    ///
    /// Modbus over Serial Line §2.5.1.1：
    ///   · 波特率 ≤ 19200：3.5 个字符时间；
    ///   · 波特率 > 19200：固定 1.75 ms（规范就是这么定的，不是"1.5 字符"）。
    ///
    /// `cfg.silence_ms != 0` 时用显式值（调试/特殊硬件用）；`0` 表示按波特率算
    /// —— 这是默认值，也是 `slave.hpp` 里承诺的语义。
    ///
    /// @note 以前 `run()` 直接拿 `cfg.silence_ms` 用，默认 0 ⇒ 收到字节立刻切帧，
    ///       真串口上一帧会被切成多帧、CRC 全错。pty 测试一次到齐，掩盖了这个问题。
    [[nodiscard]] std::chrono::microseconds frame_gap() const noexcept
    {
        if (cfg.silence_ms != 0) {
            return std::chrono::milliseconds(cfg.silence_ms);
        }
        if (baud_rate == 0) {
            return std::chrono::milliseconds(4);
        }
        if (baud_rate > 19200) {
            return std::chrono::microseconds(1750);
        }
        // 3.5 字符 × 10 bit/字符（1 起始 + 8 数据 + 1 停止）
        const double us = 3.5 * 10.0 * 1'000'000.0 / static_cast<double>(baud_rate);
        return std::chrono::microseconds(static_cast<std::int64_t>(us + 0.5));
    }

    /// 帧处理线程：等 silence（或 length 模式下收够期望长度），然后处理整帧。
    void run()
    {
        const auto silence   = frame_gap();
        const bool by_length = (cfg.boundary == Config::FrameBoundary::length);

        while (!m_stop.load()) {
            std::vector<std::uint8_t> frame;
            {
                std::unique_lock<std::mutex> lk(m_mtx);
                // 等到"上次收到字节后过了 silence"或"收到停止信号"
                while (!m_stop.load()) {
                    if (m_rx.empty()) {
                        m_cv.wait_for(lk, 50ms);   // 没数据，小睡等
                        continue;
                    }

                    // ① length 模式：能算出期望长度、且已收齐 ⇒ 立刻切帧，
                    //    不用等静默（比纯长度少一次静默延迟，也避免了"粘帧"）。
                    if (by_length) {
                        const std::size_t want =
                            detail::rtu_request_frame_len(m_rx.data(), m_rx.size());
                        if (want > 0 && m_rx.size() >= want) {
                            frame.assign(m_rx.begin(),
                                         m_rx.begin() + static_cast<std::ptrdiff_t>(want));
                            m_rx.erase(m_rx.begin(),
                                       m_rx.begin() + static_cast<std::ptrdiff_t>(want));
                            break;
                        }
                        // 算不出来（字节还不够 / 功能码不认识）或还没收够 ⇒
                        // 落到下面的 silence 兜底。这正是 length 模式相对"纯长度"
                        // 的改进：不会因为不认识的功能码或畸形帧把这一路卡死。
                    }

                    // ② silence 兜底：线路安静够久 ⇒ 把现有字节当成一帧
                    const auto elapsed = std::chrono::steady_clock::now() - m_last_rx;
                    if (elapsed >= silence) {
                        frame = std::move(m_rx);
                        m_rx.clear();
                        break;
                    }
                    m_cv.wait_for(lk, silence - elapsed);
                }
            }
            if (m_stop.load()) {
                break;
            }
            if (!frame.empty()) {
                handle_frame(frame);
            }
        }
    }

    /// 取该 unit 对应的寄存器组。
    ///
    /// 未配置 `bank_for` ⇒ 全部 unit 共用 `create_rtu/create_tcp` 传入的那个 bank。
    /// 配置了但回调返回空 ⇒ 该 unit 视为"不存在"，调用处静默丢弃。
    ///
    /// ★ `bank_for` 是应用回调，抛异常不能穿出去：
    ///   RTU 侧会打断帧线程（run() 外层兜底后从站直接停摆），TCP 侧会关掉连接。
    ///   这里按"该 unit 没有 bank"处理（返回空 ⇒ 静默丢弃），与它返回空指针的
    ///   语义一致；同时计入 bad_frames，便于从日志看出是回调在抛。
    [[nodiscard]] std::shared_ptr<RegisterBank> pick_bank(std::uint8_t unit)
    {
        if (cfg.bank_for) {
            try {
                return cfg.bank_for(unit);
            } catch (...) {
                stats.bad_frames.fetch_add(1, std::memory_order_relaxed);
                return nullptr;
            }
        }
        return bank;
    }

    void handle_frame(std::vector<std::uint8_t>& frame)
    {
        // 最短：地址(1) + 功能码(1) + CRC(2) = 4
        if (frame.size() < 4) {
            stats.bad_frames.fetch_add(1, std::memory_order_relaxed);
            return;
        }

        // ---- CRC ----
        const std::size_t data_len = frame.size() - 2;
        const std::uint16_t want = crc16(span<const std::uint8_t>(frame.data(), data_len));
        const std::uint16_t got = static_cast<std::uint16_t>(
            frame[data_len] | (static_cast<std::uint16_t>(frame[data_len + 1]) << 8));
        if (want != got) {
            stats.bad_frames.fetch_add(1, std::memory_order_relaxed);
            return;   // 从站对 CRC 错的帧**不应答**
        }

        // ---- 地址 ----
        //
        // 2026-11-02 多从站：判据从"等于 cfg.address"放宽到 cfg.accepts(unit)。
        //   留空 addresses ⇒ accepts() 退化成 `unit == cfg.address`，行为不变。
        const std::uint8_t unit = frame[0];
        const bool broadcast = (unit == 0);
        if (!cfg.accepts(unit)) {
            return;   // 本实例不应答该地址，静默丢弃
        }

        // ---- 取该 unit 的寄存器组 ----
        //
        // ⚠️ 必须持住 shared_ptr 到 handle_pdu 结束：bank_for 每次都可能新建，
        //    只留裸指针会在回调返回后悬空。
        auto b = pick_bank(unit);
        if (!b) {
            return;   // 该 unit 未注册银行 ⇒ 等价于"总线上没有这台设备"，不应答
        }

        stats.requests.fetch_add(1, std::memory_order_relaxed);

        // ---- 解析 PDU ----
        const span<const std::uint8_t> pdu(frame.data() + 1, data_len - 1);
        std::vector<std::uint8_t> resp;
        if (!handle_pdu(unit, *b, pdu, resp)) {
            return;   // 无法处理（畸形帧），丢弃
        }

        // 广播不应答
        if (broadcast) {
            return;
        }

        if (resp.size() == 2 && (resp[0] & 0x80u)) {
            stats.exceptions.fetch_add(1, std::memory_order_relaxed);
        }

        // 🔴 响应地址必须回显**请求的 unit**，不能写 cfg.address ——
        //    多从站模式下 cfg.address 只是个"默认值"，回错地址会让主站把
        //    响应当噪声丢弃（主站按自己发出的 unit 校验应答）。
        //
        // ★ 用 owner 托管响应帧：netmech 的串口写是异步的（只排指针、不拷数据），
        //   局部 vector 一出作用域就悬垂。从站连发响应 / 低波特率时上一笔写可能
        //   还没回调，本帧会被排队 → 稍后从已释放内存取数据（UAF）。
        auto out = std::make_shared<std::vector<std::uint8_t>>(
            detail::wrap_rtu(unit, span<const std::uint8_t>(resp.data(), resp.size())));
        if (port) {
            // 写不进去（队满 / 端口已关）要把响应丢掉并计数 —— 以前完全忽略返回值，
            // 表现是"主站超时、从站什么都不知道"。
            if (!port->write(netmech::ConstBuf{out, out->data(), out->size()})) {
                stats.send_failures.fetch_add(1, std::memory_order_relaxed);
            }
        }
    }

    /// 处理一个请求 PDU，产出响应 PDU。
    /// @param[in] unit 请求里的 unit（要透传给 RegisterBank：透传/网关场景靠它区分设备）
    /// @return false 表示无法处理（畸形帧，静默丢弃）
    bool handle_pdu(std::uint8_t unit, RegisterBank& b, span<const std::uint8_t> pdu,
                    std::vector<std::uint8_t>& resp);

    /// TCP 的一个客户端会话。
    ///
    /// Modbus TCP 的 ADU 上限：MBAP(7) + PDU 最大 253 = 260。
    static constexpr std::size_t kMaxTcpAduLen = 260;

    /// 和 RTU 不同：不用 silence 切帧（MBAP 有长度字段），收到的字节只可能
    /// 是"半个帧"或"几个帧粘一起"，按长度字段循环切就行。
    class TcpSession final : public netmech::ConnHandler,
                             public std::enable_shared_from_this<TcpSession> {
    public:
        TcpSession(std::shared_ptr<Impl> owner, std::shared_ptr<netmech::Conn> conn)
            : m_owner(std::move(owner)), m_conn(std::move(conn))
        {
            m_owner->clients.fetch_add(1);
        }

        ~TcpSession() override
        {
            if (m_owner) {
                m_owner->clients.fetch_sub(1);
            }
        }

        bool on_open(std::shared_ptr<netmech::Conn> conn) override
        {
            (void)conn;
            
            // 空闲超时：借 netmech 的接收侧保活定时器实现 —— 连续
            // idle_timeout 没收到任何字节就回调 on_idle，max_miss=1 时
            // 回调返回后 netmech 以 keepalive_timeout 结束连接。
            // 从站不主动发请求，所以"没收到字节"就是"没人来问"。
            if (m_owner->cfg.idle_timeout.count() > 0) {
                m_conn->enable_recv_keepalive(m_owner->cfg.idle_timeout,
                                              m_owner->cfg.idle_timeout, 1);
            }
            return start_read();
        }

        void on_read(std::error_code ec, netmech::Buf buf, std::size_t n) override
        {
            if (ec) {
                close_conn();
                return;
            }
            if (n > 0) {
                m_rx.insert(m_rx.end(), static_cast<const std::uint8_t*>(buf.data),
                            static_cast<const std::uint8_t*>(buf.data) + n);
                // ★ 缓冲上限：正常情况最多堆"两帧黏一起"（520 字节）。
                //   超过说明对端在发自造帧（或我们切帧逻辑坏），直接断 ——
                //   半帧永远到不齐时 m_rx 会无限涨，恶意客户端能靠这个占内存。
                if (m_rx.size() > kMaxTcpAduLen * 2) {
                    close_conn();
                    return;
                }
                try {
                    process();
                } catch (...) {
                    // 处理过程中抛异常（bank 已在 handle_pdu 收敛，这里兜的是其它
                    // 逃逸异常，例如分配失败）：连接内部状态已不可信，直接关掉。
                    // 关键点是**不能**跳过下面的 start_read() —— 否则这条会话会
                    // 变成"没有在途读"的僵尸：不响应、不断开，只有对端超时。
                    close_conn();
                    return;
                }
            }
            start_read();
        }

        void on_write(std::error_code ec, netmech::ConstBuf, std::size_t) override
        {
            if (ec) {
                close_conn();
            }
        }

        /// 关闭连接（幂等、空安全）。on_close 之后 m_conn 已经释放，这里要能容忍。
        void close_conn() noexcept
        {
            if (m_conn) {
                m_conn->close();
            }
        }

        /// 连接终结。
        ///
        /// ★ 这里必须把 m_conn 放掉：netmech 的 finish_handler 会把 handler
        ///   （也就是本会话）留成 retired_handler 快照，直到 Conn::Impl 析构；
        ///   而本会话又持着 shared_ptr<Conn> —— 两者互持成环，会话永远不析构，
        ///   `client_count()` 只增不减（实测：空闲超时断开后仍为 1）。
        ///   放掉 m_conn 后，Conn::Impl 随关闭流程结束而析构 → 快照释放 →
        ///   本会话析构 → clients 正确递减。
        void on_close() override
        {
            m_conn.reset();
        }

    private:
        bool start_read()
        {
            // on_close 之后 m_conn 已释放：不要再发起读（否则空指针解引用 ——
            // netmech 在关闭时会先投递 on_read(aborted) 再调 on_close 的顺序
            // 不保证，两种顺序都可能出现）。
            return m_conn
                && m_conn->read(netmech::Buf{nullptr, m_chunk.data(), m_chunk.size()});
        }

        void process()
        {
            // MBAP: tid(2) | proto(2) | len(2) | unit(1) | pdu
            while (true) {
                if (m_rx.size() < detail::kMbapLen) {
                    return;
                }
                const std::uint16_t tid   = static_cast<std::uint16_t>((m_rx[0] << 8) | m_rx[1]);
                const std::uint16_t proto = static_cast<std::uint16_t>((m_rx[2] << 8) | m_rx[3]);
                const std::uint16_t len   = static_cast<std::uint16_t>((m_rx[4] << 8) | m_rx[5]);
                const std::uint8_t  unit  = m_rx[6];

                if (proto != 0 || len < 2) {
                    // 帧已经不可信，关掉连接（让它重连重来）
                    close_conn();
                    return;
                }
                const std::size_t total = 6 + len;
                // ★ 上限：Modbus TCP 的 ADU 最大 260（MBAP 7 + PDU 253）。
                //   不查的话，一个 len=0xFFFF 的伪帧就能让我们等 64KB，
                //   并把这期间的流量全堆在 m_rx 里。
                if (total > kMaxTcpAduLen) {
                    close_conn();
                    return;
                }
                if (m_rx.size() < total) {
                    return;   // 还没收够
                }

                // ★ 显式配置了 `Config::addresses` 时按它过滤单元：
                //   不在名单里的 unit 静默丢弃（与 RTU 的地址过滤一致）。
                //   默认（addresses 为空）TCP 仍保持"谁都能问"的宽松语义 ——
                //   Modbus TCP 的 unit 常被忽略/当路由字段，很多客户端用 0/255。
                //   以前这里完全不看 addresses，配了 --units 也不生效。
                if (!m_owner->cfg.addresses.empty() && !m_owner->cfg.accepts(unit)) {
                    m_rx.erase(m_rx.begin(),
                               m_rx.begin() + static_cast<std::ptrdiff_t>(total));
                    continue;
                }

                // 取出 PDU（跳过 7 字节 MBAP）
                const span<const std::uint8_t> pdu(m_rx.data() + detail::kMbapLen,
                                                   total - detail::kMbapLen);

                // 多从站（2026-11-02）：按 unit 取寄存器组；
                // 取不到 ⇒ 视为"这台设备不存在"，静默丢弃（但**不**关连接 ——
                // 同一个 TCP 连接上还会来别的 unit 的请求）。
                auto b = m_owner->pick_bank(unit);

                std::vector<std::uint8_t> resp;
                const bool ok = b ? m_owner->handle_pdu(unit, *b, pdu, resp) : false;

                // 吃掉这一帧
                m_rx.erase(m_rx.begin(), m_rx.begin() + static_cast<std::ptrdiff_t>(total));

                if (!ok) {
                    continue;   // 畸形帧或该 unit 无 bank，忽略
                }

                // 组响应：回填 tid + unit
                std::vector<std::uint8_t> out;
                out.reserve(detail::kMbapLen + resp.size());
                out.push_back(static_cast<std::uint8_t>(tid >> 8));
                out.push_back(static_cast<std::uint8_t>(tid & 0xFF));
                out.push_back(0);
                out.push_back(0);                                  // 协议 ID
                const std::uint16_t out_len = static_cast<std::uint16_t>(1 + resp.size());
                out.push_back(static_cast<std::uint8_t>(out_len >> 8));
                out.push_back(static_cast<std::uint8_t>(out_len & 0xFF));
                out.push_back(unit);                               // 回填单元 ID
                out.insert(out.end(), resp.begin(), resp.end());

                if (resp.size() == 2 && (resp[0] & 0x80u)) {
                    m_owner->stats.exceptions.fetch_add(1, std::memory_order_relaxed);
                }
                // 写不进去（队满 / 连接已断）⇒ 关掉这条连接：客户端立刻能重连
                // 重来，比"响应静默丢失、只能等超时"强。
                if (!m_owner->write_conn(m_conn, std::move(out))) {
                    close_conn();
                    return;
                }
            }
        }

        std::shared_ptr<Impl>            m_owner;
        std::shared_ptr<netmech::Conn>   m_conn;
        std::vector<std::uint8_t>        m_rx;
        std::array<std::uint8_t, 1024>   m_chunk{};
    };

    /// 往一个 Conn 写（TCP 会话用）。
    /// @return false 表示没写进去（发送队列满 / 连接已关闭）
    bool write_conn(const std::shared_ptr<netmech::Conn>& conn,
                    std::vector<std::uint8_t>&& data)
    {
        if (!conn) {
            return false;   // on_close 之后会话已放掉连接
        }
        auto holder = std::make_shared<std::vector<std::uint8_t>>(std::move(data));
        if (!conn->write(netmech::ConstBuf{holder, holder->data(), holder->size()})) {
            // 以前完全忽略返回值：写队列满（netmech 默认 2MB/256 笔）时响应
            // 静默丢失，客户端只会超时，从站侧也看不到任何异常计数。
            stats.send_failures.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        return true;
    }

    /// TCP 的 Linkable：每个 accept 建一个会话。
    class Link final : public netmech::Linkable {
    public:
        explicit Link(std::shared_ptr<Impl> owner) : m_owner(std::move(owner)) {}

        std::shared_ptr<netmech::ConnHandler> on_accept(netmech::Conn& c) override
        {
            auto owner = m_owner.lock();
            if (!owner) {
                return nullptr;
            }

            // ---- 连接数上限：超了直接拒 ----
            // 与 on_client_connected 拒绝走同一条路径（不读一帧、不回应）。
            // accept 由监听器串行驱动，所以"读计数 → 建会话"之间不会有并发 insert。
            if (owner->cfg.max_clients != 0
                && owner->clients.load(std::memory_order_relaxed) >= owner->cfg.max_clients) {
                owner->stats.rejected_clients.fetch_add(1, std::memory_order_relaxed);
                return nullptr;
            }

            // ---- 连接建立回调：接受/拒绝由使用端决定（替代旧 authorize）----
            // 返回 nullptr 就是"拒收"，netmech 会立刻关掉这条连接
            // （不读一帧、不回应），且不会投递 on_linkdown。
            const auto addr = c.remote_address();
            // id 直接用 netmech::Conn::id()：稳定、库推荐作 map 键，
            // on_accept 与 on_linkdown 拿到的是同一个值。
            const std::uint64_t id = c.id();
            bool ok = true;
            if (owner->cfg.on_client_connected) {
                try {
                    ok = owner->cfg.on_client_connected(id, addr.address_string(), addr.port());
                } catch (...) {
                    ok = false;   // 回调抛异常 ⇒ 按拒绝处理，绝不穿过 netmech 的回调栈
                }
            }
            if (!ok) {
                owner->stats.rejected_clients.fetch_add(1, std::memory_order_relaxed);
                return nullptr;
            }

            return std::make_shared<TcpSession>(owner, c.shared_from_this());
        }

        /// 连接断开：netmech 的 `Linkable::on_linkdown` 契约保证"每个被接管的
        /// 连接恰好调用一次"（拒绝的连接不调用），在 close 流程里紧跟
        /// `on_close()` 投递（`conn.cpp:900`），**不依赖 Conn 析构**。
        /// 因此断开通知放这一层，而不是 TcpSession 析构。
        void on_linkdown(netmech::Conn& c) override
        {
            auto owner = m_owner.lock();
            if (!owner || !owner->cfg.on_client_disconnected) {
                return;
            }
            const auto addr = c.remote_address();   // 关闭后返回缓存值，仍可用
            try {
                owner->cfg.on_client_disconnected(c.id(), addr.address_string(), addr.port());
            } catch (...) {
                // netmech 外层已包 try/catch；这里再兜一层，保证不穿回调栈。
            }
        }

    private:
        std::weak_ptr<Impl> m_owner;
    };

    void stop()
    {
        m_stop.store(true);
        m_cv.notify_all();
        if (port) {
            port->close();
        }
    }

    std::mutex                m_mtx;
    std::condition_variable   m_cv;
    std::vector<std::uint8_t> m_rx;
    std::chrono::steady_clock::time_point m_last_rx{};
    std::atomic<bool>         m_stop{false};
    std::thread               m_thread;
};

// =============================================================================
//  请求处理
// =============================================================================

bool Slave::Impl::handle_pdu(std::uint8_t unit, RegisterBank& b,
                             span<const std::uint8_t> pdu,
                             std::vector<std::uint8_t>& resp)
{
    resp.clear();

    // ★ 每个请求开始先复位异常槽：上一次失败设置的 03/04 不能污染这一帧。
    //   覆盖 last_exception() 的旧实现仍由应用自己管理状态（见 slave.hpp 注释）。
    b.clear_exception();

    // ★ 未知功能码必须回 01（illegal_function），不能静默丢弃。
    //   以前这里直接交给 decode_pdu：它对不认识的功能码返回 invalid_argument，
    //   handle_pdu 就把它当"畸形帧"丢了 —— 主站只能等到超时，无法区分
    //   "设备不支持这个功能码"和"设备根本没响应"。规范（V1.1b3 §7）要求前者
    //   回 01，后者才是静默。
    //   注意：功能码最高位为 1 的是异常响应，从站不该收到，仍按畸形帧丢弃。
    if (!pdu.empty() && (pdu[0] & 0x80u) == 0
        && !is_known_function(static_cast<Function>(pdu[0]))) {
        (void)encode_exception(static_cast<Function>(pdu[0]),
                               Exception::illegal_function, resp);
        return true;
    }

    DecodedPdu d;
    if (decode_pdu(pdu, d) != std::error_code{}) {
        return false;
    }
    if (d.kind == PduKind::exception) {
        return false;   // 从站不该收到异常响应
    }

    const Function f = d.function;
    std::vector<bool>          bits;
    std::vector<std::uint16_t> regs;

    auto exception = [&](Exception e) {
        (void)encode_exception(f, e, resp);
    };

    /// bank 说"这单做不了"时，到底回哪个异常码。
    ///
    /// 默认 `02`（illegal_data_address，与改造前一致）；
    /// 若 bank 用 `last_exception()` 明确指定了别的码，以它为准。
    ///
    /// 🔴 为什么需要这个（2026-11-02）：AM3SE 事件记录区**没有新事件**时，
    ///   厂家规约回的是 `01 84 03`（illegal_data_value），不是 02；
    ///   而 EAS 靠"回 03"来判断队列空、靠"传输失败"判断链路故障 ——
    ///   两者混不起来，回错码会让 EAS 把空队列当成配置错误。
    ///
    /// ★ last_exception() 同样是应用实现的虚函数，也可能抛 —— 它一抛就说明
    ///   bank 连"该回哪个码"都给不出来，按 04 处理，不能让它穿出去。
    auto bank_error = [&]() -> Exception {
        try {
            const Exception e = b.last_exception();
            return e == Exception::none ? Exception::illegal_data_address : e;
        } catch (...) {
            return Exception::server_device_failure;
        }
    };

    /// bank 调用的三态结果：
    ///   ok      = bank 返回 true（正常处理）
    ///   refused = bank 返回 false（按 bank_error() 决定异常码）
    ///   threw   = bank 抛了异常（resp 已经是 04，调用处直接返回）
    enum class BankResult { ok, refused, threw };

    /// 调用应用侧 bank 的统一入口（**异常收敛点**）。
    ///
    /// 设计文档 §8.2 的承诺是「bank 抛异常 → 库捕获 → 回 04（从站故障）并记
    /// stats.exceptions」，这里负责前半段：**绝不把异常带出本函数**。
    /// 不兜住的后果分两条链路：
    ///   · RTU：handle_frame 跑在独立线程里，异常穿出 run() = std::terminate
    ///     （整个进程直接死）；
    ///   · TCP：netmech 的 invoke_void 会吞掉异常，但会跳过 on_read 末尾的
    ///     start_read() —— 该会话从此没有在途读，静默僵死（不响应也不断开）。
    /// 统计口径：resp 变成 2 字节异常帧后，由 handle_frame / TcpSession::process
    /// 统一计数，这里不重复自增。
    auto call_bank = [&](auto&& fn) -> BankResult {
        try {
            return fn() ? BankResult::ok : BankResult::refused;
        } catch (...) {
            exception(Exception::server_device_failure);   // resp = f|0x80, 04
            return BankResult::threw;
        }
    };

    switch (f) {
    case Function::read_coils: {
        if (d.count == 0 || d.count > Limits::max_read_bits) { exception(Exception::illegal_data_value); return true; }
        const BankResult br = call_bank([&] { return b.read_coils(unit, d.address, d.count, bits); });
        if (br == BankResult::threw)   { return true; }               // resp 已是 04
        if (br == BankResult::refused) { exception(bank_error()); return true; }
        // bank 必须严格返回 count 个：多填/少填都不能凑合发帧 —— byte count 与
        // 请求的 count 对不上，对端会判 bad_length；多填还会让字节数溢出 uint8_t。
        if (bits.size() != d.count) { exception(Exception::server_device_failure); return true; }
        const std::size_t nbytes = (bits.size() + 7) / 8;
        resp.push_back(static_cast<std::uint8_t>(f));
        resp.push_back(static_cast<std::uint8_t>(nbytes));
        resp.resize(2 + nbytes, 0);
        for (std::size_t i = 0; i < bits.size(); ++i) {
            if (bits[i]) {
                resp[2 + i / 8] = static_cast<std::uint8_t>(resp[2 + i / 8] | (1u << (i % 8)));
            }
        }
        return true;
    }

    case Function::read_discrete_inputs: {
        if (d.count == 0 || d.count > Limits::max_read_bits) { exception(Exception::illegal_data_value); return true; }
        const BankResult br = call_bank([&] { return b.read_discrete_inputs(unit, d.address, d.count, bits); });
        if (br == BankResult::threw)   { return true; }
        if (br == BankResult::refused) { exception(bank_error()); return true; }
        if (bits.size() != d.count) { exception(Exception::server_device_failure); return true; }
        const std::size_t nbytes = (bits.size() + 7) / 8;
        resp.push_back(static_cast<std::uint8_t>(f));
        resp.push_back(static_cast<std::uint8_t>(nbytes));
        resp.resize(2 + nbytes, 0);
        for (std::size_t i = 0; i < bits.size(); ++i) {
            if (bits[i]) {
                resp[2 + i / 8] = static_cast<std::uint8_t>(resp[2 + i / 8] | (1u << (i % 8)));
            }
        }
        return true;
    }

    case Function::read_holding_registers: {
        if (d.count == 0 || d.count > Limits::max_read_registers) { exception(Exception::illegal_data_value); return true; }
        const BankResult br = call_bank([&] { return b.read_holding_registers(unit, d.address, d.count, regs); });
        if (br == BankResult::threw)   { return true; }
        if (br == BankResult::refused) { exception(bank_error()); return true; }
        if (regs.size() != d.count) { exception(Exception::server_device_failure); return true; }
        resp.push_back(static_cast<std::uint8_t>(f));
        resp.push_back(static_cast<std::uint8_t>(regs.size() * 2));
        for (std::uint16_t v : regs) {
            resp.push_back(static_cast<std::uint8_t>(v >> 8));
            resp.push_back(static_cast<std::uint8_t>(v & 0xFF));
        }
        return true;
    }

    case Function::read_input_registers: {
        if (d.count == 0 || d.count > Limits::max_read_registers) { exception(Exception::illegal_data_value); return true; }
        const BankResult br = call_bank([&] { return b.read_input_registers(unit, d.address, d.count, regs); });
        if (br == BankResult::threw)   { return true; }
        if (br == BankResult::refused) { exception(bank_error()); return true; }
        if (regs.size() != d.count) { exception(Exception::server_device_failure); return true; }
        resp.push_back(static_cast<std::uint8_t>(f));
        resp.push_back(static_cast<std::uint8_t>(regs.size() * 2));
        for (std::uint16_t v : regs) {
            resp.push_back(static_cast<std::uint8_t>(v >> 8));
            resp.push_back(static_cast<std::uint8_t>(v & 0xFF));
        }
        return true;
    }

    case Function::write_single_coil: {
        // 05 的"值"是 FF00（开）或 0000（关），其他值非法
        if (d.value != 0xFF00 && d.value != 0x0000) { exception(Exception::illegal_data_value); return true; }
        const BankResult br = call_bank([&] { return b.write_single_coil(unit, d.address, d.value == 0xFF00); });
        if (br == BankResult::threw)   { return true; }
        if (br == BankResult::refused) { exception(bank_error()); return true; }
        // 回显原请求（规范定长 5 字节；decode_pdu 已拒绝尾部垃圾，
        // 这里显式取前 5 字节做防御性截断，绝不把多余字节回显给对端）。
        resp.assign(pdu.begin(), pdu.begin() + 5);
        return true;
    }

    case Function::write_single_register: {
        const BankResult br = call_bank([&] { return b.write_single_register(unit, d.address, d.value); });
        if (br == BankResult::threw)   { return true; }
        if (br == BankResult::refused) { exception(bank_error()); return true; }
        // 同 05：定长 5 字节回显，防御性截断。
        resp.assign(pdu.begin(), pdu.begin() + 5);
        return true;
    }

    case Function::write_multiple_coils: {
        if (d.bits.size() > Limits::max_write_bits || d.bits.empty()) { exception(Exception::illegal_data_value); return true; }
        const BankResult br = call_bank([&] { return b.write_multiple_coils(unit, d.address, d.bits); });
        if (br == BankResult::threw)   { return true; }
        if (br == BankResult::refused) { exception(bank_error()); return true; }
        // 回显"地址 + 数量"
        resp.push_back(static_cast<std::uint8_t>(f));
        resp.push_back(static_cast<std::uint8_t>(d.address >> 8));
        resp.push_back(static_cast<std::uint8_t>(d.address & 0xFF));
        resp.push_back(static_cast<std::uint8_t>(d.bits.size() >> 8));
        resp.push_back(static_cast<std::uint8_t>(d.bits.size() & 0xFF));
        return true;
    }

    case Function::write_multiple_registers: {
        if (d.registers.size() > Limits::max_write_registers || d.registers.empty()) { exception(Exception::illegal_data_value); return true; }
        const BankResult br = call_bank([&] { return b.write_multiple_registers(unit, d.address, d.registers); });
        if (br == BankResult::threw)   { return true; }
        if (br == BankResult::refused) { exception(bank_error()); return true; }
        resp.push_back(static_cast<std::uint8_t>(f));
        resp.push_back(static_cast<std::uint8_t>(d.address >> 8));
        resp.push_back(static_cast<std::uint8_t>(d.address & 0xFF));
        resp.push_back(static_cast<std::uint8_t>(d.registers.size() >> 8));
        resp.push_back(static_cast<std::uint8_t>(d.registers.size() & 0xFF));
        return true;
    }
    }

    // 未知功能码
    exception(Exception::illegal_function);
    return true;
}

// =============================================================================
//  Slave
// =============================================================================

Slave::Slave() = default;

Slave::~Slave()
{
    stop();
}

std::shared_ptr<Slave> Slave::create_rtu(const std::string& device,
                                         std::uint32_t baud_rate,
                                         const Config& cfg,
                                         std::shared_ptr<RegisterBank> bank,
                                         std::error_code& ec,
                                         std::shared_ptr<netmech::Runtime> runtime)
{
    ec.clear();

    // bank 与 bank_for 至少要有一个（2026-11-02）：
    //   只有 bank      ⇒ 全部 unit 共用它（改造前行为）
    //   只有 bank_for  ⇒ 每 unit 各自分派（"一个串口模拟多台从站"的用法）
    //   两个都有       ⇒ bank_for 优先，bank 作兜底
    if (!bank && !cfg.bank_for) {
        ec = make_error_code(ModbusErrc::invalid_argument);
        return nullptr;
    }

    // address 只在"没配 addresses"时才是判据；配了 addresses 就以集合为准，
    // 此时 address 留默认值即可（不必强求它在 1~247 之间）。
    if (cfg.addresses.empty() && (cfg.address == 0 || cfg.address > 247)) {
        ec = make_error_code(ModbusErrc::invalid_argument);
        return nullptr;
    }

    // 串口参数校验（data_bits 5~8、枚举值合法）—— 与主站同一套判断，
    // 见 rtu.cpp 的 serial_params_valid()。
    if (!detail::serial_params_valid(cfg.parity, cfg.stop_bits, cfg.data_bits)) {
        ec = make_error_code(ModbusErrc::invalid_argument);
        return nullptr;
    }

    auto impl = std::make_shared<Impl>();
    impl->cfg  = cfg;
    impl->bank = std::move(bank);

    impl->baud_rate = baud_rate;

    netmech::SerialConfig sc;
    sc.device    = device;
    sc.baud_rate = baud_rate;
    // 以前这里写死 8N1（只设 data_bits=8），8E1/8O1 的设备根本连不上。
    detail::apply_serial_params(cfg.parity, cfg.stop_bits, cfg.data_bits, sc);

    auto sink = std::make_shared<Impl::SerialSink>(impl);
    impl->port = netmech::SerialPort::create(sc, sink, std::move(runtime), ec);
    if (!impl->port) {
        return nullptr;
    }
    sink->start();

    impl->m_thread = std::thread([impl] {
        // 最后一道防线：bank 的异常已在 handle_pdu 里收敛成 04 响应，这里再兜
        // 一层是为了杜绝"任何逃逸异常 → 线程函数返回 → std::terminate 整进程"。
        // 真走到这里说明有未预料的故障：置 stop 让 is_running() 变 false，
        // 至少上层能观测到从站已停，而不是静默死掉。
        try {
            impl->run();
        } catch (...) {
            impl->m_stop.store(true);
        }
    });

    auto s = std::shared_ptr<Slave>(new Slave());
    s->m_impl = impl;
    return s;
}

std::shared_ptr<Slave> Slave::create_tcp(std::uint16_t port,
                                         const Config& cfg,
                                         std::shared_ptr<RegisterBank> bank,
                                         std::error_code& ec,
                                         std::shared_ptr<netmech::Runtime> runtime)
{
    // 原签名保持源码兼容：旧实现硬编码 0.0.0.0，现在统一转发到可指定 host 的重载。
    return Slave::create_tcp(std::string{}, port, cfg, std::move(bank), ec,
                             std::move(runtime));
}

std::shared_ptr<Slave> Slave::create_tcp(const std::string& host,
                                         std::uint16_t port,
                                         const Config& cfg,
                                         std::shared_ptr<RegisterBank> bank,
                                         std::error_code& ec,
                                         std::shared_ptr<netmech::Runtime> runtime)
{
    ec.clear();

    // 同 create_rtu：bank 与 bank_for 至少给一个（TCP 多 unit 时常用 bank_for）
    if (!bank && !cfg.bank_for) {
        ec = make_error_code(ModbusErrc::invalid_argument);
        return nullptr;
    }

    auto impl = std::make_shared<Impl>();
    impl->cfg  = cfg;
    impl->bank = std::move(bank);

    impl->mech = netmech::Mechanism::create(netmech::Mechanism::Config{},
                                            std::move(runtime));
    if (!impl->mech) {
        ec = make_error_code(ModbusErrc::io_error);
        return nullptr;
    }

    auto link = std::make_shared<Impl::Link>(impl);
    impl->link = link;                    // ★ 存住（见成员注释）
    // 空 host 回退到"监听所有网卡"，与旧行为逐字一致。
    const netmech::Address addr{host.empty() ? "0.0.0.0" : host, port};
    impl->mech->set_linkable(link);
    ec = impl->mech->listen(addr, link);
    if (ec) {
        return nullptr;
    }

    auto s = std::shared_ptr<Slave>(new Slave());
    s->m_impl = impl;
    return s;
}

bool Slave::is_running() const noexcept
{
    if (!m_impl || m_impl->m_stop.load()) {
        return false;
    }
    if (m_impl->mech) {
        return m_impl->mech->running();
    }
    return m_impl->port && m_impl->port->is_open();
}

std::size_t Slave::client_count() const noexcept
{
    if (!m_impl) {
        return 0;
    }
    if (m_impl->mech) {
        return m_impl->clients.load();
    }
    return (m_impl->port && m_impl->port->is_open()) ? 1u : 0u;
}

std::uint16_t Slave::listening_port() const noexcept
{
    if (!m_impl || !m_impl->mech) {
        return 0;
    }
    try {
        // create_tcp(0, …) 时由系统分配端口；读的是**实际绑定地址**，
        // 不是入参里的 0。多监听时取第一个（测试场景只有一个）。
        const auto addrs = m_impl->mech->listening_addresses();
        return addrs.empty() ? std::uint16_t{0} : addrs.front().port();
    } catch (...) {
        return 0;
    }
}

Slave::Stats Slave::stats() const noexcept
{
    Stats out;
    if (!m_impl) {
        return out;
    }
    // 快照：字段各自原子读，**不保证四个数属于同一瞬间** —— 统计用途够了，
    // 想要严格一致就得为它加锁，那是把锁塞进热路径，不值。
    out.requests         = m_impl->stats.requests.load(std::memory_order_relaxed);
    out.exceptions       = m_impl->stats.exceptions.load(std::memory_order_relaxed);
    out.bad_frames       = m_impl->stats.bad_frames.load(std::memory_order_relaxed);
    out.rejected_clients = m_impl->stats.rejected_clients.load(std::memory_order_relaxed);
    out.send_failures    = m_impl->stats.send_failures.load(std::memory_order_relaxed);
    return out;
}

void Slave::stop() noexcept
{
    if (!m_impl) {
        return;
    }
    m_impl->stop();
    if (m_impl->m_thread.joinable()) {
        if (m_impl->m_thread.get_id() == std::this_thread::get_id()) {
            // ★ 在帧线程上调用（典型：bank 回调里"收到某写请求后停机"）：
            //   join 自己是 UB（std::system_error 在 noexcept 里抛出 → terminate）。
            //   detach：线程对象变 non-joinable；帧线程创建时捕获了
            //   shared_ptr<Impl>（强自持），m_stop 已置位，run() 随即
            //   退出并释放 Impl —— 无 UAF、无 terminate。
            m_impl->m_thread.detach();
        } else {
            m_impl->m_thread.join();
        }
    }
    if (m_impl->mech) {
        m_impl->mech->close_all();
    }
}

} // namespace modbusmech
