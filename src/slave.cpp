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

} // namespace

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
            m_rx.insert(m_rx.end(), data, data + n);
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

    /// 帧处理线程：等 silence，然后处理整帧。
    void run()
    {
        const auto silence = frame_gap();

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
    [[nodiscard]] std::shared_ptr<RegisterBank> pick_bank(std::uint8_t unit) const
    {
        if (cfg.bank_for) {
            return cfg.bank_for(unit);
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
        if (!handle_pdu(*b, pdu, resp)) {
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
        auto out = detail::wrap_rtu(unit, span<const std::uint8_t>(resp.data(), resp.size()));
        if (port) {
            port->write(netmech::ConstBuf{nullptr, out.data(), out.size()});
        }
    }

    /// 处理一个请求 PDU，产出响应 PDU。
    /// @return false 表示无法处理（畸形/未知功能码且不想回异常）
    bool handle_pdu(RegisterBank& b, span<const std::uint8_t> pdu, std::vector<std::uint8_t>& resp);

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
            return start_read();
        }

        void on_read(std::error_code ec, netmech::Buf buf, std::size_t n) override
        {
            if (ec) {
                m_conn->close();
                return;
            }
            if (n > 0) {
                m_rx.insert(m_rx.end(), static_cast<const std::uint8_t*>(buf.data),
                            static_cast<const std::uint8_t*>(buf.data) + n);
                // ★ 缓冲上限：正常情况最多堆"两帧黏一起"（520 字节）。
                //   超过说明对端在发自造帧（或我们切帧逻辑坏），直接断 ——
                //   半帧永远到不齐时 m_rx 会无限涨，恶意客户端能靠这个占内存。
                if (m_rx.size() > kMaxTcpAduLen * 2) {
                    m_conn->close();
                    return;
                }
                process();
            }
            start_read();
        }

        void on_write(std::error_code ec, netmech::ConstBuf, std::size_t) override
        {
            if (ec) {
                m_conn->close();
            }
        }

        void on_close() override {}

    private:
        bool start_read()
        {
            return m_conn->read(netmech::Buf{nullptr, m_chunk.data(), m_chunk.size()});
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
                    m_conn->close();
                    return;
                }
                const std::size_t total = 6 + len;
                // ★ 上限：Modbus TCP 的 ADU 最大 260（MBAP 7 + PDU 253）。
                //   不查的话，一个 len=0xFFFF 的伪帧就能让我们等 64KB，
                //   并把这期间的流量全堆在 m_rx 里。
                if (total > kMaxTcpAduLen) {
                    m_conn->close();
                    return;
                }
                if (m_rx.size() < total) {
                    return;   // 还没收够
                }

                // 取出 PDU（跳过 7 字节 MBAP）
                const span<const std::uint8_t> pdu(m_rx.data() + detail::kMbapLen,
                                                   total - detail::kMbapLen);

                // 多从站（2026-11-02）：按 unit 取寄存器组；
                // 取不到 ⇒ 视为"这台设备不存在"，静默丢弃（但**不**关连接 ——
                // 同一个 TCP 连接上还会来别的 unit 的请求）。
                auto b = m_owner->pick_bank(unit);

                std::vector<std::uint8_t> resp;
                const bool ok = b ? m_owner->handle_pdu(*b, pdu, resp) : false;

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
                m_owner->write_conn(m_conn, std::move(out));
            }
        }

        std::shared_ptr<Impl>            m_owner;
        std::shared_ptr<netmech::Conn>   m_conn;
        std::vector<std::uint8_t>        m_rx;
        std::array<std::uint8_t, 1024>   m_chunk{};
    };

    /// 往一个 Conn 写（TCP 会话用）。
    void write_conn(const std::shared_ptr<netmech::Conn>& conn,
                    std::vector<std::uint8_t>&& data)
    {
        auto holder = std::make_shared<std::vector<std::uint8_t>>(std::move(data));
        conn->write(netmech::ConstBuf{holder, holder->data(), holder->size()});
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

            // ---- 鉴权：每条新连接问一次应用 ----
            // 返回 nullptr 就是"拒收"，netmech 会立刻关掉这条连接
            // （不读一帧、不回应，对端只看到连接被关）。
            if (owner->cfg.authorize) {
                const auto addr = c.remote_address();
                bool       ok   = false;
                try {
                    ok = owner->cfg.authorize(addr.address_string(), addr.port());
                } catch (...) {
                    ok = false;   // 回调抛异常 ⇒ 按拒绝处理，绝不穿过 netmech 的回调栈
                }
                if (!ok) {
                    // 与其它计数器保持一致的写法（on_accept 由监听器串行驱动）
                    owner->stats.rejected_clients.fetch_add(1, std::memory_order_relaxed);
                    return nullptr;
                }
            }

            return std::make_shared<TcpSession>(owner, c.shared_from_this());
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

bool Slave::Impl::handle_pdu(RegisterBank& b, span<const std::uint8_t> pdu,
                             std::vector<std::uint8_t>& resp)
{
    resp.clear();

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
    auto bank_error = [&]() -> Exception {
        const Exception e = b.last_exception();
        return e == Exception::none ? Exception::illegal_data_address : e;
    };

    switch (f) {
    case Function::read_coils: {
        if (d.count == 0 || d.count > Limits::max_read_bits) { exception(Exception::illegal_data_value); return true; }
        if (!b.read_coils(d.address, d.count, bits))      { exception(bank_error()); return true; }
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
        if (!b.read_discrete_inputs(d.address, d.count, bits)) { exception(bank_error()); return true; }
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
        if (!b.read_holding_registers(d.address, d.count, regs)) { exception(bank_error()); return true; }
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
        if (!b.read_input_registers(d.address, d.count, regs)) { exception(bank_error()); return true; }
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
        if (!b.write_single_coil(d.address, d.value == 0xFF00)) { exception(bank_error()); return true; }
        // 回显原请求
        resp.assign(pdu.begin(), pdu.end());
        return true;
    }

    case Function::write_single_register: {
        if (!b.write_single_register(d.address, d.value)) { exception(bank_error()); return true; }
        resp.assign(pdu.begin(), pdu.end());
        return true;
    }

    case Function::write_multiple_coils: {
        if (d.bits.size() > Limits::max_write_bits || d.bits.empty()) { exception(Exception::illegal_data_value); return true; }
        if (!b.write_multiple_coils(d.address, d.bits)) { exception(bank_error()); return true; }
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
        if (!b.write_multiple_registers(d.address, d.registers)) { exception(bank_error()); return true; }
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
                                         std::error_code& ec)
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

    auto impl = std::make_shared<Impl>();
    impl->cfg  = cfg;
    impl->bank = std::move(bank);

    impl->baud_rate = baud_rate;

    netmech::SerialConfig sc;
    sc.device    = device;
    sc.baud_rate = baud_rate;
    sc.data_bits = 8;

    auto sink = std::make_shared<Impl::SerialSink>(impl);
    impl->port = netmech::SerialPort::create(sc, sink, ec);
    if (!impl->port) {
        return nullptr;
    }
    sink->start();

    impl->m_thread = std::thread([impl] { impl->run(); });

    auto s = std::shared_ptr<Slave>(new Slave());
    s->m_impl = impl;
    return s;
}

std::shared_ptr<Slave> Slave::create_tcp(std::uint16_t port,
                                         const Config& cfg,
                                         std::shared_ptr<RegisterBank> bank,
                                         std::error_code& ec)
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

    impl->mech = netmech::Mechanism::create();
    if (!impl->mech) {
        ec = make_error_code(ModbusErrc::io_error);
        return nullptr;
    }

    auto link = std::make_shared<Impl::Link>(impl);
    impl->link = link;                    // ★ 存住（见成员注释）
    const netmech::Address addr{"0.0.0.0", port};
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
    return out;
}

void Slave::stop() noexcept
{
    if (!m_impl) {
        return;
    }
    m_impl->stop();
    if (m_impl->m_thread.joinable()) {
        m_impl->m_thread.join();
    }
    if (m_impl->mech) {
        m_impl->mech->close_all();
    }
}

} // namespace modbusmech
