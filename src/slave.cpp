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

#include <modbusmech/pdu.hpp>
#include <netmech/net.hpp>
#include <netmech/serial.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace modbusmech {

namespace {

using namespace std::chrono_literals;

/// 给 PDU 追加 CRC 并返回完整帧。
std::vector<std::uint8_t> wrap_rtu(std::uint8_t unit, span<const std::uint8_t> pdu)
{
    std::vector<std::uint8_t> frame(1 + pdu.size() + 2);
    frame[0] = unit;
    if (!pdu.empty()) {
        std::memcpy(frame.data() + 1, pdu.data(), pdu.size());
    }
    const std::uint16_t crc = crc16(span<const std::uint8_t>(frame.data(), frame.size() - 2));
    frame[frame.size() - 2] = static_cast<std::uint8_t>(crc & 0xFF);
    frame[frame.size() - 1] = static_cast<std::uint8_t>((crc >> 8) & 0xFF);
    return frame;
}

} // namespace

// =============================================================================
//  Impl
// =============================================================================

class Slave::Impl : public std::enable_shared_from_this<Slave::Impl> {
public:
    Config                        cfg;
    std::shared_ptr<RegisterBank> bank;
    mutable Stats                 stats;

    // ---- 串口 ----
    std::shared_ptr<netmech::SerialPort> port;

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

    /// 帧处理线程：等 silence，然后处理整帧。
    void run()
    {
        const auto silence = std::chrono::milliseconds(cfg.silence_ms);

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

    void handle_frame(std::vector<std::uint8_t>& frame)
    {
        // 最短：地址(1) + 功能码(1) + CRC(2) = 4
        if (frame.size() < 4) {
            ++stats.bad_frames;
            return;
        }

        // ---- CRC ----
        const std::size_t data_len = frame.size() - 2;
        const std::uint16_t want = crc16(span<const std::uint8_t>(frame.data(), data_len));
        const std::uint16_t got = static_cast<std::uint16_t>(
            frame[data_len] | (static_cast<std::uint16_t>(frame[data_len + 1]) << 8));
        if (want != got) {
            ++stats.bad_frames;
            return;   // 从站对 CRC 错的帧**不应答**
        }

        // ---- 地址 ----
        const std::uint8_t unit = frame[0];
        const bool broadcast = (unit == 0);
        if (!broadcast && unit != cfg.address) {
            return;   // 不是给我的，静默丢弃
        }

        ++stats.requests;

        // ---- 解析 PDU ----
        const span<const std::uint8_t> pdu(frame.data() + 1, data_len - 1);
        std::vector<std::uint8_t> resp;
        if (!handle_pdu(pdu, resp)) {
            return;   // 无法处理（畸形帧），丢弃
        }

        // 广播不应答
        if (broadcast) {
            return;
        }

        if (resp.size() == 2 && (resp[0] & 0x80u)) {
            ++stats.exceptions;
        }

        auto out = wrap_rtu(cfg.address, span<const std::uint8_t>(resp.data(), resp.size()));
        if (port) {
            port->write(netmech::ConstBuf{nullptr, out.data(), out.size()});
        }
    }

    /// 处理一个请求 PDU，产出响应 PDU。
    /// @return false 表示无法处理（畸形/未知功能码且不想回异常）
    bool handle_pdu(span<const std::uint8_t> pdu, std::vector<std::uint8_t>& resp);

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

bool Slave::Impl::handle_pdu(span<const std::uint8_t> pdu,
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

    switch (f) {
    case Function::read_coils: {
        if (d.count == 0 || d.count > Limits::max_read_bits) { exception(Exception::illegal_data_value); return true; }
        if (!bank->read_coils(d.address, d.count, bits))      { exception(Exception::illegal_data_address); return true; }
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
        if (!bank->read_discrete_inputs(d.address, d.count, bits)) { exception(Exception::illegal_data_address); return true; }
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
        if (!bank->read_holding_registers(d.address, d.count, regs)) { exception(Exception::illegal_data_address); return true; }
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
        if (!bank->read_input_registers(d.address, d.count, regs)) { exception(Exception::illegal_data_address); return true; }
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
        if (!bank->write_single_coil(d.address, d.value == 0xFF00)) { exception(Exception::illegal_data_address); return true; }
        // 回显原请求
        resp.assign(pdu.begin(), pdu.end());
        return true;
    }

    case Function::write_single_register: {
        if (!bank->write_single_register(d.address, d.value)) { exception(Exception::illegal_data_address); return true; }
        resp.assign(pdu.begin(), pdu.end());
        return true;
    }

    case Function::write_multiple_coils: {
        if (d.bits.size() > Limits::max_write_bits || d.bits.empty()) { exception(Exception::illegal_data_value); return true; }
        if (!bank->write_multiple_coils(d.address, d.bits)) { exception(Exception::illegal_data_address); return true; }
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
        if (!bank->write_multiple_registers(d.address, d.registers)) { exception(Exception::illegal_data_address); return true; }
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

    if (!bank) {
        ec = make_error_code(ModbusErrc::invalid_argument);
        return nullptr;
    }
    if (cfg.address == 0 || cfg.address > 247) {
        ec = make_error_code(ModbusErrc::invalid_argument);
        return nullptr;
    }

    auto impl = std::make_shared<Impl>();
    impl->cfg  = cfg;
    impl->bank = std::move(bank);

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

bool Slave::is_running() const noexcept
{
    return m_impl && !m_impl->m_stop.load();
}

std::size_t Slave::client_count() const noexcept
{
    return (m_impl && m_impl->port && m_impl->port->is_open()) ? 1u : 0u;
}

Slave::Stats Slave::stats() const noexcept
{
    return m_impl ? m_impl->stats : Stats{};
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
}

} // namespace modbusmech
