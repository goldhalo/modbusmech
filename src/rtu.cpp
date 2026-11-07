// =============================================================================
//  rtu.cpp —— RTU 传输实现
// =============================================================================
//  串口是异步的（netmech），Master 的 API 是同步的，所以这里做"异步转同步"：
//
//      on_read 回调 → 追加到 m_rx → notify
//      receive()   → 条件变量等 m_rx 够长 → 拷出来
//
//  线程模型：`receive` 在调用线程（应用的采集线程）上跑，`on_read` 在 io
//  线程上跑，靠 mutex + condition_variable 同步。
// =============================================================================
#include "rtu.hpp"
#include "wire.hpp"

#include <modbusmech/pdu.hpp>
#include <netmech/serial.hpp>

#include <algorithm>
#include <array>
#include <cstring>
#include <memory>
#include <vector>
#include <mutex>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <system_error>
#include <utility>

namespace modbusmech {
namespace detail {

namespace {

/// RTP 头长度不存在于 Modbus；这里算的是"除了地址和功能码之外还要多少"。
/// 见 receive() 里的分阶段等待。
constexpr std::size_t kRtuHeader = 2;    // 地址 + 功能码
constexpr std::size_t kRtuCrc     = 2;

} // namespace

/// 从"地址 + 功能码"推算整个响应的长度（不含 CRC）。
///
/// @return 0 表示"还算不出来"（数据不够）或功能码未知 —— 两者都要由调用方
///         区分：读类响应只收到 2 字节时是前者（要等第 3 字节），未知功能码是后者。
std::size_t rtu_response_body_len(const std::uint8_t* p, std::size_t avail) noexcept
{
    if (avail < 2) {
        return 0;
    }
    const std::uint8_t func = p[1];

    // 异常响应：地址 + 功能码|0x80 + 异常码 = 3（+CRC）
    if (func & 0x80u) {
        return 3;
    }

    switch (static_cast<Function>(func)) {
    case Function::read_coils:
    case Function::read_discrete_inputs:
    case Function::read_holding_registers:
    case Function::read_input_registers:
        // 地址 + 功能码 + 字节数 = 3，后面跟 byte_count 个数据字节
        if (avail < 3) {
            return 0;
        }
        return 3 + static_cast<std::size_t>(p[2]);

    case Function::write_single_coil:
    case Function::write_single_register:
    case Function::write_multiple_coils:
    case Function::write_multiple_registers:
        // 地址 + 功能码 + 地址(2) + 值/数量(2) = 6
        return 6;

    default:
        return 0;
    }
}


std::size_t rtu_request_frame_len(const std::uint8_t* p, std::size_t avail) noexcept
{
    if (avail < 2) {
        return 0;
    }
    const std::uint8_t func = p[1];
    if (func & 0x80u) {
        return 0;   // 从站不该收到异常响应；算不出就交给 silence 兜底
    }

    switch (static_cast<Function>(func)) {
    case Function::read_coils:
    case Function::read_discrete_inputs:
    case Function::read_holding_registers:
    case Function::read_input_registers:
    case Function::write_single_coil:
    case Function::write_single_register:
        // 地址(1) + 功能码(1) + 4 + CRC(2)
        return 8;

    case Function::write_multiple_coils:
    case Function::write_multiple_registers:
        // 地址(1) + 功能码(1) + 地址(2) + 数量(2) + 字节数(1) + 数据(n) + CRC(2)
        if (avail < 7) {
            return 0;   // 还没收到 byte count，算不出来
        }
        return 9 + static_cast<std::size_t>(p[6]);

    default:
        return 0;       // 未知功能码：交给 silence 兜底
    }
}


bool serial_params_valid(SerialParity parity, SerialStopBits stop_bits,
                         std::uint8_t data_bits) noexcept
{
    // 枚举值必须在值域内：调用方 static_cast 出非法值时不能静默当成默认值。
    const bool parity_ok = (parity == SerialParity::none || parity == SerialParity::odd
                            || parity == SerialParity::even);
    const bool stop_ok   = (stop_bits == SerialStopBits::one
                            || stop_bits == SerialStopBits::two);
    return parity_ok && stop_ok && data_bits >= 5 && data_bits <= 8;
}

void apply_serial_params(SerialParity parity, SerialStopBits stop_bits,
                         std::uint8_t data_bits, netmech::SerialConfig& sc)
{
    sc.data_bits = data_bits;
    // 显式映射而不是 static_cast，免得将来任一侧加值域时静默错位。
    switch (parity) {
    case SerialParity::none: sc.parity = netmech::SerialParity::none; break;
    case SerialParity::odd:  sc.parity = netmech::SerialParity::odd;  break;
    case SerialParity::even: sc.parity = netmech::SerialParity::even; break;
    }
    switch (stop_bits) {
    case SerialStopBits::one: sc.stop_bits = netmech::SerialStopBits::one; break;
    case SerialStopBits::two: sc.stop_bits = netmech::SerialStopBits::two; break;
    }
}


// =============================================================================
//  Handler：netmech 的串口事件
// =============================================================================
class RtuTransport::Handler final : public netmech::SerialHandler {
public:
    explicit Handler(std::shared_ptr<RtuTransport> owner) : m_owner(std::move(owner)) {}

    void on_read(std::error_code ec, netmech::Buf buf, std::size_t n) override
    {
        auto owner = m_owner.lock();
        if (!owner) {
            return;
        }
        if (ec) {
            owner->close();   // 串口出错 → 关掉，唤醒等待者
            return;
        }
        if (n > 0) {
            owner->on_bytes(static_cast<const std::uint8_t*>(buf.data), n);
        }
        // 继续读（读循环不能断）
        if (owner->m_port) {
            owner->m_port->read(netmech::Buf{nullptr, m_chunk.data(), m_chunk.size()});
        }
    }

    void start()
    {
        if (auto owner = m_owner.lock()) {
            owner->m_port->read(netmech::Buf{nullptr, m_chunk.data(), m_chunk.size()});
        }
    }

    std::array<std::uint8_t, RtuTransport::kReadChunk> m_chunk{};

private:
    std::weak_ptr<RtuTransport> m_owner;
};

// =============================================================================
//  RtuTransport
// =============================================================================

RtuTransport::RtuTransport() = default;

RtuTransport::~RtuTransport()
{
    close();
}

std::shared_ptr<RtuTransport> RtuTransport::create(const std::string& device,
                                                   const Config& cfg,
                                                   std::error_code& ec,
                                                   std::shared_ptr<netmech::Runtime> runtime)
{
    ec.clear();

    // 串口参数校验：data_bits 5~8；枚举值必须在值域内（防止调用方
    // static_cast 出非法值被静默当成默认值）。
    if (!serial_params_valid(cfg.parity, cfg.stop_bits, cfg.data_bits)) {
        ec = make_error_code(ModbusErrc::invalid_argument);
        return nullptr;
    }

    auto self = std::shared_ptr<RtuTransport>(new RtuTransport());
    self->m_cfg     = cfg;
    self->m_runtime = std::move(runtime);

    netmech::SerialConfig sc;
    sc.device    = device;
    sc.baud_rate = cfg.baud_rate;
    apply_serial_params(cfg.parity, cfg.stop_bits, cfg.data_bits, sc);

    auto handler = std::make_shared<Handler>(self);
    self->m_port = netmech::SerialPort::create(sc, handler, self->m_runtime, ec);
    if (!self->m_port) {
        return nullptr;
    }
    handler->start();
    return self;
}

bool RtuTransport::is_open() const noexcept
{
    return m_port && m_port->is_open();
}

void RtuTransport::on_bytes(const std::uint8_t* data, std::size_t n)
{
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        m_rx.insert(m_rx.end(), data, data + n);
        m_last_rx = std::chrono::steady_clock::now();
    }
    m_cv.notify_all();
}

void RtuTransport::resync()
{
    // 等到"线路连续安静 silence_ms"为止，期间到达的字节一律丢弃。
    //
    // 为什么需要：RTU 没有事务号。一次请求超时后，从站的**迟到响应**可能在
    // 下一笔请求期间到达，而 receive() 只能按"地址 + 功能码 + 长度"认帧 ——
    // 轮询同一从站、同一功能码时这三者必然相同，迟到响应就会被当成这次的
    // 响应，静默返回上一轮的寄存器值（后果比报错严重得多）。
    //
    // 代价：只在**传输失败之后**调用。线路本来就没数据时 m_last_rx 早已过期，
    // 这里立即返回，正常路径不付任何延迟。
    const auto quiet    = std::chrono::milliseconds(m_cfg.silence_ms);
    const auto cap      = std::chrono::milliseconds(50);   // 防止对端一直发导致卡住
    const auto deadline = std::chrono::steady_clock::now() + cap;

    std::unique_lock<std::mutex> lk(m_mtx);
    for (;;) {
        const auto now = std::chrono::steady_clock::now();
        m_rx.clear();                                     // 已经到的，全部丢掉
        if (m_closed || now >= deadline) {
            return;
        }
        const auto quiet_until = m_last_rx + quiet;
        if (now >= quiet_until) {
            return;                                       // 线路已经安静够久
        }
        m_cv.wait_until(lk, (std::min)(quiet_until, deadline));
    }
}

void RtuTransport::flush_input()
{
    std::lock_guard<std::mutex> lk(m_mtx);
    m_rx.clear();
}

std::error_code RtuTransport::send(span<const std::uint8_t> pdu, std::uint8_t unit,
                                   std::uint16_t& tid)
{
    tid = 0;   // RTU 没有事务号
    if (!is_open()) {
        return make_error_code(ModbusErrc::not_connected);
    }

    // RTU 的封装在这里做：地址 + PDU + CRC16
    //
    // ★ 必须用 owner 把帧交给 netmech 托管：串口写是**异步**的，netmech 只把
    //   指针排进发送队列（netmech 的 SerialPort::write 不拷贝数据），等内核
    //   真正取数据时本函数早已返回。以前这里传的是局部 vector + 空 owner，
    //   一旦上一笔写还没回调、本笔被排队，稍后就会从已释放的内存取数据（UAF）。
    //   用 shared_ptr 当 owner 是 O(1) 的引用计数开销，换生命周期正确 ——
    //   与 tcp.cpp 的发送路径保持同一写法。
    auto frame = std::make_shared<std::vector<std::uint8_t>>(
        detail::wrap_rtu(unit, pdu));
    if (!m_port->write(netmech::ConstBuf{frame, frame->data(), frame->size()})) {
        return make_error_code(ModbusErrc::io_error);
    }
    return {};
}

std::error_code RtuTransport::receive(std::vector<std::uint8_t>& out,
                                      std::chrono::milliseconds timeout,
                                      std::uint8_t unit, std::uint16_t /*tid*/)
{
    out.clear();

    const auto deadline = std::chrono::steady_clock::now() + timeout;

    // ---- 第一阶段：同步到"地址 + 功能码" ----
    //
    //  ★ 这里是个**重同步循环**，不是"等 2 个字节"那么简单：
    //    · 总线上可能有别的从站的应答（多主站/上一次的残留），地址不匹配就
    //      丢 1 个字节继续找 —— 以前是直接返回 bad_unit_id，残帧留在缓冲里，
    //      retries=0 时这一轮就没救了；
    //    · 读类响应（01/02/03/04）的**总长要等第 3 字节（byte count）**才能算。
    //      以前只等到 2 字节就去算长度，rtu_response_body_len 在 avail<3 时
    //      返回 0 → 直接判 unexpected_response。9600bps 下 1 字节 ≈ 1ms，
    //      串口几乎必然分次到达 —— 真机上表现为"读类请求随机失败"，
    //      而 pty 测试一次 write 全到，反而测不出来。
    {
        std::unique_lock<std::mutex> lk(m_mtx);

        for (;;) {
            if (!m_cv.wait_until(lk, deadline, [&] {
                    return m_closed || m_rx.size() >= kRtuHeader;
                })) {
                return make_error_code(ModbusErrc::timeout);
            }
            if (m_closed) {
                return make_error_code(ModbusErrc::not_connected);
            }
            if (m_rx[0] == unit) {
                break;
            }
            // 不是给我们的：吃掉这 1 个字节再找帧头
            m_rx.erase(m_rx.begin());
        }

        const std::uint8_t func = m_rx[1];
        const bool         is_read =
            (func & 0x80u) == 0 && (func == 1 || func == 2 || func == 3 || func == 4);
        if (is_read) {
            if (!m_cv.wait_until(lk, deadline, [&] {
                    return m_closed || m_rx.size() >= 3;
                })) {
                return make_error_code(ModbusErrc::timeout);
            }
            if (m_closed) {
                return make_error_code(ModbusErrc::not_connected);
            }
        }
    }

    // ---- 第二阶段：按功能码算总长 ----
    std::size_t body_len = 0;
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        body_len = rtu_response_body_len(m_rx.data(), m_rx.size());
    }
    if (body_len == 0) {
        return make_error_code(ModbusErrc::unexpected_response);
    }
    const std::size_t total = body_len + kRtuCrc;

    // ---- 第三阶段：等够总长 ----
    {
        std::unique_lock<std::mutex> lk(m_mtx);
        if (!m_cv.wait_until(lk, deadline, [&] {
                return m_closed || m_rx.size() >= total;
            })) {
            return make_error_code(ModbusErrc::timeout);
        }
        if (m_closed) {
            return make_error_code(ModbusErrc::not_connected);
        }
        out.assign(m_rx.begin(), m_rx.begin() + static_cast<std::ptrdiff_t>(total));
        // 把这一帧从缓冲里摘掉（后面可能还有别的从站的应答）
        m_rx.erase(m_rx.begin(), m_rx.begin() + static_cast<std::ptrdiff_t>(total));
    }

    // ---- 第四阶段：CRC 校验 ----
    const std::size_t data_len = out.size() - kRtuCrc;
    const std::uint16_t want = crc16(span<const std::uint8_t>(out.data(), data_len));
    const std::uint16_t got = static_cast<std::uint16_t>(
        out[data_len] | (static_cast<std::uint16_t>(out[data_len + 1]) << 8));

    if (want != got) {
        return make_error_code(ModbusErrc::crc_mismatch);
    }
    return {};
}

void RtuTransport::close() noexcept
{
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (m_closed) {
            return;
        }
        m_closed = true;
    }
    m_cv.notify_all();
    if (m_port) {
        m_port->close();
    }
}

} // namespace detail
} // namespace modbusmech
