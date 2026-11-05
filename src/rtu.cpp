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

/// 从"地址 + 功能码"推算整个响应的长度（不含 CRC）。
///
/// @return 0 表示功能码未知（无法推算）
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

} // namespace

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
                                                   std::error_code& ec)
{
    ec.clear();

    auto self = std::shared_ptr<RtuTransport>(new RtuTransport());
    self->m_cfg = cfg;

    netmech::SerialConfig sc;
    sc.device    = device;
    sc.baud_rate = cfg.baud_rate;
    sc.data_bits = cfg.data_bits;
    sc.parity    = netmech::SerialParity::none;
    sc.stop_bits = netmech::SerialStopBits::one;

    auto handler = std::make_shared<Handler>(self);
    self->m_port = netmech::SerialPort::create(sc, handler, ec);
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
    }
    m_cv.notify_all();
}

void RtuTransport::flush_input()
{
    std::lock_guard<std::mutex> lk(m_mtx);
    m_rx.clear();
}

std::error_code RtuTransport::send(span<const std::uint8_t> frame)
{
    if (!is_open()) {
        return make_error_code(ModbusErrc::not_connected);
    }
    if (!m_port->write(netmech::ConstBuf{nullptr, frame.data(), frame.size()})) {
        return make_error_code(ModbusErrc::io_error);
    }
    return {};
}

std::error_code RtuTransport::receive(std::vector<std::uint8_t>& out,
                                      std::chrono::milliseconds timeout,
                                      std::uint8_t unit)
{
    out.clear();

    const auto deadline = std::chrono::steady_clock::now() + timeout;

    // ---- 第一阶段：等地址 + 功能码（2 字节）----
    {
        std::unique_lock<std::mutex> lk(m_mtx);
        if (!m_cv.wait_until(lk, deadline, [&] {
                return m_closed || !m_rx.empty();
            })) {
            return make_error_code(ModbusErrc::timeout);
        }
        if (m_closed) {
            return make_error_code(ModbusErrc::not_connected);
        }
        // 地址不匹配：可能是别的从站的应答（多主站/错配），丢掉
        // 这里先只要求至少 1 字节，拿到地址再判断
        if (!m_cv.wait_until(lk, deadline, [&] {
                return m_closed || m_rx.size() >= 1;
            })) {
            return make_error_code(ModbusErrc::timeout);
        }
        if (m_rx[0] != unit) {
            // 不是给我们的，丢弃这一帧的头部，让上层重试/超时
            return make_error_code(ModbusErrc::bad_unit_id);
        }
        if (!m_cv.wait_until(lk, deadline, [&] {
                return m_closed || m_rx.size() >= kRtuHeader;
            })) {
            return make_error_code(ModbusErrc::timeout);
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
