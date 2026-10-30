// =============================================================================
//  rtu.hpp —— RTU 传输（内部头）
// =============================================================================
//  串口 + 帧边界 + CRC。基于 netmech::SerialPort（异步读，这里转同步）。
// =============================================================================
#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include <modbusmech/common.hpp>

#include "transport.hpp"

namespace netmech {
class SerialPort;
}

namespace modbusmech {
namespace detail {

/// RTU 传输：一问一答。
class RtuTransport final : public Transport,
                           public std::enable_shared_from_this<RtuTransport> {
public:
    struct Config {
        std::uint32_t baud_rate = 9600;
        std::uint8_t  data_bits = 8;
        /// 3.5 字符静默（毫秒）。0 = 按波特率自动算。
        std::uint32_t silence_ms = 0;
    };

    static std::shared_ptr<RtuTransport>
    create(const std::string& device, const Config& cfg, std::error_code& ec);

    ~RtuTransport() override;

    [[nodiscard]] bool is_open() const noexcept override;
    std::error_code send(span<const std::uint8_t> frame) override;
    std::error_code receive(std::vector<std::uint8_t>& out,
                            std::chrono::milliseconds timeout,
                            std::uint8_t unit) override;
    void flush_input() override;
    void close() noexcept override;
    [[nodiscard]] bool is_rtu() const noexcept override { return true; }

    /// 3.5 字符的时间（毫秒）。
    [[nodiscard]] std::uint32_t silence_ms() const noexcept;

    /// 每次 on_read 提交的缓冲大小。
    static constexpr std::size_t kReadChunk = 512;

private:
    RtuTransport();

    /// netmech 的 SerialHandler 实现（内部类）。
    class Handler;
    friend class Handler;

    void on_bytes(const std::uint8_t* data, std::size_t n);

    std::shared_ptr<netmech::SerialPort> m_port;
    Config                               m_cfg;

    std::mutex              m_mtx;
    std::condition_variable m_cv;
    std::vector<std::uint8_t> m_rx;
    bool                    m_closed = false;
};

} // namespace detail
} // namespace modbusmech
