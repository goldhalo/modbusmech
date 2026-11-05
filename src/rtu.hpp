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
#include <chrono>
#include <cstddef>
#include <string>
#include <system_error>

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

        /// 帧结束判定的静默时间（毫秒）。
        ///
        /// @par 什么时候需要它
        /// **只有 RS-485 半双工需要**：同一对线不能同时收发，从站只能靠
        /// "线路安静下来"判断一帧结束。RS-232 是全双工、TCP 有 MBAP 长度
        /// 字段，都不需要这个值。
        ///
        /// @par 为什么不按波特率自动算
        /// 规范给的是"3.5 个字符时间"（9600bps ≈ 4ms），但实际现场：
        ///   * USB 转 485 的转换器有额外的收发切换延迟
        ///   * 不同设备对"多长算一帧结束"的容忍度不一样
        ///   * 线长、干扰也会拉长尾部
        /// 库算出来的值大概率不准，**不如让使用者按实测调**。
        ///
        /// @par 建议值
        /// 9600bps 从 5ms 起调；19200 从 3ms 起；更高速率可降到 2ms。
        /// 太大（>20ms）会让响应变慢，太小会切帧。
        std::uint32_t silence_ms = 5;
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

    /// 当前配置的静默时间（毫秒）。
    [[nodiscard]] std::uint32_t silence_ms() const noexcept { return m_cfg.silence_ms; }

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
