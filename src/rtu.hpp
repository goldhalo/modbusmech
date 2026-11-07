// =============================================================================
//  rtu.hpp —— RTU 传输（内部头）
// =============================================================================
//  串口 + 帧边界 + CRC。基于 netmech::SerialPort（异步读，这里转同步）。
// =============================================================================
#pragma once

#include "modbusmech/export.hpp"

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

// 只为把串口参数映射进 netmech::SerialConfig（内部头，公共头不引 netmech）。
#include <netmech/serial.hpp>

#include "transport.hpp"

namespace modbusmech {
namespace detail {

/// RTU 传输：一问一答。
class RtuTransport final : public Transport,
                           public std::enable_shared_from_this<RtuTransport> {
public:
    struct Config {
        std::uint32_t baud_rate = 9600;
        std::uint8_t  data_bits = 8;

        /// 校验位 / 停止位（由 Modbus 侧的 Config 透传；默认 8N1）。
        SerialParity   parity    = SerialParity::none;
        SerialStopBits stop_bits = SerialStopBits::one;

        /// 帧结束判定的静默时间（毫秒）。
        ///
        /// @par 什么时候需要它
        /// **只有 RS-485 半双工需要**：同一对线不能同时收发，从站只能靠
        /// "线路安静下来"判断一帧结束。RS-232 是全双工、TCP 有 MBAP 长度
        /// 字段，都不需要这个值。
        ///
        /// @par 主站侧拿它干什么
        /// 主站按功能码推长度收帧，不需要它来切帧；但它被用作**重同步的静默
        /// 阈值**：一次请求失败（超时/CRC 错）后，`resync()` 会等到线路连续
        /// 安静这么久才返回，把"还在路上"的迟到响应收完丢掉——否则下一次
        /// 请求可能把上一笔的迟到响应当成自己的（RTU 没有事务号，无法从协议
        /// 上区分）。
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
        /// 太大（>20ms）会让失败后的重试变慢，太小会切帧。
        std::uint32_t silence_ms = 5;
    };

    static std::shared_ptr<RtuTransport>
    create(const std::string& device, const Config& cfg, std::error_code& ec);

    ~RtuTransport() override;

    [[nodiscard]] bool is_open() const noexcept override;
    std::error_code send(span<const std::uint8_t> pdu, std::uint8_t unit,
                          std::uint16_t& tid) override;
    std::error_code receive(std::vector<std::uint8_t>& out,
                            std::chrono::milliseconds timeout,
                            std::uint8_t unit, std::uint16_t tid) override;
    void flush_input() override;
    void close() noexcept override;
    [[nodiscard]] bool is_rtu() const noexcept override { return true; }

    /// 失败后的重同步：等到线路安静 `silence_ms` 再清缓冲（见 transport.hpp）。
    void resync() override;

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
    /// 最近一次收到字节的时刻（受 m_mtx 保护）。resync() 用它判断"线路是否安静"。
    std::chrono::steady_clock::time_point m_last_rx{};
    bool                    m_closed = false;
};

/// 按已收到的响应前缀推算"整帧（不含 CRC）有多长"。
///
/// @param[in] p     已收到的字节
/// @param[in] avail 已收到多少字节
/// @return 0 表示**现在还算不出来**（数据不够）或功能码不认识。
///
/// @note 关键点：01/02/03/04 这些读类响应，长度要靠**第 3 字节（byte count）**
///       才能算 —— 只收到 2 字节时返回 0。调用方必须等够 3 字节再来问，
///       否则会把"数据还没到"当成"响应不对"。这条曾经是个真 bug
///       （9600bps 下读类请求随机失败）。
[[nodiscard]] MODBUSMECH_API std::size_t rtu_response_body_len(const std::uint8_t* p,
                                                std::size_t         avail) noexcept;

/// 按已收到的**请求**前缀推算"整帧（含地址与 CRC）有多长"。
///
/// 从站侧的 length 帧边界用它：读/写单个类请求固定 8 字节；0F/10 为
/// `9 + byte_count`（地址 1 + 功能码 1 + 地址 2 + 数量 2 + 字节数 1 + 数据 + CRC 2）。
///
/// @param[in] p     已收到的字节
/// @param[in] avail 已收到多少字节
/// @return 0 表示**现在还算不出来**（数据不够）或功能码不认识 ——
///         这两种情况调用方都应当回退到 silence 兜底，而不是傻等。
[[nodiscard]] MODBUSMECH_API std::size_t rtu_request_frame_len(const std::uint8_t* p,
                                                std::size_t         avail) noexcept;

/// 串口参数是否在值域内（data_bits 5~8；枚举值合法）。
[[nodiscard]] MODBUSMECH_API bool serial_params_valid(SerialParity    parity,
                                       SerialStopBits  stop_bits,
                                       std::uint8_t    data_bits) noexcept;

/// 把串口参数填进 netmech 的 SerialConfig（主站/从站共用一套映射，
/// 免得两处 switch 各写一遍、将来加值域时改一处忘一处）。
MODBUSMECH_API void apply_serial_params(SerialParity parity, SerialStopBits stop_bits,
                         std::uint8_t data_bits, netmech::SerialConfig& sc);

} // namespace detail
} // namespace modbusmech
