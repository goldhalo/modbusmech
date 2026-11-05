// =============================================================================
//  modbusmech - 主站
// =============================================================================
//  文件    : modbusmech/master.hpp
//  用途    : 主动发起请求的一方（采集设备）。
//
//  ★ 同步 API
//  所有 request 都是**阻塞**的：发出去、等响应、返回。串口本来就是"一问一
//  答"串行，阻塞语义最自然；TCP 想并发就开多个 Master 实例。
//
//  ★ 重试
//  `Config::retries` 是**传输失败**的重试次数（超时、串口错误）。从站返回的
//  协议异常不重试——那是它明确拒绝了（除非 `Exception::server_device_busy`）。
// =============================================================================
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <modbusmech/common.hpp>
#include <modbusmech/pdu.hpp>

namespace netmech {
class SerialPort;
class Conn;
}

namespace modbusmech {

/// 主站。
class Master {
public:
    struct Config {
        /// 单元号（RTU 的叫从站地址，TCP 的叫单元 ID）。
        std::uint8_t unit = 1;

        /// 单次请求的超时。
        ///
        /// @note 9600bps 下一个 256 字节的 RTU 帧要 267ms，所以别设太小。
        ///       默认 500ms 是工业现场常见值。
        std::chrono::milliseconds timeout{500};

        /// 传输失败的重试次数（不含首次）。
        int retries = 1;

        /// 重试间隔。
        std::chrono::milliseconds retry_delay{100};
    };

    // ---- 创建 ----

    /// RTU 主站（串口）。
    ///
    /// @param[in] device 串口设备（"COM3" / "/dev/ttyUSB0"）
    /// @param[in] cfg    配置
    /// @param[out] ec    失败原因
    [[nodiscard]] static std::shared_ptr<Master>
    create_rtu(const std::string& device, const Config& cfg,
               std::uint32_t baud_rate, std::error_code& ec);

    /// TCP 主站（客户端）。
    [[nodiscard]] static std::shared_ptr<Master>
    create_tcp(const std::string& host, std::uint16_t port, const Config& cfg,
               std::error_code& ec);

    ~Master();
    Master(const Master&) = delete;
    Master& operator=(const Master&) = delete;

    // ---- 读写 ----
    //
    // 全部阻塞。返回值里的 Result 区分"传输失败"和"从站异常"。
    // 读成功时 out 被填充；失败时 out 内容未定义。

    Result read_coils(std::uint16_t address, std::uint16_t count,
                      std::vector<bool>& out);

    Result read_discrete_inputs(std::uint16_t address, std::uint16_t count,
                                std::vector<bool>& out);

    Result read_holding_registers(std::uint16_t address, std::uint16_t count,
                                  std::vector<std::uint16_t>& out);

    Result read_input_registers(std::uint16_t address, std::uint16_t count,
                                std::vector<std::uint16_t>& out);

    Result write_single_coil(std::uint16_t address, bool value);

    Result write_single_register(std::uint16_t address, std::uint16_t value);

    Result write_multiple_coils(std::uint16_t address,
                                const std::vector<bool>& values);

    Result write_multiple_registers(std::uint16_t address,
                                    const std::vector<std::uint16_t>& values);

    // ---- 状态 ----

    [[nodiscard]] bool           is_connected() const noexcept;
    [[nodiscard]] std::uint8_t   unit() const noexcept;
    void                         set_unit(std::uint8_t unit) noexcept;

    /// 已发出的请求数 / 成功数 / 超时数 / 异常数（诊断用）。
    struct Stats {
        std::uint64_t requests = 0;
        std::uint64_t succeeded = 0;
        std::uint64_t timeouts  = 0;   ///< 等响应超时
        std::uint64_t send_errors = 0; ///< 本地发送失败（连接断了 / 写不进去），不是超时
        std::uint64_t exceptions = 0;
        std::uint64_t crc_errors = 0;
    };
    [[nodiscard]] Stats stats() const noexcept;

    void close() noexcept;

private:
    Master();
    class Impl;
    std::shared_ptr<Impl> m_impl;
};

} // namespace modbusmech
