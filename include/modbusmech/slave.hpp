// =============================================================================
//  modbusmech - 从站
// =============================================================================
//  文件    : modbusmech/slave.hpp
//  用途    : 被动响应请求的一方（被采集的设备 / 模拟器）。
//
//  ★ 寄存器由应用提供
//  库**不持有**任何寄存器数据。应用实现 `RegisterBank`，库在收到请求时回调它。
//  这样：
//    * 应用可以直接把值存在自己的数据结构里（不必同步两份）
//    * 写操作能被应用感知（可以触发副作用，比如真的去操作继电器）
//    * 没有"库里的数组"和"应用里的数据"不一致的问题
//
//  ★ 线程
//  `RegisterBank` 的方法在**该连接所在的 io 线程**上被调用（RTU 是唯一的串
//  口线程，TCP 是每个客户端各自的回调）。同一个 bank 被多个连接共享时，应用
//  需要自己加锁。
// =============================================================================
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <modbusmech/common.hpp>

namespace netmech {
class SerialPort;
class Mechanism;
}

namespace modbusmech {

/// 从站的寄存器访问接口。**应用实现这个**。
///
/// 每个方法返回 false = 该地址不存在 → 库自动回 `illegal_data_address`。
class RegisterBank {
public:
    virtual ~RegisterBank() = default;

    /// 读线圈（功能码 01，可读写位）。
    ///
    /// @param[in] address 起始地址（0 基）
    /// @param[in] count   个数
    /// @param[out] out    结果，size 会被设为 count
    /// @return false 表示地址越界 → 回异常码 02
    virtual bool read_coils(std::uint16_t address, std::uint16_t count,
                            std::vector<bool>& out) = 0;

    /// 读离散输入（功能码 02，只读位）。
    virtual bool read_discrete_inputs(std::uint16_t address, std::uint16_t count,
                                      std::vector<bool>& out) = 0;

    /// 读保持寄存器（功能码 03，可读写）。
    virtual bool read_holding_registers(std::uint16_t address, std::uint16_t count,
                                        std::vector<std::uint16_t>& out) = 0;

    /// 读输入寄存器（功能码 04，只读）。
    virtual bool read_input_registers(std::uint16_t address, std::uint16_t count,
                                      std::vector<std::uint16_t>& out) = 0;

    /// 写单个线圈（功能码 05）。
    virtual bool write_single_coil(std::uint16_t address, bool value) = 0;

    /// 写单个寄存器（功能码 06）。
    virtual bool write_single_register(std::uint16_t address, std::uint16_t value) = 0;

    /// 写多个线圈（功能码 0F）。
    virtual bool write_multiple_coils(std::uint16_t address,
                                      const std::vector<bool>& values) = 0;

    /// 写多个寄存器（功能码 10）。
    virtual bool write_multiple_registers(std::uint16_t address,
                                          const std::vector<std::uint16_t>& values) = 0;
};

/// 从站。
class Slave {
public:
    struct Config {
        /// RTU 的从站地址（1~247）。0 表示广播（不应答）。
        std::uint8_t address = 1;

        /// RTU 的"帧结束"判定方式。
        enum class FrameBoundary {
            /// 靠 3.5 字符静默（规范做法）。需要高精度定时器。
            silence,

            /// 靠"读够期望长度"（更可靠，但对未知请求不适用）。
            length,
        };
        FrameBoundary boundary = FrameBoundary::silence;

        /// 3.5 字符静默的**毫秒数**（按波特率算，1.5 字符用于 >19200）。
        ///
        /// @note 9600bps 的 3.5 字符 ≈ 4ms。设 0 表示按波特率自动计算。
        std::uint32_t silence_ms = 0;
    };

    /// RTU 从站（串口）。
    [[nodiscard]] static std::shared_ptr<Slave>
    create_rtu(const std::string& device, std::uint32_t baud_rate,
               const Config& cfg, std::shared_ptr<RegisterBank> bank,
               std::error_code& ec);

    /// TCP 从站（服务端，**支持多个客户端**）。
    ///
    /// @param[in] port 监听端口（Modbus TCP 默认 502）
    ///
    /// @note 每个客户端是独立的会话，但共享同一个 `bank`。应用要保证 bank
    ///       线程安全（或者接受并发读写的语义）。
    [[nodiscard]] static std::shared_ptr<Slave>
    create_tcp(std::uint16_t port, const Config& cfg,
               std::shared_ptr<RegisterBank> bank, std::error_code& ec);

    ~Slave();
    Slave(const Slave&) = delete;
    Slave& operator=(const Slave&) = delete;

    [[nodiscard]] bool is_running() const noexcept;

    /// 当前 TCP 客户端数（RTU 恒为 0 或 1）。
    [[nodiscard]] std::size_t client_count() const noexcept;

    struct Stats {
        std::uint64_t requests   = 0;
        std::uint64_t exceptions = 0;   ///< 回出去异常的次数
        std::uint64_t bad_frames = 0;   ///< CRC/长度错的帧（直接丢弃）
    };
    [[nodiscard]] Stats stats() const noexcept;

    void stop() noexcept;

private:
    Slave();
    class Impl;
    std::shared_ptr<Impl> m_impl;
};

} // namespace modbusmech
