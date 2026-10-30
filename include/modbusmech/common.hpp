// =============================================================================
//  modbusmech - 基础类型（功能码 / 异常码 / 结果）
// =============================================================================
//  文件    : modbusmech/common.hpp
//  用途    : 协议层的常量与类型。纯定义，不依赖网络。
//  参考    : Modbus Application Protocol V1.1b3（协议的公开规范）
// =============================================================================
#pragma once

#include <cstdint>
#include <string>
#include <system_error>

namespace modbusmech {

// =============================================================================
//  功能码
// =============================================================================

/// Modbus 功能码。
enum class Function : std::uint8_t {
    read_coils              = 0x01,   ///< 读线圈（可读写位）
    read_discrete_inputs    = 0x02,   ///< 读离散输入（只读位）
    read_holding_registers  = 0x03,   ///< 读保持寄存器（可读写）
    read_input_registers    = 0x04,   ///< 读输入寄存器（只读）
    write_single_coil       = 0x05,   ///< 写单个线圈
    write_single_register   = 0x06,   ///< 写单个寄存器
    write_multiple_coils    = 0x0F,   ///< 写多个线圈
    write_multiple_registers= 0x10,   ///< 写多个寄存器
};

/// 功能码是否已知。
[[nodiscard]] bool is_known_function(Function f) noexcept;

// =============================================================================
//  异常码
// =============================================================================

/// 从站返回的异常码（响应里功能码的最高位置 1，紧跟一个字节的异常码）。
enum class Exception : std::uint8_t {
    none                     = 0x00,   ///< 不是异常（正常响应）
    illegal_function         = 0x01,   ///< 功能码不支持
    illegal_data_address     = 0x02,   ///< 地址越界
    illegal_data_value       = 0x03,   ///< 数据值非法（如数量超限）
    server_device_failure    = 0x04,   ///< 从站内部故障
    acknowledge              = 0x05,   ///< 已接受，但需要时间
    server_device_busy       = 0x06,   ///< 从站忙
    memory_parity_error      = 0x08,   ///< 存储奇偶校验错
    gateway_path_unavailable = 0x0A,   ///< 网关路径不可用
    gateway_target_failed    = 0x0B,   ///< 网关目标无响应
};

/// 异常码的可读名字（英文，便于日志检索）。
[[nodiscard]] const char* exception_name(Exception e) noexcept;

// =============================================================================
//  协议限制
// =============================================================================

/// 单次读写数量的上限（Modbus 规范规定）。
struct Limits {
    /// 读线圈 / 离散输入：单次最多 2000 个。
    static constexpr std::uint16_t max_read_bits = 2000;

    /// 读寄存器：单次最多 125 个。
    static constexpr std::uint16_t max_read_registers = 125;

    /// 写多个线圈：单次最多 1968 个。
    static constexpr std::uint16_t max_write_bits = 1968;

    /// 写多个寄存器：单次最多 123 个。
    static constexpr std::uint16_t max_write_registers = 123;

    /// RTU 的 ADU 上限（地址 1 + PDU 253 + CRC 2）。
    static constexpr std::size_t max_rtu_adu = 256;

    /// TCP 的 ADU 上限（MBAP 7 + PDU 253）。
    static constexpr std::size_t max_tcp_adu = 260;

    /// PDU 上限（功能码 1 + 数据 252）。
    static constexpr std::size_t max_pdu = 253;
};

// =============================================================================
//  结果
// =============================================================================

/// 一次请求的结果。
///
/// 分两类失败，**必须区分**：
///  - **传输层失败**（`ec` 非空）：超时、串口错误、连接断了。请求可能根本没到。
///  - **协议层异常**（`exception != none`）：从站收到了请求但拒绝。请求到了。
///
/// 两者的重试策略不同：传输失败可以盲目重试；协议异常重试也没用（除非是
/// `server_device_busy`）。
struct Result {
    std::error_code ec{};                 ///< 传输层错误（空 = 传输成功）
    Exception       exception = Exception::none;   ///< 从站返回的异常码

    [[nodiscard]] bool ok() const noexcept
    {
        return !ec && exception == Exception::none;
    }

    /// 是否值得重试。传输失败、或从站忙 → true。
    [[nodiscard]] bool retryable() const noexcept;

    /// 人类可读的说明（日志用）。
    [[nodiscard]] std::string describe() const;
};

// =============================================================================
//  错误码
// =============================================================================

enum class ModbusErrc {
    ok = 0,
    invalid_argument,     ///< 参数非法（数量超限、地址越界、单元号非法）
    frame_too_short,      ///< 帧长度不足
    frame_too_long,       ///< 帧长度超限
    crc_mismatch,         ///< CRC 校验失败（RTU）
    bad_protocol_id,      ///< MBAP 的协议标识不是 0（TCP）
    bad_transaction,      ///< 事务 ID 不匹配（TCP）
    bad_unit_id,          ///< 单元 ID 不匹配
    bad_length,           ///< 长度字段与实际不符
    unexpected_response,  ///< 响应的功能码/格式对不上请求
    timeout,              ///< 超时
    not_connected,        ///< 未连接
    io_error,             ///< 传输层错误
};

[[nodiscard]] const std::error_category& modbus_category() noexcept;
[[nodiscard]] std::error_code make_error_code(ModbusErrc e) noexcept;

} // namespace modbusmech

namespace std {
template <>
struct is_error_code_enum<modbusmech::ModbusErrc> : true_type {};
} // namespace std
