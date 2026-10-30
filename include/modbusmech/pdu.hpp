// =============================================================================
//  modbusmech - PDU 编解码（纯函数，无 IO）
// =============================================================================
//  文件    : modbusmech/pdu.hpp
//  用途    : 把"功能码请求/响应"和字节序列互转。**不碰网络、不碰串口**。
//  参考    : Modbus Application Protocol V1.1b3
//
//  ★ 为什么单独一层
//  这是整个库里唯一"纯"的部分：输入字节、输出结构，没有副作用。所有协议
//  细节的正确性都靠这一层的测试兜住——上层的传输问题、从站的业务问题不该
//  让这里的 bug 混进来。
//
//  ★ PDU 与 ADU
//    PDU = 功能码 + 数据                    （协议层，本文件）
//    ADU = RTU: 地址 + PDU + CRC16
//          TCP: MBAP(7) + PDU              （传输层，见 rtu.hpp / tcp.hpp）
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <system_error>
#include <vector>

#include <modbusmech/common.hpp>
#include <modbusmech/span.hpp>

namespace modbusmech {

// =============================================================================
//  CRC16（RTU 用）
// =============================================================================

/// Modbus CRC16（多项式 0xA001，初值 0xFFFF）。
///
/// @param[in] data 数据（**不含** CRC 本身）
/// @return CRC 值（小端写入帧尾：低字节在前）
[[nodiscard]] std::uint16_t crc16(span<const std::uint8_t> data) noexcept;

// =============================================================================
//  请求 / 响应的结构
// =============================================================================

/// 功能码 01/02 的请求：读位。
struct ReadBitsRequest {
    std::uint16_t address = 0;
    std::uint16_t count   = 0;   ///< 1..2000
};

/// 功能码 03/04 的请求：读寄存器。
struct ReadRegistersRequest {
    std::uint16_t address = 0;
    std::uint16_t count   = 0;   ///< 1..125
};

/// 功能码 05 的请求：写单个线圈。
struct WriteCoilRequest {
    std::uint16_t address = 0;
    bool          value   = false;
};

/// 功能码 06 的请求：写单个寄存器。
struct WriteRegisterRequest {
    std::uint16_t address = 0;
    std::uint16_t value   = 0;
};

/// 功能码 0F 的请求：写多个线圈。
struct WriteCoilsRequest {
    std::uint16_t      address = 0;
    std::vector<bool>  values;               ///< 每个元素一个线圈，size 即数量（1..1968）
};

/// 功能码 10 的请求：写多个寄存器。
struct WriteRegistersRequest {
    std::uint16_t              address = 0;
    std::uint16_t              count   = 0;   ///< 1..123
    std::vector<std::uint16_t> values;
};

// =============================================================================
//  编码（结构 → PDU 字节）
// =============================================================================

/// 编码读位请求（功能码 01/02）。
///
/// @param[in] function 只能是 read_coils 或 read_discrete_inputs
/// @param[out] out     PDU 字节（会被清空后重填）
/// @return 成功返回空 error_code；数量越界返回 invalid_argument
[[nodiscard]] std::error_code encode_pdu(Function f, const ReadBitsRequest& req,
                                         std::vector<std::uint8_t>& out) noexcept;

[[nodiscard]] std::error_code encode_pdu(Function f, const ReadRegistersRequest& req,
                                         std::vector<std::uint8_t>& out) noexcept;

[[nodiscard]] std::error_code encode_pdu(Function f, const WriteCoilRequest& req,
                                         std::vector<std::uint8_t>& out) noexcept;

[[nodiscard]] std::error_code encode_pdu(Function f, const WriteRegisterRequest& req,
                                         std::vector<std::uint8_t>& out) noexcept;

[[nodiscard]] std::error_code encode_pdu(Function f, const WriteCoilsRequest& req,
                                         std::vector<std::uint8_t>& out) noexcept;

[[nodiscard]] std::error_code encode_pdu(Function f, const WriteRegistersRequest& req,
                                         std::vector<std::uint8_t>& out) noexcept;

/// 编码异常响应（从站用）：功能码 | 0x80，后跟异常码。
[[nodiscard]] std::error_code encode_exception(Function f, Exception e,
                                               std::vector<std::uint8_t>& out) noexcept;

// =============================================================================
//  解码（PDU 字节 → 结构）
// =============================================================================

/// 解码后的 PDU 类型。
enum class PduKind {
    request,     ///< 正常请求
    exception,   ///< 异常响应（功能码最高位为 1）
};

/// 解码结果：先判断是请求还是异常，再按功能码取具体字段。
struct DecodedPdu {
    PduKind  kind      = PduKind::request;
    Function function  = Function::read_coils;
    Exception exception = Exception::none;

    // 请求字段（按 function 取用）
    std::uint16_t              address = 0;
    std::uint16_t              count   = 0;
    std::uint16_t              value   = 0;      ///< 05/06 的写入值
    std::vector<bool>          bits;             ///< 0F 的线圈值（解包后）
    std::vector<std::uint16_t> registers;        ///< 10 的寄存器值
};

/// 解码一个 PDU。
///
/// @param[in] pdu 完整 PDU（第一个字节是功能码）
/// @return 失败时 ec 说明原因（太短 / 长度不符 / 未知功能码）
[[nodiscard]] std::error_code decode_pdu(span<const std::uint8_t> pdu,
                                         DecodedPdu& out) noexcept;

// =============================================================================
//  响应解析
// =============================================================================

/// 解析"读位/读寄存器"的响应。
///
/// @param[in] pdu  完整响应 PDU
/// @param[in] f    请求用的功能码（必须与响应一致）
/// @param[in] want 期望的元素个数（用于校验字节数）
/// @param[out] bits   读位的结果（f 为 01/02 时填）
/// @param[out] regs   读寄存器的结果（f 为 03/04 时填）
/// @param[out] ex     从站返回的异常（若非 none，其余输出无意义）
///
/// @return 传输/格式错误；异常码在 ex 里单独返回（不算 error_code）
[[nodiscard]] std::error_code parse_read_response(
    span<const std::uint8_t> pdu, Function f, std::uint16_t want,
    std::vector<bool>& bits, std::vector<std::uint16_t>& regs,
    Exception& ex) noexcept;

/// 解析"写单个/多个"的响应（回显校验）。
///
/// @param[in] pdu  完整响应 PDU
/// @param[in] f    请求用的功能码
/// @param[in] addr 请求的起始地址
/// @param[in] cnt  请求的数量（05/06 传 1）
/// @param[out] ex  异常码
[[nodiscard]] std::error_code parse_write_response(
    span<const std::uint8_t> pdu, Function f,
    std::uint16_t addr, std::uint16_t cnt, Exception& ex) noexcept;

} // namespace modbusmech
