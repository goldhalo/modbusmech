// =============================================================================
//  pdu.cpp —— PDU 编解码实现
// =============================================================================
//  所有函数都是纯的：不碰网络、不碰时间、不抛异常。出错一律走 error_code。
// =============================================================================
#include <modbusmech/pdu.hpp>

#include <cstring>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <system_error>

namespace modbusmech {

namespace {

void put_u16(std::vector<std::uint8_t>& v, std::uint16_t x)
{
    v.push_back(static_cast<std::uint8_t>(x >> 8));    // 大端
    v.push_back(static_cast<std::uint8_t>(x & 0xFF));
}

std::uint16_t get_u16(span<const std::uint8_t> d, std::size_t off) noexcept
{
    return static_cast<std::uint16_t>((d[off] << 8) | d[off + 1]);
}

/// 位打包：每字节 8 个位，低位在前（Modbus 规范）。
void pack_bits(const std::vector<bool>& bits, std::vector<std::uint8_t>& out)
{
    const std::size_t nbytes = (bits.size() + 7) / 8;
    out.resize(nbytes, 0);
    for (std::size_t i = 0; i < bits.size(); ++i) {
        if (bits[i]) {
            out[i / 8] = static_cast<std::uint8_t>(out[i / 8] | (1u << (i % 8)));
        }
    }
}

void unpack_bits(span<const std::uint8_t> data, std::size_t count,
                 std::vector<bool>& out)
{
    out.assign(count, false);
    for (std::size_t i = 0; i < count && i / 8 < data.size(); ++i) {
        out[i] = ((data[i / 8] >> (i % 8)) & 1u) != 0;
    }
}

} // namespace

// =============================================================================
//  CRC16
// =============================================================================

std::uint16_t crc16(span<const std::uint8_t> data) noexcept
{
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < data.size(); ++i) {
        crc = static_cast<std::uint16_t>(crc ^ data[i]);
        for (int b = 0; b < 8; ++b) {
            if (crc & 1u) {
                crc = static_cast<std::uint16_t>((crc >> 1) ^ 0xA001u);
            } else {
                crc = static_cast<std::uint16_t>(crc >> 1);
            }
        }
    }
    return crc;
}

// =============================================================================
//  编码
// =============================================================================

std::error_code encode_pdu(Function f, const ReadBitsRequest& req,
                           std::vector<std::uint8_t>& out) noexcept
{
    if (f != Function::read_coils && f != Function::read_discrete_inputs) {
        return make_error_code(ModbusErrc::invalid_argument);
    }
    if (req.count == 0 || req.count > Limits::max_read_bits) {
        return make_error_code(ModbusErrc::invalid_argument);
    }

    out.clear();
    out.push_back(static_cast<std::uint8_t>(f));
    put_u16(out, req.address);
    put_u16(out, req.count);
    return {};
}

std::error_code encode_pdu(Function f, const ReadRegistersRequest& req,
                           std::vector<std::uint8_t>& out) noexcept
{
    if (f != Function::read_holding_registers && f != Function::read_input_registers) {
        return make_error_code(ModbusErrc::invalid_argument);
    }
    if (req.count == 0 || req.count > Limits::max_read_registers) {
        return make_error_code(ModbusErrc::invalid_argument);
    }

    out.clear();
    out.push_back(static_cast<std::uint8_t>(f));
    put_u16(out, req.address);
    put_u16(out, req.count);
    return {};
}

std::error_code encode_pdu(Function f, const WriteCoilRequest& req,
                           std::vector<std::uint8_t>& out) noexcept
{
    if (f != Function::write_single_coil) {
        return make_error_code(ModbusErrc::invalid_argument);
    }

    out.clear();
    out.push_back(static_cast<std::uint8_t>(f));
    put_u16(out, req.address);
    put_u16(out, req.value ? 0xFF00u : 0x0000u);   // 规范：开 = FF00，关 = 0000
    return {};
}

std::error_code encode_pdu(Function f, const WriteRegisterRequest& req,
                           std::vector<std::uint8_t>& out) noexcept
{
    if (f != Function::write_single_register) {
        return make_error_code(ModbusErrc::invalid_argument);
    }

    out.clear();
    out.push_back(static_cast<std::uint8_t>(f));
    put_u16(out, req.address);
    put_u16(out, req.value);
    return {};
}

std::error_code encode_pdu(Function f, const WriteCoilsRequest& req,
                           std::vector<std::uint8_t>& out) noexcept
{
    if (f != Function::write_multiple_coils) {
        return make_error_code(ModbusErrc::invalid_argument);
    }
    if (req.values.empty() || req.values.size() > Limits::max_write_bits) {
        return make_error_code(ModbusErrc::invalid_argument);
    }

    const std::uint16_t count = static_cast<std::uint16_t>(req.values.size());
    const std::size_t   nbytes = (count + 7) / 8;

    out.clear();
    out.push_back(static_cast<std::uint8_t>(f));
    put_u16(out, req.address);
    put_u16(out, count);
    out.push_back(static_cast<std::uint8_t>(nbytes));

    std::vector<std::uint8_t> packed;
    pack_bits(req.values, packed);
    out.insert(out.end(), packed.begin(), packed.begin() + nbytes);
    return {};
}

std::error_code encode_pdu(Function f, const WriteRegistersRequest& req,
                           std::vector<std::uint8_t>& out) noexcept
{
    if (f != Function::write_multiple_registers) {
        return make_error_code(ModbusErrc::invalid_argument);
    }
    if (req.values.empty() || req.values.size() > Limits::max_write_registers) {
        return make_error_code(ModbusErrc::invalid_argument);
    }

    const std::uint16_t count = static_cast<std::uint16_t>(req.values.size());

    out.clear();
    out.push_back(static_cast<std::uint8_t>(f));
    put_u16(out, req.address);
    put_u16(out, count);
    out.push_back(static_cast<std::uint8_t>(count * 2));   // 字节数
    for (std::uint16_t v : req.values) {
        put_u16(out, v);
    }
    return {};
}

std::error_code encode_exception(Function f, Exception e,
                                 std::vector<std::uint8_t>& out) noexcept
{
    if (e == Exception::none) {
        return make_error_code(ModbusErrc::invalid_argument);
    }
    out.clear();
    out.push_back(static_cast<std::uint8_t>(static_cast<std::uint8_t>(f) | 0x80u));
    out.push_back(static_cast<std::uint8_t>(e));
    return {};
}

// =============================================================================
//  解码
// =============================================================================

std::error_code decode_pdu(span<const std::uint8_t> pdu, DecodedPdu& out) noexcept
{
    out = DecodedPdu{};

    if (pdu.empty()) {
        return make_error_code(ModbusErrc::frame_too_short);
    }

    const std::uint8_t raw = pdu[0];

    // 异常响应：功能码最高位为 1，后跟异常码
    if (raw & 0x80u) {
        if (pdu.size() < 2) {
            return make_error_code(ModbusErrc::frame_too_short);
        }
        out.kind      = PduKind::exception;
        out.function  = static_cast<Function>(raw & 0x7Fu);
        out.exception = static_cast<Exception>(pdu[1]);
        return {};
    }

    out.kind     = PduKind::request;
    out.function = static_cast<Function>(raw);

    switch (out.function) {
    case Function::read_coils:
    case Function::read_discrete_inputs:
        if (pdu.size() < 5) {
            return make_error_code(ModbusErrc::frame_too_short);
        }
        // 规范定长 5 字节：尾部多余字节是畸形帧，不能接受
        // （旧实现只查下限，05/06 从站会把尾部垃圾原样回显给对端）。
        if (pdu.size() != 5) {
            return make_error_code(ModbusErrc::bad_length);
        }
        out.address = get_u16(pdu, 1);
        out.count   = get_u16(pdu, 3);
        break;

    case Function::read_holding_registers:
    case Function::read_input_registers:
        if (pdu.size() < 5) {
            return make_error_code(ModbusErrc::frame_too_short);
        }
        if (pdu.size() != 5) {
            return make_error_code(ModbusErrc::bad_length);
        }
        out.address = get_u16(pdu, 1);
        out.count   = get_u16(pdu, 3);
        break;

    case Function::write_single_coil:
    case Function::write_single_register:
        if (pdu.size() < 5) {
            return make_error_code(ModbusErrc::frame_too_short);
        }
        if (pdu.size() != 5) {
            return make_error_code(ModbusErrc::bad_length);
        }
        out.address = get_u16(pdu, 1);
        out.value   = get_u16(pdu, 3);
        break;

    case Function::write_multiple_coils: {
        if (pdu.size() < 6) {
            return make_error_code(ModbusErrc::frame_too_short);
        }
        out.address = get_u16(pdu, 1);
        out.count   = get_u16(pdu, 3);
        const std::uint8_t bc = pdu[5];
        // ★ byte count 必须与 count **精确匹配**。
        //   以前只校验"缓冲够长"：bc < (count+7)/8 时，缺的那些位会被
        //   unpack_bits 静默解成 0，从站于是把这段地址的线圈真的写成 0
        //   （协议规定畸形请求要回 03/04，绝不能执行写）。
        if (bc == 0 || pdu.size() < 6u + bc
            || static_cast<std::size_t>(bc)
                   != (static_cast<std::size_t>(out.count) + 7) / 8) {
            return make_error_code(ModbusErrc::bad_length);
        }
        unpack_bits(pdu.subspan(6, bc), out.count, out.bits);
        break;
    }

    case Function::write_multiple_registers: {
        if (pdu.size() < 6) {
            return make_error_code(ModbusErrc::frame_too_short);
        }
        out.address = get_u16(pdu, 1);
        out.count   = get_u16(pdu, 3);
        const std::uint8_t bc = pdu[5];
        // 同上：bc 必须等于 count × 2，少一个都说明帧不完整，不能猜着写。
        if (bc == 0 || (bc % 2) != 0 || pdu.size() < 6u + bc
            || static_cast<std::size_t>(bc) != static_cast<std::size_t>(out.count) * 2u) {
            return make_error_code(ModbusErrc::bad_length);
        }
        out.registers.reserve(bc / 2);
        for (std::size_t i = 0; i < bc; i += 2) {
            out.registers.push_back(get_u16(pdu, 6 + i));
        }
        break;
    }

    default:
        return make_error_code(ModbusErrc::invalid_argument);
    }
    return {};
}

// =============================================================================
//  响应解析
// =============================================================================

std::error_code parse_read_response(span<const std::uint8_t> pdu, Function f,
                                    std::uint16_t want,
                                    std::vector<bool>& bits,
                                    std::vector<std::uint16_t>& regs,
                                    Exception& ex) noexcept
{
    ex = Exception::none;
    bits.clear();
    regs.clear();

    if (pdu.empty()) {
        return make_error_code(ModbusErrc::frame_too_short);
    }

    // 异常响应
    if (pdu[0] & 0x80u) {
        if (pdu.size() < 2) {
            return make_error_code(ModbusErrc::frame_too_short);
        }
        if (static_cast<std::uint8_t>(f) != (pdu[0] & 0x7Fu)) {
            return make_error_code(ModbusErrc::unexpected_response);
        }
        ex = static_cast<Exception>(pdu[1]);
        return {};
    }

    if (pdu[0] != static_cast<std::uint8_t>(f)) {
        return make_error_code(ModbusErrc::unexpected_response);
    }
    if (pdu.size() < 2) {
        return make_error_code(ModbusErrc::frame_too_short);
    }

    const std::uint8_t bc = pdu[1];
    if (pdu.size() < 2u + bc) {
        return make_error_code(ModbusErrc::bad_length);
    }

    switch (f) {
    case Function::read_coils:
    case Function::read_discrete_inputs: {
        const std::size_t need_bytes = (want + 7) / 8;
        if (bc != need_bytes) {
            return make_error_code(ModbusErrc::bad_length);
        }
        unpack_bits(pdu.subspan(2, bc), want, bits);
        break;
    }
    case Function::read_holding_registers:
    case Function::read_input_registers: {
        if (bc != want * 2) {
            return make_error_code(ModbusErrc::bad_length);
        }
        regs.reserve(want);
        for (std::size_t i = 0; i < want; ++i) {
            regs.push_back(get_u16(pdu, 2 + i * 2));
        }
        break;
    }
    default:
        return make_error_code(ModbusErrc::unexpected_response);
    }
    return {};
}

namespace {

/// 写响应解析的唯一实现。check_value=false 时跳过 05/06 值回显校验
/// （legacy 5 参重载拿不到请求值，保持旧行为）。
std::error_code parse_write_response_impl(span<const std::uint8_t> pdu, Function f,
                                          std::uint16_t addr, std::uint16_t cnt,
                                          std::uint16_t value, bool check_value,
                                          Exception& ex) noexcept
{
    ex = Exception::none;

    if (pdu.empty()) {
        return make_error_code(ModbusErrc::frame_too_short);
    }

    if (pdu[0] & 0x80u) {
        if (pdu.size() < 2) {
            return make_error_code(ModbusErrc::frame_too_short);
        }
        if (static_cast<std::uint8_t>(f) != (pdu[0] & 0x7Fu)) {
            return make_error_code(ModbusErrc::unexpected_response);
        }
        ex = static_cast<Exception>(pdu[1]);
        return {};
    }

    if (pdu[0] != static_cast<std::uint8_t>(f)) {
        return make_error_code(ModbusErrc::unexpected_response);
    }
    if (pdu.size() < 5) {
        return make_error_code(ModbusErrc::frame_too_short);
    }

    if (get_u16(pdu, 1) != addr) {
        return make_error_code(ModbusErrc::unexpected_response);
    }

    // ★ 回显的内容按功能码不同：
    //   05 / 06  → 地址 + **值**（写入的值，不是数量）
    //   0F / 10  → 地址 + **数量**
    // 所以要分别校验，不能统一按数量比。
    if (f == Function::write_multiple_coils
        || f == Function::write_multiple_registers) {
        if (get_u16(pdu, 3) != cnt) {
            return make_error_code(ModbusErrc::unexpected_response);
        }
    } else if (check_value
               && (f == Function::write_single_coil
                   || f == Function::write_single_register)) {
        // 值回显：规范要求响应原样回显请求值，不对即写未生效。
        if (get_u16(pdu, 3) != value) {
            return make_error_code(ModbusErrc::unexpected_response);
        }
    }
    return {};
}

} // namespace

std::error_code parse_write_response(span<const std::uint8_t> pdu, Function f,
                                     std::uint16_t addr, std::uint16_t cnt,
                                     Exception& ex) noexcept
{
    // legacy 重载：拿不到请求值，05/06 值回显不校验（保持旧行为）。
    return parse_write_response_impl(pdu, f, addr, cnt, 0, /*check_value=*/false, ex);
}

std::error_code parse_write_response(span<const std::uint8_t> pdu, Function f,
                                     std::uint16_t addr, std::uint16_t cnt,
                                     std::uint16_t value, Exception& ex) noexcept
{
    return parse_write_response_impl(pdu, f, addr, cnt, value, /*check_value=*/true, ex);
}

} // namespace modbusmech
