// ============================================================================
//  test_pdu.cpp —— PDU 编解码测试
//
//  全部用 Modbus 规范（V1.1b3）里的标准例子逐字节比对。协议层的 bug 一旦
//  流到上层就很难定位，所以这里宁可写得啰嗦。
// ============================================================================
#include <modbusmech/pdu.hpp>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <cstdint>

#include "test_framework.hpp"

using namespace modbusmech;
using namespace netmech_test;

namespace {

using Bytes = std::vector<std::uint8_t>;

/// 把 vector 包成 span。
span<const std::uint8_t> sp(const Bytes& v)
{
    return span<const std::uint8_t>(v.data(), v.size());
}

} // namespace

// ---------------------------------------------------------------------------
// 1. CRC16：用规范里的已知值
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_crc16_known_values)
{
    // 01 03 00 00 00 0A  →  CRC = 0xCDC5（线上低字节先：C5 CD）
    const Bytes a{0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
    CHECK_EQ(crc16(sp(a)), 0xCDC5);

    // 01 03 00 00 00 01  →  0x0A84
    const Bytes b{0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    CHECK_EQ(crc16(sp(b)), 0x0A84);

    // 空数据 → 初值
    CHECK_EQ(crc16(span<const std::uint8_t>()), 0xFFFF);
}

// ---------------------------------------------------------------------------
// 2. 读保持寄存器请求：03 00 00 00 0A（规范原文的例子）
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_encode_read_holding_registers)
{
    Bytes pdu;
    const auto ec = encode_pdu(Function::read_holding_registers,
                               ReadRegistersRequest{0x0000, 10}, pdu);
    CHECK(!ec);
    REQUIRE(pdu.size() == 5u);
    CHECK_EQ(pdu[0], 0x03);
    CHECK_EQ(pdu[1], 0x00);
    CHECK_EQ(pdu[2], 0x00);
    CHECK_EQ(pdu[3], 0x00);
    CHECK_EQ(pdu[4], 0x0A);
}

// ---------------------------------------------------------------------------
// 3. 读线圈请求：01 00 13 00 25（规范原文：地址 0x13，37 个）
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_encode_read_coils)
{
    Bytes pdu;
    REQUIRE(!encode_pdu(Function::read_coils, ReadBitsRequest{0x0013, 0x25}, pdu));
    REQUIRE(pdu.size() == 5u);
    const Bytes want{0x01, 0x00, 0x13, 0x00, 0x25};
    CHECK(std::memcmp(pdu.data(), want.data(), 5) == 0);
}

// ---------------------------------------------------------------------------
// 4. 写单寄存器：06 00 01 00 03
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_encode_write_single_register)
{
    Bytes pdu;
    REQUIRE(!encode_pdu(Function::write_single_register,
                        WriteRegisterRequest{0x0001, 0x0003}, pdu));
    const Bytes want{0x06, 0x00, 0x01, 0x00, 0x03};
    CHECK(pdu == want);
}

// ---------------------------------------------------------------------------
// 5. 写单线圈：开的编码是 FF00，关是 0000
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_encode_write_single_coil)
{
    Bytes on;
    REQUIRE(!encode_pdu(Function::write_single_coil, WriteCoilRequest{0x00AC, true}, on));
    const Bytes want_on{0x05, 0x00, 0xAC, 0xFF, 0x00};
    CHECK(on == want_on);

    Bytes off;
    REQUIRE(!encode_pdu(Function::write_single_coil, WriteCoilRequest{0x00AC, false}, off));
    const Bytes want_off{0x05, 0x00, 0xAC, 0x00, 0x00};
    CHECK(off == want_off);
}

// ---------------------------------------------------------------------------
// 6. 写多寄存器（功能码 10）
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_encode_write_multiple_registers)
{
    Bytes pdu;
    WriteRegistersRequest req;
    req.address = 0x0001;
    req.values  = {0x000A, 0x0102};
    REQUIRE(!encode_pdu(Function::write_multiple_registers, req, pdu));

    // 10 00 01 00 02 04 00 0A 01 02
    const Bytes want{0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02};
    CHECK(pdu == want);
}

// ---------------------------------------------------------------------------
// 7. 写多线圈 + 位打包（低位在前）
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_encode_write_multiple_coils_bit_order)
{
    Bytes pdu;
    WriteCoilsRequest req;
    req.address = 0x0013;
    // 10 个线圈：第 0、3、7、8、9 个为 true
    req.values  = {true, false, false, true, false, false, false, true, true, true};
    REQUIRE(!encode_pdu(Function::write_multiple_coils, req, pdu));

    // 逐位算（低位在前）：
    //   byte0: bit0=T, bit3=T, bit7=T → 1000 1001 = 0x89
    //   byte1: bit0=T, bit1=T（即第 8、9 个）→ 0000 0011 = 0x03
    // 功能码是 0F（写多线圈），不是 10（写多寄存器）
    const Bytes want{0x0F, 0x00, 0x13, 0x00, 0x0A, 0x02, 0x89, 0x03};
    CHECK(pdu == want);
}

// ---------------------------------------------------------------------------
// 8. 数量超限被拒
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_rejects_count_overflow)
{
    Bytes pdu;
    // 126 > 125（上限）
    CHECK(encode_pdu(Function::read_holding_registers,
                     ReadRegistersRequest{0, 126}, pdu) == ModbusErrc::invalid_argument);

    CHECK(encode_pdu(Function::read_coils,
                     ReadBitsRequest{0, 2001}, pdu) == ModbusErrc::invalid_argument);  // 2001 > 2000

    // 数量 0 也不行
    CHECK(encode_pdu(Function::read_holding_registers,
                     ReadRegistersRequest{0, 0}, pdu) == ModbusErrc::invalid_argument);
}

// ---------------------------------------------------------------------------
// 9. 解码请求
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_decode_request)
{
    const Bytes pdu{0x03, 0x00, 0x10, 0x00, 0x05};
    DecodedPdu d;
    REQUIRE(!decode_pdu(sp(pdu), d));
    CHECK(d.kind == PduKind::request);
    CHECK(d.function == Function::read_holding_registers);
    CHECK_EQ(d.address, 0x0010);
    CHECK_EQ(d.count, 5);
}

// ---------------------------------------------------------------------------
// 10. 解码异常响应
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_decode_exception)
{
    const Bytes pdu{0x83, 0x02};   // 功能码 03 | 0x80，异常码 02
    DecodedPdu d;
    REQUIRE(!decode_pdu(sp(pdu), d));
    CHECK(d.kind == PduKind::exception);
    CHECK(d.function == Function::read_holding_registers);
    CHECK(d.exception == Exception::illegal_data_address);
}

// ---------------------------------------------------------------------------
// 11. 解析读寄存器响应
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_parse_read_registers_response)
{
    const Bytes pdu{0x03, 0x04, 0x00, 0x0A, 0x01, 0x02};   // 2 个寄存器
    std::vector<std::uint16_t> regs;
    std::vector<bool> bits;
    Exception ex = Exception::illegal_function;

    REQUIRE(!parse_read_response(sp(pdu), Function::read_holding_registers, 2,
                                 bits, regs, ex));
    CHECK(ex == Exception::none);
    REQUIRE(regs.size() == 2u);
    CHECK_EQ(regs[0], 0x000A);
    CHECK_EQ(regs[1], 0x0102);
}

// ---------------------------------------------------------------------------
// 12. 解析读线圈响应（位解包）
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_parse_read_coils_response)
{
    // 01 02 89 03：10 个位
    //   0x89 = 1000 1001 → bit0,bit3,bit7
    //   0x03 = 0000 0011 → bit8,bit9
    const Bytes pdu{0x01, 0x02, 0x89, 0x03};
    std::vector<bool> bits;
    std::vector<std::uint16_t> regs;
    Exception ex;

    REQUIRE(!parse_read_response(sp(pdu), Function::read_coils, 10, bits, regs, ex));
    CHECK(ex == Exception::none);
    REQUIRE(bits.size() == 10u);
    CHECK(bits[0]);      // 0x89 bit0
    CHECK(!bits[1]);
    CHECK(!bits[2]);
    CHECK(bits[3]);      // 0x89 bit3
    CHECK(!bits[4]);
    CHECK(!bits[5]);
    CHECK(!bits[6]);
    CHECK(bits[7]);      // 0x89 bit7
    CHECK(bits[8]);      // 0x03 bit0 → 第 8 个
    CHECK(bits[9]);      // 0x03 bit1 → 第 9 个
}

// ---------------------------------------------------------------------------
// 13. 响应里的异常码被正确取出
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_parse_exception_response)
{
    const Bytes pdu{0x83, 0x04};   // 功能码 03 | 0x80，异常码 04（从站故障）
    std::vector<std::uint16_t> regs;
    std::vector<bool> bits;
    Exception ex = Exception::none;

    REQUIRE(!parse_read_response(sp(pdu), Function::read_holding_registers, 2,
                                 bits, regs, ex));
    CHECK(ex == Exception::server_device_failure);
    CHECK(regs.empty());
}

// ---------------------------------------------------------------------------
// 14. 字节数不符 → bad_length
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_parse_wrong_byte_count)
{
    // 请求 2 个寄存器，但响应说 3 字节（不是偶数，也不等于 4）
    const Bytes pdu{0x03, 0x03, 0x00, 0x0A, 0x01};
    std::vector<std::uint16_t> regs;
    std::vector<bool> bits;
    Exception ex;
    const auto ec = parse_read_response(sp(pdu), Function::read_holding_registers, 2,
                                        bits, regs, ex);
    CHECK(ec == ModbusErrc::bad_length);
}

// ---------------------------------------------------------------------------
// 15. 功能码对不上 → unexpected_response
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_parse_mismatched_function)
{
    const Bytes pdu{0x04, 0x02, 0x00, 0x01};   // 回的是 04，但请求的是 03
    std::vector<std::uint16_t> regs;
    std::vector<bool> bits;
    Exception ex;
    const auto ec = parse_read_response(sp(pdu), Function::read_holding_registers, 1,
                                        bits, regs, ex);
    CHECK(ec == ModbusErrc::unexpected_response);
}

// ---------------------------------------------------------------------------
// 16. 写响应回显校验
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_parse_write_response_echo)
{
    const Bytes ok{0x06, 0x00, 0x01, 0x00, 0x03};
    Exception ex;
    CHECK(!parse_write_response(sp(ok), Function::write_single_register, 0x0001, 1, ex));
    CHECK(ex == Exception::none);

    // 回显的地址不对
    const Bytes bad{0x06, 0x00, 0x02, 0x00, 0x03};
    ex = Exception::none;
    CHECK(parse_write_response(sp(bad), Function::write_single_register, 0x0001, 1, ex)
          == ModbusErrc::unexpected_response);
}

// ---------------------------------------------------------------------------
// 17. 空/过短的 PDU 不崩
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_rejects_short_input)
{
    DecodedPdu d;
    CHECK(decode_pdu(span<const std::uint8_t>(), d) == ModbusErrc::frame_too_short);

    const Bytes tiny{0x03, 0x00};   // 只有 2 字节，读请求要 5
    CHECK(decode_pdu(sp(tiny), d) == ModbusErrc::frame_too_short);

    std::vector<std::uint16_t> regs;
    std::vector<bool> bits;
    Exception ex;
    CHECK(parse_read_response(span<const std::uint8_t>(),
                              Function::read_holding_registers, 1, bits, regs, ex)
          == ModbusErrc::frame_too_short);
}

// ---------------------------------------------------------------------------
// 18. 编码异常响应
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_encode_exception_response)
{
    Bytes pdu;
    REQUIRE(!encode_exception(Function::read_holding_registers,
                              Exception::illegal_data_address, pdu));
    const Bytes want{0x83, 0x02};
    CHECK(pdu == want);
}

// ---------------------------------------------------------------------------
// 19. decode_pdu：0F/10 的 byte count 必须与 count 精确匹配
//
//     回归：以前只校验"缓冲够长"，bc 比 count 少时缺的位会被 unpack_bits
//     静默解成 0，从站于是把那段地址的线圈/寄存器真的写成 0（协议规定畸形
//     请求不能执行写）。现在少一个字节就判 bad_length，从站丢弃并不执行。
// ---------------------------------------------------------------------------
NETMECH_TEST(pdu_decode_rejects_bc_count_mismatch)
{
    DecodedPdu d;

    // 0F：count=16 需要 2 字节数据，bc=1 ⇒ 拒
    const Bytes coils_short{0x0F, 0x00, 0x00, 0x00, 0x10, 0x01, 0xFF};
    CHECK(decode_pdu(sp(coils_short), d) == ModbusErrc::bad_length);

    // 0F：count=10 需要 2 字节，bc=2 ⇒ 收（合法，别误杀）
    const Bytes coils_ok{0x0F, 0x00, 0x00, 0x00, 0x0A, 0x02, 0xFF, 0x03};
    CHECK(!decode_pdu(sp(coils_ok), d));
    CHECK_EQ(d.count, 10);
    CHECK_EQ(d.bits.size(), std::size_t{10});

    // 10：count=2 需要 4 字节数据，bc=2 ⇒ 拒
    const Bytes regs_short{0x10, 0x00, 0x00, 0x00, 0x02, 0x02, 0x12, 0x34};
    CHECK(decode_pdu(sp(regs_short), d) == ModbusErrc::bad_length);

    // 10：count=2，bc=4 ⇒ 收
    const Bytes regs_ok{0x10, 0x00, 0x00, 0x00, 0x02, 0x04,
                        0x12, 0x34, 0x56, 0x78};
    CHECK(!decode_pdu(sp(regs_ok), d));
    CHECK_EQ(d.registers.size(), std::size_t{2});
    CHECK_EQ(d.registers[0], 0x1234);
}
