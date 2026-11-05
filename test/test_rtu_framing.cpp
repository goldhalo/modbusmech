// ============================================================================
//  test_rtu_framing.cpp —— RTU 组帧 / 长度推算（**纯逻辑，跨平台**）
//
//  为什么单独一个文件：RTU 的端到端测试靠 pty（Linux only），Windows CI 上
//  RTU 路径完全没有覆盖。而历史上有两个真 bug 恰好都在这一层能测出来：
//    · 读类响应的长度要靠第 3 字节（byte count），只等到 2 字节就算长度
//      ⇒ 9600bps 真串口上"读类请求随机失败"，pty 因为一次到齐测不出来；
//    · CRC 低字节在前（与其余字段相反），写反了谁都连不上。
//  这两个都不需要真串口。
// ============================================================================
#include <modbusmech/pdu.hpp>

#include "rtu.hpp"
#include "wire.hpp"

#include <cstdint>
#include <vector>

#include "test_framework.hpp"

using namespace modbusmech;
using namespace modbusmech::detail;
using namespace netmech_test;

// ---------------------------------------------------------------------------
// 1. 读类响应：必须等第 3 字节（byte count）才能算长度
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_body_len_read_needs_byte_count)
{
    // 地址=1 功能码=03 字节数=4 数据(4字节) ...
    const std::uint8_t frame[] = {0x01, 0x03, 0x04, 0x00, 0x0A, 0x00, 0x0B, 0, 0};

    // 只有 2 字节：**算不出来**，必须返回 0（调用方要等第 3 字节）
    CHECK_EQ(rtu_response_body_len(frame, 2), std::size_t{0});

    // 3 字节起可算：地址+功能码+字节数+数据 = 3 + 4 = 7
    CHECK_EQ(rtu_response_body_len(frame, 3), std::size_t{7});
    CHECK_EQ(rtu_response_body_len(frame, 9), std::size_t{7});
}

// ---------------------------------------------------------------------------
// 2. 异常响应固定 3 字节（地址 + 功能码|0x80 + 异常码）
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_body_len_exception_is_three)
{
    const std::uint8_t frame[] = {0x01, 0x83, 0x02};
    CHECK_EQ(rtu_response_body_len(frame, 2), std::size_t{3});
    CHECK_EQ(rtu_response_body_len(frame, 3), std::size_t{3});
}

// ---------------------------------------------------------------------------
// 3. 写类响应固定 6 字节（地址 + 功能码 + 地址(2) + 值/数量(2)）
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_body_len_write_is_six)
{
    const std::uint8_t single[] = {0x01, 0x06, 0x00, 0x10, 0x00, 0x2A};
    const std::uint8_t multi[]  = {0x01, 0x10, 0x00, 0x10, 0x00, 0x02};
    CHECK_EQ(rtu_response_body_len(single, 2), std::size_t{6});
    CHECK_EQ(rtu_response_body_len(multi, 6), std::size_t{6});
}

// ---------------------------------------------------------------------------
// 4. 不认识的功能码返回 0（调用方据此判 unexpected_response）
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_body_len_unknown_func_is_zero)
{
    const std::uint8_t frame[] = {0x01, 0x41, 0x00, 0x00};
    CHECK_EQ(rtu_response_body_len(frame, 4), std::size_t{0});
    CHECK_EQ(rtu_response_body_len(frame, 1), std::size_t{0});
}

// ---------------------------------------------------------------------------
// 5. wrap_rtu：地址在最前、CRC 在最后且**低字节在前**
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_wrap_puts_addr_first_and_crc_little_endian)
{
    const std::uint8_t pdu[] = {0x03, 0x00, 0x00, 0x00, 0x02};
    const auto frame = wrap_rtu(0x07, span<const std::uint8_t>(pdu, sizeof pdu));

    REQUIRE(frame.size() == sizeof pdu + 3);
    CHECK_EQ(frame[0], 0x07);                       // 地址
    CHECK_EQ(frame[1], 0x03);                       // 功能码
    CHECK_EQ(frame[frame.size() - 2], static_cast<std::uint8_t>(crc16(
                 span<const std::uint8_t>(frame.data(), frame.size() - 2)) & 0xFF));
    CHECK_EQ(frame[frame.size() - 1], static_cast<std::uint8_t>(
                 (crc16(span<const std::uint8_t>(frame.data(), frame.size() - 2)) >> 8) & 0xFF));

    // 一个已知向量：01 03 00 00 00 0A → CRC = 0xCDC5（低字节 0xC5 在前）
    const std::uint8_t pdu2[] = {0x03, 0x00, 0x00, 0x00, 0x0A};
    const auto frame2 = wrap_rtu(0x01, span<const std::uint8_t>(pdu2, sizeof pdu2));
    REQUIRE(frame2.size() == 8);
    CHECK_EQ(frame2[6], 0xC5);
    CHECK_EQ(frame2[7], 0xCD);
}

// ---------------------------------------------------------------------------
// 6. 大端工具：put/get 必须对称且是大端
// ---------------------------------------------------------------------------
NETMECH_TEST(wire_u16_is_big_endian)
{
    std::vector<std::uint8_t> v;
    put_u16(v, 0x1234);
    REQUIRE(v.size() == 2);
    CHECK_EQ(v[0], 0x12);   // 高字节先
    CHECK_EQ(v[1], 0x34);
    CHECK_EQ(get_u16(v.data()), 0x1234);
    CHECK_EQ(get_u16(span<const std::uint8_t>(v.data(), v.size()), 0), 0x1234);
}
