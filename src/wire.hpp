// =============================================================================
//  wire.hpp —— 线上格式的小工具（内部头）
// =============================================================================
//  Modbus 全部是**大端**（高字节先）：
//    * PDU 里的地址/数量/寄存器值；
//    * TCP 的 MBAP 字段；
//    * RTU 的 CRC 是**低字节先**（唯一反例），所以单独写清楚。
//
//  以前 `put_u16/get_u16/wrap_rtu` 在 pdu.cpp、tcp.cpp、slave.cpp、master.cpp
//  各写了一份（拷贝粘贴出来的四份），改一处忘三处是迟早的事。统一到这里。
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <modbusmech/pdu.hpp>   // crc16
#include <modbusmech/span.hpp>

namespace modbusmech {
namespace detail {

/// 追加一个大端 uint16。
inline void put_u16(std::vector<std::uint8_t>& v, std::uint16_t x)
{
    v.push_back(static_cast<std::uint8_t>(x >> 8));
    v.push_back(static_cast<std::uint8_t>(x & 0xFF));
}

/// 读大端 uint16（调用方保证至少有 2 字节）。
[[nodiscard]] inline std::uint16_t get_u16(const std::uint8_t* p) noexcept
{
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
}

/// 从 span 的 @p off 处读大端 uint16。
[[nodiscard]] inline std::uint16_t get_u16(span<const std::uint8_t> d,
                                           std::size_t              off) noexcept
{
    return get_u16(d.data() + off);
}

/// 把 PDU 包成 RTU 帧：`地址(1) | PDU | CRC(2)`。
///
/// @note CRC **低字节在前** —— 与其余所有字段相反，是最容易写错的一处。
[[nodiscard]] inline std::vector<std::uint8_t> wrap_rtu(std::uint8_t              unit,
                                                        span<const std::uint8_t> pdu)
{
    std::vector<std::uint8_t> frame(1 + pdu.size() + 2);
    frame[0] = unit;
    for (std::size_t i = 0; i < pdu.size(); ++i) {
        frame[1 + i] = pdu[i];
    }

    const std::uint16_t crc =
        crc16(span<const std::uint8_t>(frame.data(), frame.size() - 2));
    frame[frame.size() - 2] = static_cast<std::uint8_t>(crc & 0xFF);          // 低字节先
    frame[frame.size() - 1] = static_cast<std::uint8_t>((crc >> 8) & 0xFF);
    return frame;
}

} // namespace detail
} // namespace modbusmech
