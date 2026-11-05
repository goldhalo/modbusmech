// =============================================================================
//  transport.hpp —— 传输层抽象（内部头）
// =============================================================================
//  Master 只依赖这个接口，不关心底下是串口还是 TCP。
//
//  两者的差别都在"怎么划出一帧"上：
//    RTU：靠"先读 2 字节，按功能码推长度，再读剩余" + CRC
//    TCP：靠 MBAP 的长度字段
//  所以 receive() 由具体实现自己决定读多少。
// =============================================================================
#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <system_error>
#include <vector>

#include <modbusmech/span.hpp>

namespace modbusmech {
namespace detail {

/// 一次"发请求、收一帧响应"的能力。
class Transport {
public:
    virtual ~Transport() = default;

    [[nodiscard]] virtual bool is_open() const noexcept = 0;

    /// 发一个**请求 PDU**（从功能码起，不含地址/MBAP/CRC）。
    ///
    /// 封装由各实现自己做：
    ///   * RTU：`地址 + PDU + CRC16`；
    ///   * TCP：`MBAP(tid/proto/len/unit) + PDU`。
    ///
    /// @param[in] pdu  请求 PDU
    /// @param[in] unit 单元号 / 从站地址
    ///
    /// @note 以前这里收的是"已经封好的帧"，于是 TCP 侧的 unit 只能从
    ///       transport 自己的成员取 —— 那个成员从未被赋值，恒为 1
    ///       （`cfg.unit != 1` 时发出去的是 unit=1、校验却按 cfg.unit，
    ///       必然失败）。改成按调用传入，从根上消掉这个状态。
    virtual std::error_code send(span<const std::uint8_t> pdu, std::uint8_t unit) = 0;

    /// 收一帧完整的响应。
    ///
    /// @param[out] out     收到的完整 ADU（含 CRC 或 MBAP）
    /// @param[in]  timeout 整体超时
    /// @param[in]  unit    期望的单元号（RTU 用来过滤别的从站的应答）
    ///
    /// @note 实现内部会处理"数据分几次到"的情况。
    virtual std::error_code receive(std::vector<std::uint8_t>& out,
                                    std::chrono::milliseconds timeout,
                                    std::uint8_t unit) = 0;

    /// 丢掉接收缓冲里的残留（上次超时的半帧）。
    virtual void flush_input() = 0;

    virtual void close() noexcept = 0;

    /// 是否 RTU（决定 ADU 的组装方式）。
    [[nodiscard]] virtual bool is_rtu() const noexcept = 0;
};

} // namespace detail
} // namespace modbusmech
