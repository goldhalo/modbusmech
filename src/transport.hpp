// =============================================================================
//  transport.hpp —— 传输层抽象（内部头）
// =============================================================================
//  Master 只依赖这个接口，不关心底下是串口还是 TCP。
//
//  两者的差别都在"怎么划出一帧"上：
//    RTU：靠"先读 2 字节，按功能码推长度，再读剩余" + CRC
//    TCP：靠 MBAP 的长度字段
//  所以 receive() 由具体实现自己决定读多少。
//
//  ★ 事务句柄（tid）
//  TCP 的 MBAP 里有 2 字节事务号，协议允许**同一连接上多个请求同时在途**。
//  为了让上层能做流水线，send() 通过 @p tid 返回本次请求的句柄，receive()
//  再带上它把响应取回来。RTU 没有事务号，tid 恒为 0（send 时写入、receive 忽略）。
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
    /// @param[in]  pdu  请求 PDU
    /// @param[in]  unit 单元号 / 从站地址
    /// @param[out] tid  本次请求的事务句柄（TCP = MBAP 事务号；RTU = 0），
    ///                  receive() 要用它取回对应的响应。
    ///
    /// @note 以前这里收的是"已经封好的帧"，于是 TCP 侧的 unit 只能从
    ///       transport 自己的成员取 —— 那个成员从未被赋值，恒为 1
    ///       （`cfg.unit != 1` 时发出去的是 unit=1、校验却按 cfg.unit，
    ///       必然失败）。改成按调用传入，从根上消掉这个状态。
    virtual std::error_code send(span<const std::uint8_t> pdu, std::uint8_t unit,
                                 std::uint16_t& tid) = 0;

    /// 收一帧完整的响应。
    ///
    /// @param[out] out     收到的完整 ADU（含 CRC 或 MBAP）
    /// @param[in]  timeout 整体超时
    /// @param[in]  unit    期望的单元号（RTU 用来过滤别的从站的应答）
    /// @param[in]  tid     send() 返回的事务句柄
    ///
    /// @note 实现内部会处理"数据分几次到"的情况。
    /// @note TCP 实现支持**多个 tid 同时在途**（流水线）：不同线程各自
    ///       send 得到自己的 tid，再各自 receive 等自己的响应。
    virtual std::error_code receive(std::vector<std::uint8_t>& out,
                                    std::chrono::milliseconds timeout,
                                    std::uint8_t unit, std::uint16_t tid) = 0;

    /// 丢掉接收缓冲里的残留（上次超时的半帧）。
    virtual void flush_input() = 0;

    /// 传输失败后的"重同步"：把线路上**可能还在途**的残留丢掉。
    ///
    /// 与 flush_input() 的区别在"在途"两个字：flush_input 只清已经到达缓冲的
    /// 字节；本条要求实现把"还没到、但已经在路上"的迟到帧也处理掉。
    ///   * TCP：有事务号（TID）兜底，迟到帧不会被误认，等价于 flush_input；
    ///   * RTU：没有事务号，迟到响应会被下一次 receive 当成自己的响应（静默错
    ///     数据）。所以 RTU 实现要"等到线路安静一段再清"，把还在传输中的帧收完。
    /// @note 只在**传输失败之后**调用，正常路径不能付这个延迟。
    virtual void resync() = 0;

    virtual void close() noexcept = 0;

    /// 是否 RTU（决定 ADU 的组装方式）。
    [[nodiscard]] virtual bool is_rtu() const noexcept = 0;

    /// 已发生的重连次数（只有能重连的传输层才有意义，默认 0）。
    [[nodiscard]] virtual std::uint64_t reconnects() const noexcept { return 0; }

    /// 连接超时（TCP 建连用；RTU 忽略）。
    virtual void set_connect_timeout(std::chrono::milliseconds) noexcept {}

    /// 同一连接允许的在途请求上限（TCP 流水线；RTU 恒为 1）。
    virtual void set_max_in_flight(std::uint32_t) noexcept {}
};

} // namespace detail
} // namespace modbusmech
