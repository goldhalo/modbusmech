// =============================================================================
//  modbusmech - 异步主站（TCP）
// =============================================================================
//  文件    : modbusmech/async_master.hpp
//  用途    : 不阻塞调用线程的主站。请求 + 回调，**库负责超时**。
//
//  ★ 为什么需要它
//  `Master` 的 API 是阻塞的：一个线程同时只能等一个响应。做 TCP 采集时，
//  要么开 N 个 Master（N 条连接 + N 条线程），要么被"一问一答"卡住。
//  `AsyncMaster` 把等待挪进库里：调用方提交请求就返回，结果在回调里拿。
//
//  ★ 超时口径（**和阻塞版一致**）
//  每个请求都按 `Config::timeout` 计时：到点没有响应，回调会以
//  `Result{ec = ModbusErrc::timeout}` 触发。**回调恰好触发一次**，调用方
//  不需要自己再做超时兜底。
//
//  ★ 回调线程
//  所有回调都在 **AsyncMaster 内部的同一条派发线程**上串行执行 —— 调用方
//  不需要给回调加锁，也不要在回调里做阻塞操作（会拖慢后续回调）。
//
//  ★ 实现方式（重要，决定能力边界）
//  内部是 `Config::max_workers` 条工作线程 + 1 条派发线程，工作线程驱动
//  **阻塞版 Master**；同一连接上的多个请求靠 MBAP 事务号在传输层做流水线
//  （`max_in_flight`）。所以：
//    · 并发度上限 = `max_workers`；
//    · 不是"零线程"实现：需要零线程的事件驱动 API 属于另一套设计（评估见
//      docs/design/tcp-nonblocking-eval.md）。
//
//  ★ 只覆盖常用功能码
//  01/02/03/04/05/06/0F/10 全都有 —— 编码/解析复用 pdu 层，成本很低。
// =============================================================================
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <modbusmech/common.hpp>

namespace modbusmech {

/// 异步主站（TCP 客户端）。
class AsyncMaster {
public:
    struct Config {
        /// 默认 unit（带 unit 参数的重载不受它影响）。
        std::uint8_t unit = 1;

        /// 单次请求超时：到点没有响应，回调以 `timeout` 触发。
        std::chrono::milliseconds timeout{500};

        /// 工作线程数 = 同一时刻允许在途的请求数（也是传输层的 max_in_flight）。
        /// 默认 4。注意：不是所有从站都支持流水线，先拿真机验证再调大。
        std::uint32_t max_workers = 4;

        /// 连接断开后，请求时是否按需建连（建连失败/超时直接失败）。
        /// 默认 false，与阻塞版一致。
        bool auto_reconnect = false;

        /// 建连超时。
        std::chrono::milliseconds connect_timeout{3000};

        /// 串口/连接断开重试次数（与阻塞版 `Master::Config::retries` 同义）。
        int retries = 0;
        std::chrono::milliseconds retry_delay{100};
    };

    using ReadBitsFn = std::function<void(Result, std::vector<bool>)>;
    using ReadRegsFn = std::function<void(Result, std::vector<std::uint16_t>)>;
    using WriteFn    = std::function<void(Result)>;

    /// 创建 TCP 异步主站。
    [[nodiscard]] static std::shared_ptr<AsyncMaster>
    create_tcp(const std::string& host, std::uint16_t port, const Config& cfg,
               std::error_code& ec);

    ~AsyncMaster();
    AsyncMaster(const AsyncMaster&) = delete;
    AsyncMaster& operator=(const AsyncMaster&) = delete;

    // ---- 读 ----

    /// 读线圈（01）。回调在派发线程执行，恰好一次。
    void read_coils(std::uint8_t unit, std::uint16_t address, std::uint16_t count,
                    ReadBitsFn cb);

    /// 读离散输入（02）。
    void read_discrete_inputs(std::uint8_t unit, std::uint16_t address,
                              std::uint16_t count, ReadBitsFn cb);

    /// 读保持寄存器（03）。
    void read_holding_registers(std::uint8_t unit, std::uint16_t address,
                                std::uint16_t count, ReadRegsFn cb);

    /// 读输入寄存器（04）。
    void read_input_registers(std::uint8_t unit, std::uint16_t address,
                              std::uint16_t count, ReadRegsFn cb);

    // ---- 写 ----

    void write_single_coil(std::uint8_t unit, std::uint16_t address, bool value,
                           WriteFn cb);
    void write_single_register(std::uint8_t unit, std::uint16_t address,
                               std::uint16_t value, WriteFn cb);
    void write_multiple_coils(std::uint8_t unit, std::uint16_t address,
                              const std::vector<bool>& values, WriteFn cb);
    void write_multiple_registers(std::uint8_t unit, std::uint16_t address,
                                  const std::vector<std::uint16_t>& values, WriteFn cb);

    // ---- 状态 ----

    /// 已提交但还没回调完成的请求数。
    [[nodiscard]] std::size_t pending_count() const noexcept;

    /// 是否还能接收新请求（close() 之后为 false）。
    [[nodiscard]] bool is_running() const noexcept;

    /// 关闭：不再接收新请求；在途/排队的请求会以 `not_connected` 结束
    /// （回调仍会触发，保证"恰好一次"），然后等待线程退出。
    void close() noexcept;

private:
    AsyncMaster();
    class Impl;
    std::shared_ptr<Impl> m_impl;
};

} // namespace modbusmech
