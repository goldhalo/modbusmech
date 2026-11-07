// =============================================================================
//  modbusmech - 主站
// =============================================================================
//  文件    : modbusmech/master.hpp
//  用途    : 主动发起请求的一方（采集设备）。
//
//  ★ 同步 API
//  所有 request 都是**阻塞**的：发出去、等响应、返回。串口本来就是"一问一
//  答"串行，阻塞语义最自然；TCP 想并发就开多个 Master 实例。
//
//  ★ 重试
//  `Config::retries` 是**传输失败**的重试次数（超时、串口错误）。从站返回的
//  协议异常不重试——那是它明确拒绝了（除非 `Exception::server_device_busy`）。
//
//  ★ RTU 的"迟到响应"：已知残余风险，**无法彻底消除**
//  RTU 帧里没有事务号。一旦某笔请求超时，从站**迟到的**那个响应，在协议层面
//  与下一笔请求的响应完全一致（地址、功能码、字节数都可能相同），库无法区分。
//  本库的处置是两道：
//    1) 每笔请求前清接收缓冲（丢掉"已经到达"的残帧）；
//    2) 传输失败后做一次"重同步"——等线路连续安静 `silence_ms` 再清缓冲，
//       把"还在传输途中"的迟到帧收掉（RTU transport 的 resync()）。
//  仍然消不掉的窗口：迟到帧在重同步**之后**才到达，且与下一笔请求的功能码、
//  字节数相同（典型是轮询同一从站、同一功能码、只改地址）。此时它会被当成
//  新请求的响应，**静默返回上一轮的数据**——`Result::ok()` 为真，没有任何
//  错误迹象。这是 RTU 协议的固有限制，不是本库的 bug。
//  现场对策（按优先级）：
//    ① `Config::timeout` 留足余量，别贴着设备的最大响应时间配；
//    ② 超时后丢弃该轮采集、下一轮重新开始（不要把超时当成偶发忽略）；
//    ③ 对关键点位做"写后回读"校验。
//  Modbus TCP 没有这个问题（MBAP 有事务 ID，迟到响应的 TID 对不上会被丢弃）。
// =============================================================================
#pragma once

#include "modbusmech/export.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <modbusmech/common.hpp>
#include <modbusmech/pdu.hpp>

namespace netmech {
class SerialPort;
class Conn;
}

namespace modbusmech {

/// 主站。
class MODBUSMECH_API Master {
public:
    struct Config {
        /// 单元号（RTU 的叫从站地址，TCP 的叫单元 ID）。
        std::uint8_t unit = 1;

        /// 单次请求的超时。
        ///
        /// @note 9600bps 下一个 256 字节的 RTU 帧要 267ms，所以别设太小。
        ///       默认 500ms 是工业现场常见值。
        std::chrono::milliseconds timeout{500};

        /// 传输失败的重试次数（不含首次）。
        int retries = 1;

        /// 重试间隔。
        std::chrono::milliseconds retry_delay{100};

        /// TCP 主站：连接断开后，下一次请求时是否**按需建连**（默认 false）。
        ///
        /// @par 默认关的原因
        /// 短连接设备（回完一个响应就断开）开着它也没用——对端不发包照样会断，
        /// "自动重连"只是把失败点往后挪。默认语义保持"连接断了就报错"，让
        /// 调用方立刻知道链路出问题。
        ///
        /// @par 打开之后的行为
        /// 请求发出前发现没有连接 ⇒ 先建连；**建连失败/超时直接返回失败**
        /// （不会在传输层里循环重试）。适合"对端随时会关连接、但每次都愿意
        /// 接受新连接"的场景（部分 PLC/网关）。
        bool auto_reconnect = false;

        /// TCP 建连超时（首次连接与按需重连都用它）。默认 3s。
        std::chrono::milliseconds connect_timeout{3000};

        /// TCP 主站：同一连接允许的**在途请求数上限**（MBAP 流水线）。
        ///
        /// - 默认 1：与 RTU 一样串行（一次只有一个事务在途），兼容性最好；
        /// - N > 1：允许 N 个请求同时在途，靠 MBAP 事务号匹配响应。
        ///   此时**同一个 Master 可以被多个线程并发使用**（每线程一次请求），
        ///   吞吐不再受"一问一答"限制。
        ///
        /// @warning 不是所有从站都支持流水线：很多 PLC/网关一次只处理一个请求，
        ///          收到第二个会排队或丢弃。上线前务必拿真机验证，再决定 N。
        /// @note RTU 忽略本项（物理半双工，只能一问一答）。
        std::uint32_t max_in_flight = 1;

        // ---- 串口参数（只有 create_rtu 生效；create_tcp 忽略）----
        //
        // 现在可配了：以前这三个是硬编码的 8N1，而 Modbus over Serial Line
        // V1.02 §2.3.2 的默认是 8E1 —— 8E1/8O1 的设备以前根本连不上。

        /// 数据位（5~8）。
        std::uint8_t   data_bits = 8;

        /// 校验位（默认 none = 8N1，与改造前行为一致；现场 8E1 很常见）。
        SerialParity   parity    = SerialParity::none;

        /// 停止位。
        SerialStopBits stop_bits = SerialStopBits::one;
    };

    // ---- 创建 ----

    /// RTU 主站（串口）。
    ///
    /// @param[in] device 串口设备（"COM3" / "/dev/ttyUSB0"）
    /// @param[in] cfg    配置
    /// @param[out] ec    失败原因
    [[nodiscard]] static std::shared_ptr<Master>
    create_rtu(const std::string& device, const Config& cfg,
               std::uint32_t baud_rate, std::error_code& ec);

    /// TCP 主站（客户端）。
    [[nodiscard]] static std::shared_ptr<Master>
    create_tcp(const std::string& host, std::uint16_t port, const Config& cfg,
               std::error_code& ec);

    ~Master();
    Master(const Master&) = delete;
    Master& operator=(const Master&) = delete;

    // ---- 读写 ----
    //
    // 全部**阻塞**。返回值里的 Result 区分"传输失败"和"从站异常"。
    // 读成功时 out 被填充；失败时 out 内容未定义。
    //
    // @par 两组重载：带 unit 与不带 unit
    //  不带 unit 的用 `Config::unit`（单设备场景，最常用）；
    //  带 unit 的**每次请求显式指定**（`read_coils(unit, address, count, out)`）。
    //
    //  ★ 为什么需要带 unit 的那组
    //   一条 485 总线通常挂几十台设备（现场典型：50 块电表 + 1 台综保）。
    //   如果只能靠 `set_unit()` 改状态，那么"改 unit + 发请求"这两步之间必须
    //   自己加锁 —— 任何一处忘了，两路设备就会串包。
    //   带 unit 的重载把 unit 变成**参数**：不碰 `set_unit` 那个共享状态。
    //
    // @par 并发
    //  默认（`Config::max_in_flight == 1`）与 RTU 一样：同一个 Master 的并发
    //  调用由**调用方**自己串行化，并发调用会拿到 `ModbusErrc::busy`。
    //   · RS-485 是半双工，物理上只能一问一答 —— 串行化应该发生在"谁先发"的
    //     决策上（应用才知道哪条请求更急），库替不了。现场常见做法是**一条
    //     总线一个线程 + 优先级队列**：采集低优先级、控制高优先级。
    //   · Modbus TCP 的 MBAP 里有 2 字节事务标识（Transaction Id），协议本身
    //     允许**同一连接上多个请求同时在途**。
    //
    //  ★ TCP 流水线：把 `Config::max_in_flight` 调到 > 1，同一 Master 就可以被
    //    多个线程并发使用（每线程一次请求），响应按 MBAP 事务号匹配，吞吐不再
    //    受"一问一答"限制。默认仍是 1 —— 不是所有从站都支持流水线，先拿真机
    //    验证再调大。
    //  实现：TCP 传输层维护 `tid → 等待者` 表，解析统一在 io 线程做。

    /// @name 读（使用 Config::unit）
    /// @{
    Result read_coils(std::uint16_t address, std::uint16_t count,
                      std::vector<bool>& out);

    Result read_discrete_inputs(std::uint16_t address, std::uint16_t count,
                                std::vector<bool>& out);

    Result read_holding_registers(std::uint16_t address, std::uint16_t count,
                                  std::vector<std::uint16_t>& out);

    Result read_input_registers(std::uint16_t address, std::uint16_t count,
                                std::vector<std::uint16_t>& out);

    Result write_single_coil(std::uint16_t address, bool value);

    Result write_single_register(std::uint16_t address, std::uint16_t value);

    Result write_multiple_coils(std::uint16_t address,
                                const std::vector<bool>& values);

    Result write_multiple_registers(std::uint16_t address,
                                    const std::vector<std::uint16_t>& values);
    /// @}

    /// @name 读（显式 unit：一条总线多从站时用这组）
    /// @{
    Result read_coils(std::uint8_t unit, std::uint16_t address,
                      std::uint16_t count, std::vector<bool>& out);
    Result read_discrete_inputs(std::uint8_t unit, std::uint16_t address,
                                std::uint16_t count, std::vector<bool>& out);
    Result read_holding_registers(std::uint8_t unit, std::uint16_t address,
                                  std::uint16_t count, std::vector<std::uint16_t>& out);
    Result read_input_registers(std::uint8_t unit, std::uint16_t address,
                                std::uint16_t count, std::vector<std::uint16_t>& out);
    /// @}

    /// @name 写（显式 unit）
    /// @{
    Result write_single_coil(std::uint8_t unit, std::uint16_t address, bool value);
    Result write_single_register(std::uint8_t unit, std::uint16_t address,
                                 std::uint16_t value);
    Result write_multiple_coils(std::uint8_t unit, std::uint16_t address,
                                const std::vector<bool>& values);
    Result write_multiple_registers(std::uint8_t unit, std::uint16_t address,
                                    const std::vector<std::uint16_t>& values);
    /// @}

    // ---- 状态 ----

    [[nodiscard]] bool           is_connected() const noexcept;
    [[nodiscard]] std::uint8_t   unit() const noexcept;
    void                         set_unit(std::uint8_t unit) noexcept;

    /// 已发出的请求数 / 成功数 / 超时数 / 异常数（诊断用）。
    struct Stats {
        std::uint64_t requests = 0;
        std::uint64_t succeeded = 0;
        std::uint64_t timeouts  = 0;   ///< 等响应超时
        std::uint64_t send_errors = 0; ///< 本地发送失败（连接断了 / 写不进去），不是超时
        std::uint64_t exceptions = 0;
        std::uint64_t crc_errors = 0;
        std::uint64_t reconnects = 0;  ///< TCP 自动重连次数（短连接设备的"心跳"）
    };
    [[nodiscard]] Stats stats() const noexcept;

    void close() noexcept;

private:
    Master();
    class Impl;
    std::shared_ptr<Impl> m_impl;
};

} // namespace modbusmech
