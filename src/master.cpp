// =============================================================================
//  master.cpp —— 主站实现
// =============================================================================
//  分层：Master（业务）→ Transport（RTU / TCP）→ netmech
//
//  每次请求的流程都一样，只是 ADU 的封装不同：
//      RTU: unit | pdu | crc16(unit|pdu)
//      TCP: mbap(tid,0,len,unit) | pdu
// =============================================================================
#include <modbusmech/master.hpp>

#include <netmech/net.hpp>

#include "rtu.hpp"
#include "tcp.hpp"
#include "transport.hpp"

#include <chrono>
#include <cstring>
#include <thread>
#include <memory>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstdint>
#include <string>
#include <system_error>
#include <utility>

namespace modbusmech {

namespace {

} // namespace

// =============================================================================
//  Impl
// =============================================================================

class Master::Impl {
public:
    std::shared_ptr<detail::Transport> transport;
    Config                             cfg;

    /// 内部计数器：`stats()` 是给别的线程看的，而计数器在工作线程上变 ——
    /// 必须原子；对外仍然返回可拷贝的 `Master::Stats`（快照）。
    struct StatsAtomic {
        std::atomic<std::uint64_t> requests{0};
        std::atomic<std::uint64_t> succeeded{0};
        std::atomic<std::uint64_t> timeouts{0};
        std::atomic<std::uint64_t> send_errors{0};
        std::atomic<std::uint64_t> exceptions{0};
        std::atomic<std::uint64_t> crc_errors{0};
    };
    StatsAtomic                        stats;

    /// 发一个 PDU，收一个响应 PDU。
    ///
    /// @param[in]  pdu     请求 PDU
    /// @param[out] resp    响应 PDU（已剥掉 ADU 封装，含功能码）
    /// @param[out] ec      传输层错误
    void transaction(std::uint8_t unit, span<const std::uint8_t> pdu,
                     std::vector<std::uint8_t>& resp, std::error_code& ec)
    {
        resp.clear();
        ec.clear();

        // ★ 这里**不加锁**：并发由调用方负责（见 master.hpp 的"并发"一节）。
        //   本库替调用方加锁会挡住一些合法用法（比如应用自己按优先级排队、
        //   或者在允许流水线的链路上自己控制节奏），而且"库替你串行化"这件事
        //   一旦写进实现，调用方就很难察觉自己踩了并发。

        if (!transport) {
            ec = make_error_code(ModbusErrc::not_connected);
            return;
        }
        // ★ 这里**不再**预检查 transport->is_open()：TCP 短连接设备回完一个
        //   响应就断开，send() 内部会按需自动重连（ensure_connected）。在这里
        //   提前 return 会把重连彻底挡住 —— 表现是"只能问到第一个响应"。

        for (int attempt = 0; attempt <= cfg.retries; ++attempt) {
            stats.requests.fetch_add(1, std::memory_order_relaxed);

            // 1. 清掉上次超时留下的半帧
            transport->flush_input();

            // 2. 发送（封装交给 transport：RTU 自己加地址+CRC，TCP 自己加 MBAP）
            //
            //    ★ unit 由这里**按调用传入**，不再让 transport 自己存一份 ——
            //      以前 TCP 侧那个成员从未被赋值，恒为 1，cfg.unit != 1 时
            //      发出去 unit=1、校验却按 cfg.unit，必然失败。
            std::uint16_t tid = 0;   // TCP = 事务号；RTU = 0
            ec = transport->send(pdu, unit, tid);
            if (ec) {
                stats.send_errors.fetch_add(1, std::memory_order_relaxed);   // 不是超时：别混进 timeouts
                // 重试间隔：以前只有"等响应失败"才 sleep，发送失败会立刻重打，
                // 串口写不进去时会连着撞。
                if (attempt < cfg.retries && cfg.retry_delay.count() > 0) {
                    std::this_thread::sleep_for(cfg.retry_delay);
                }
                continue;
            }

            // 3. 收响应
            std::vector<std::uint8_t> adu;
            ec = transport->receive(adu, cfg.timeout, unit, tid);
            if (ec) {
                if (ec == ModbusErrc::crc_mismatch) {
                    stats.crc_errors.fetch_add(1, std::memory_order_relaxed);
                } else {
                    stats.timeouts.fetch_add(1, std::memory_order_relaxed);
                }
                // ★ 失败后重同步（传输层语义，见 transport.hpp）：
                //   RTU 没有事务号，本笔超时后从站的**迟到响应**可能在下一笔
                //   期间到达，而它和本次请求的地址/功能码/长度全都一样 —— 会被
                //   当成本次的响应，静默返回上一轮的寄存器值。resync() 会等到
                //   线路连续安静（silence_ms）再清缓冲，把这些在途字节收掉。
                //   TCP 有 TID 兜底，这一步等价于清缓冲，不引入延迟。
                transport->resync();
                if (attempt < cfg.retries) {
                    std::this_thread::sleep_for(cfg.retry_delay);
                }
                continue;
            }

            // 4. 剥掉 ADU 封装
            if (transport->is_rtu()) {
                // 地址(1) | pdu | crc(2)
                if (adu.size() < 4) {
                    ec = make_error_code(ModbusErrc::frame_too_short);
                    continue;
                }
                resp.assign(adu.begin() + 1, adu.end() - 2);
            } else {
                resp = std::move(adu);
            }
            return;
        }
    }

/// 读类请求的公共流程（01/02/03/04）。
    Result do_read(Function f, std::uint8_t unit, std::uint16_t address,
                   std::uint16_t count, std::vector<bool>* bits,
                   std::vector<std::uint16_t>* regs)
{
    Result r;

    std::vector<std::uint8_t> pdu;
    std::error_code ec;
    if (f == Function::read_coils || f == Function::read_discrete_inputs) {
        ec = encode_pdu(f, ReadBitsRequest{address, count}, pdu);
    } else {
        ec = encode_pdu(f, ReadRegistersRequest{address, count}, pdu);
    }
    if (ec) {
        r.ec = ec;
        return r;
    }

    std::vector<std::uint8_t> resp;
    transaction(unit, span<const std::uint8_t>(pdu.data(), pdu.size()), resp, ec);
    if (ec) {
        r.ec = ec;
        return r;
    }

    std::vector<bool> local_bits;
    std::vector<std::uint16_t> local_regs;
    Exception ex = Exception::none;

    ec = parse_read_response(span<const std::uint8_t>(resp.data(), resp.size()),
                             f, count, local_bits, local_regs, ex);
    if (ec) {
        r.ec = ec;
        return r;
    }
    if (ex != Exception::none) {
        stats.exceptions.fetch_add(1, std::memory_order_relaxed);
        r.exception = ex;
        return r;
    }

    if (bits) {
        *bits = std::move(local_bits);
    }
    if (regs) {
        *regs = std::move(local_regs);
    }
    stats.succeeded.fetch_add(1, std::memory_order_relaxed);
    return r;
}

/// 写类请求的公共流程（05/06/0F/10）。
    Result do_write(Function f, std::uint8_t unit, std::uint16_t address,
                    std::uint16_t count, const std::vector<std::uint8_t>& pdu)
{
    Result r;
    std::error_code ec;

    std::vector<std::uint8_t> resp;
    transaction(unit, span<const std::uint8_t>(pdu.data(), pdu.size()), resp, ec);
    if (ec) {
        r.ec = ec;
        return r;
    }

    Exception ex = Exception::none;
    ec = parse_write_response(span<const std::uint8_t>(resp.data(), resp.size()),
                              f, address, count, ex);
    if (ec) {
        r.ec = ec;
        return r;
    }
    if (ex != Exception::none) {
        stats.exceptions.fetch_add(1, std::memory_order_relaxed);
        r.exception = ex;
        return r;
    }
    stats.succeeded.fetch_add(1, std::memory_order_relaxed);
    return r;
}

};

// =============================================================================
//  Master
// =============================================================================

Master::Master() = default;
Master::~Master() = default;

std::shared_ptr<Master> Master::create_rtu(const std::string& device,
                                           const Config& cfg,
                                           std::uint32_t baud_rate,
                                           std::error_code& ec)
{
    ec.clear();

    // ★ retries < 0 会让 transaction() 的重试循环一次都不执行，
    //   于是 ec 是空的、resp 也是空的 —— 上层按"成功"去解析空响应，
    //   报出来的是 frame_too_short，与真实原因（配置错）完全对不上。
    if (cfg.retries < 0) {
        ec = make_error_code(ModbusErrc::invalid_argument);
        return nullptr;
    }

    detail::RtuTransport::Config rc;
    rc.baud_rate = baud_rate;
    // 串口参数由 Config 透传（以前写死 8N1，8E1/8O1 设备连不上）。
    rc.data_bits = cfg.data_bits;
    rc.parity    = cfg.parity;
    rc.stop_bits = cfg.stop_bits;

    auto tr = detail::RtuTransport::create(device, rc, ec);
    if (!tr) {
        return nullptr;
    }

    auto m = std::shared_ptr<Master>(new Master());
    m->m_impl = std::make_shared<Impl>();
    m->m_impl->transport = tr;
    m->m_impl->cfg = cfg;
    return m;
}

std::shared_ptr<Master> Master::create_tcp(const std::string& host,
                                           std::uint16_t port,
                                           const Config& cfg,
                                           std::error_code& ec)
{
    ec.clear();

    if (cfg.retries < 0) {
        ec = make_error_code(ModbusErrc::invalid_argument);
        return nullptr;
    }

    auto tr = detail::TcpTransport::create(host, port, ec);
    if (!tr) {
        return nullptr;
    }
    // 连接策略：是否按需重连、建连超时、同连接在途请求上限（流水线）。
    tr->set_auto_reconnect(cfg.auto_reconnect);
    tr->set_connect_timeout(cfg.connect_timeout);
    tr->set_max_in_flight(cfg.max_in_flight);

    auto m = std::shared_ptr<Master>(new Master());
    m->m_impl = std::make_shared<Impl>();
    m->m_impl->transport = tr;
    m->m_impl->cfg = cfg;
    return m;
}

bool Master::is_connected() const noexcept
{
    return m_impl && m_impl->transport && m_impl->transport->is_open();
}

std::uint8_t Master::unit() const noexcept
{
    return m_impl ? m_impl->cfg.unit : 0;
}

void Master::set_unit(std::uint8_t unit) noexcept
{
    if (m_impl) {
        m_impl->cfg.unit = unit;
    }
}

Master::Stats Master::stats() const noexcept
{
    Stats out;
    if (!m_impl) {
        return out;
    }
    // 快照：字段各自原子读，不保证属于同一瞬间（统计用途够了）。
    out.requests    = m_impl->stats.requests.load(std::memory_order_relaxed);
    out.succeeded   = m_impl->stats.succeeded.load(std::memory_order_relaxed);
    out.timeouts    = m_impl->stats.timeouts.load(std::memory_order_relaxed);
    out.send_errors = m_impl->stats.send_errors.load(std::memory_order_relaxed);
    out.exceptions  = m_impl->stats.exceptions.load(std::memory_order_relaxed);
    out.crc_errors  = m_impl->stats.crc_errors.load(std::memory_order_relaxed);
    // 重连次数在传输层（只有 TCP 会重连；RTU 恒 0）。
    out.reconnects  = m_impl->transport ? m_impl->transport->reconnects() : 0;
    return out;
}

void Master::close() noexcept
{
    if (m_impl && m_impl->transport) {
        m_impl->transport->close();
    }
}

// =============================================================================
//  读
// =============================================================================

Result Master::read_coils(std::uint16_t address, std::uint16_t count,
                          std::vector<bool>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_coils, m_impl->cfg.unit, address, count, &out, nullptr);
}

Result Master::read_discrete_inputs(std::uint16_t address, std::uint16_t count,
                                    std::vector<bool>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_discrete_inputs, m_impl->cfg.unit, address, count,
                   &out, nullptr);
}

Result Master::read_holding_registers(std::uint16_t address, std::uint16_t count,
                                      std::vector<std::uint16_t>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_holding_registers, m_impl->cfg.unit, address, count,
                   nullptr, &out);
}

Result Master::read_input_registers(std::uint16_t address, std::uint16_t count,
                                    std::vector<std::uint16_t>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_input_registers, m_impl->cfg.unit, address, count,
                   nullptr, &out);
}

// =============================================================================
//  写
// =============================================================================

Result Master::write_single_coil(std::uint16_t address, bool value)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    std::vector<std::uint8_t> pdu;
    WriteCoilRequest req;
    req.address = address;
    req.value   = value;
    const auto ec = encode_pdu(Function::write_single_coil, req, pdu);
    if (ec) {
        return Result{ec, Exception::none};
    }
    return m_impl->do_write(Function::write_single_coil, m_impl->cfg.unit, address, 1, pdu);
}

Result Master::write_single_register(std::uint16_t address, std::uint16_t value)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    std::vector<std::uint8_t> pdu;
    WriteRegisterRequest req;
    req.address = address;
    req.value   = value;
    const auto ec = encode_pdu(Function::write_single_register, req, pdu);
    if (ec) {
        return Result{ec, Exception::none};
    }
    return m_impl->do_write(Function::write_single_register, m_impl->cfg.unit, address, 1, pdu);
}

Result Master::write_multiple_coils(std::uint16_t address,
                                    const std::vector<bool>& values)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    std::vector<std::uint8_t> pdu;
    WriteCoilsRequest req;
    req.address = address;
    req.values  = values;
    const auto ec = encode_pdu(Function::write_multiple_coils, req, pdu);
    if (ec) {
        return Result{ec, Exception::none};
    }
    return m_impl->do_write(Function::write_multiple_coils, m_impl->cfg.unit, address,
                    static_cast<std::uint16_t>(values.size()), pdu);
}

Result Master::write_multiple_registers(std::uint16_t address,
                                        const std::vector<std::uint16_t>& values)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    std::vector<std::uint8_t> pdu;
    WriteRegistersRequest req;
    req.address = address;
    req.values  = values;
    const auto ec = encode_pdu(Function::write_multiple_registers, req, pdu);
    if (ec) {
        return Result{ec, Exception::none};
    }
    return m_impl->do_write(Function::write_multiple_registers, m_impl->cfg.unit, address,
                    static_cast<std::uint16_t>(values.size()), pdu);
}

// =============================================================================
//  读 / 写（显式 unit：一条总线多从站）
// =============================================================================

Result Master::read_coils(std::uint8_t unit, std::uint16_t address,
                          std::uint16_t count, std::vector<bool>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_coils, unit, address, count, &out, nullptr);
}

Result Master::read_discrete_inputs(std::uint8_t unit, std::uint16_t address,
                                    std::uint16_t count, std::vector<bool>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_discrete_inputs, unit, address, count, &out,
                           nullptr);
}

Result Master::read_holding_registers(std::uint8_t unit, std::uint16_t address,
                                      std::uint16_t count,
                                      std::vector<std::uint16_t>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_holding_registers, unit, address, count,
                           nullptr, &out);
}

Result Master::read_input_registers(std::uint8_t unit, std::uint16_t address,
                                    std::uint16_t count,
                                    std::vector<std::uint16_t>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_input_registers, unit, address, count,
                           nullptr, &out);
}

Result Master::write_single_coil(std::uint8_t unit, std::uint16_t address, bool value)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    std::vector<std::uint8_t> pdu;
    WriteCoilRequest req;
    req.address = address;
    req.value   = value;
    const auto ec = encode_pdu(Function::write_single_coil, req, pdu);
    if (ec) {
        return Result{ec, Exception::none};
    }
    return m_impl->do_write(Function::write_single_coil, unit, address, 1, pdu);
}

Result Master::write_single_register(std::uint8_t unit, std::uint16_t address,
                                     std::uint16_t value)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    std::vector<std::uint8_t> pdu;
    WriteRegisterRequest req;
    req.address = address;
    req.value   = value;
    const auto ec = encode_pdu(Function::write_single_register, req, pdu);
    if (ec) {
        return Result{ec, Exception::none};
    }
    return m_impl->do_write(Function::write_single_register, unit, address, 1, pdu);
}

Result Master::write_multiple_coils(std::uint8_t unit, std::uint16_t address,
                                    const std::vector<bool>& values)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    std::vector<std::uint8_t> pdu;
    WriteCoilsRequest req;
    req.address = address;
    req.values  = values;
    const auto ec = encode_pdu(Function::write_multiple_coils, req, pdu);
    if (ec) {
        return Result{ec, Exception::none};
    }
    return m_impl->do_write(Function::write_multiple_coils, unit, address,
                            static_cast<std::uint16_t>(values.size()), pdu);
}

Result Master::write_multiple_registers(std::uint8_t unit, std::uint16_t address,
                                        const std::vector<std::uint16_t>& values)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    std::vector<std::uint8_t> pdu;
    WriteRegistersRequest req;
    req.address = address;
    req.values  = values;
    const auto ec = encode_pdu(Function::write_multiple_registers, req, pdu);
    if (ec) {
        return Result{ec, Exception::none};
    }
    return m_impl->do_write(Function::write_multiple_registers, unit, address,
                            static_cast<std::uint16_t>(values.size()), pdu);
}

} // namespace modbusmech
