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
    mutable Stats                      stats;

    /// 发一个 PDU，收一个响应 PDU。
    ///
    /// @param[in]  pdu     请求 PDU
    /// @param[out] resp    响应 PDU（已剥掉 ADU 封装，含功能码）
    /// @param[out] ec      传输层错误
    void transaction(span<const std::uint8_t> pdu, std::vector<std::uint8_t>& resp,
                     std::error_code& ec)
    {
        resp.clear();
        ec.clear();

        if (!transport || !transport->is_open()) {
            ec = make_error_code(ModbusErrc::not_connected);
            return;
        }

        for (int attempt = 0; attempt <= cfg.retries; ++attempt) {
            ++stats.requests;

            // 1. 清掉上次超时留下的半帧
            transport->flush_input();

            // 2. 发送（封装交给 transport：RTU 自己加地址+CRC，TCP 自己加 MBAP）
            //
            //    ★ unit 由这里**按调用传入**，不再让 transport 自己存一份 ——
            //      以前 TCP 侧那个成员从未被赋值，恒为 1，cfg.unit != 1 时
            //      发出去 unit=1、校验却按 cfg.unit，必然失败。
            ec = transport->send(pdu, cfg.unit);
            if (ec) {
                ++stats.send_errors;   // 不是超时：别混进 timeouts
                // 重试间隔：以前只有"等响应失败"才 sleep，发送失败会立刻重打，
                // 串口写不进去时会连着撞。
                if (attempt < cfg.retries && cfg.retry_delay.count() > 0) {
                    std::this_thread::sleep_for(cfg.retry_delay);
                }
                continue;
            }

            // 3. 收响应
            std::vector<std::uint8_t> adu;
            ec = transport->receive(adu, cfg.timeout, cfg.unit);
            if (ec) {
                if (ec == ModbusErrc::crc_mismatch) {
                    ++stats.crc_errors;
                } else {
                    ++stats.timeouts;
                }
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
    Result do_read(Function f, std::uint16_t address,
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
    transaction(span<const std::uint8_t>(pdu.data(), pdu.size()), resp, ec);
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
        ++stats.exceptions;
        r.exception = ex;
        return r;
    }

    if (bits) {
        *bits = std::move(local_bits);
    }
    if (regs) {
        *regs = std::move(local_regs);
    }
    ++stats.succeeded;
    return r;
}

/// 写类请求的公共流程（05/06/0F/10）。
    Result do_write(Function f, std::uint16_t address,
                    std::uint16_t count, const std::vector<std::uint8_t>& pdu)
{
    Result r;
    std::error_code ec;

    std::vector<std::uint8_t> resp;
    transaction(span<const std::uint8_t>(pdu.data(), pdu.size()), resp, ec);
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
        ++stats.exceptions;
        r.exception = ex;
        return r;
    }
    ++stats.succeeded;
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
    return m_impl ? m_impl->stats : Stats{};
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
    return m_impl->do_read(Function::read_coils, address, count, &out, nullptr);
}

Result Master::read_discrete_inputs(std::uint16_t address, std::uint16_t count,
                                    std::vector<bool>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_discrete_inputs, address, count,
                   &out, nullptr);
}

Result Master::read_holding_registers(std::uint16_t address, std::uint16_t count,
                                      std::vector<std::uint16_t>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_holding_registers, address, count,
                   nullptr, &out);
}

Result Master::read_input_registers(std::uint16_t address, std::uint16_t count,
                                    std::vector<std::uint16_t>& out)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    return m_impl->do_read(Function::read_input_registers, address, count,
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
    const auto ec = encode_pdu(Function::write_single_coil,
                               WriteCoilRequest{address, value}, pdu);
    if (ec) {
        return Result{ec, Exception::none};
    }
    return m_impl->do_write(Function::write_single_coil, address, 1, pdu);
}

Result Master::write_single_register(std::uint16_t address, std::uint16_t value)
{
    if (!m_impl) {
        return Result{make_error_code(ModbusErrc::not_connected), Exception::none};
    }
    std::vector<std::uint8_t> pdu;
    const auto ec = encode_pdu(Function::write_single_register,
                               WriteRegisterRequest{address, value}, pdu);
    if (ec) {
        return Result{ec, Exception::none};
    }
    return m_impl->do_write(Function::write_single_register, address, 1, pdu);
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
    return m_impl->do_write(Function::write_multiple_coils, address,
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
    return m_impl->do_write(Function::write_multiple_registers, address,
                    static_cast<std::uint16_t>(values.size()), pdu);
}

} // namespace modbusmech
