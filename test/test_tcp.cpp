// ============================================================================
//  test_tcp.cpp —— Modbus TCP 端到端测试
//
//  真起 TCP 监听：Slave 在回环端口上等，Master 连上去读写。
//  TCP 比 RTU 简单（有长度字段、有事务 ID），重点验：
//    * 基本读写
//    * 事务 ID 匹配（并发时尤其重要）
//    * 多个客户端同时读（共享同一个 RegisterBank）
// ============================================================================
#include <modbusmech/master.hpp>
#include <modbusmech/slave.hpp>

#include <netmech/net.hpp>

#include <cstring>
#include <thread>
#include <vector>
#include <atomic>
#include <memory>
#include <cstddef>
#include <cstdint>
#include <string>
#include <system_error>

#include "test_framework.hpp"

using namespace modbusmech;
using namespace netmech_test;

namespace {

using namespace std::chrono_literals;

/// 100 个保持寄存器，值 = i * 10。
class Bank final : public RegisterBank {
public:
    Bank() : m_regs(100, 0)
    {
        for (std::size_t i = 0; i < m_regs.size(); ++i) {
            m_regs[i] = static_cast<std::uint16_t>(i * 10);
        }
    }

    bool read_coils(std::uint16_t, std::uint16_t, std::vector<bool>&) override
    {
        return false;
    }
    bool read_discrete_inputs(std::uint16_t, std::uint16_t,
                              std::vector<bool>&) override { return false; }
    bool read_holding_registers(std::uint16_t a, std::uint16_t n,
                                std::vector<std::uint16_t>& out) override
    {
        if (a + n > m_regs.size()) { return false; }
        out.assign(m_regs.begin() + a, m_regs.begin() + a + n);
        return true;
    }
    bool read_input_registers(std::uint16_t, std::uint16_t,
                              std::vector<std::uint16_t>&) override { return false; }
    bool write_single_coil(std::uint16_t, bool) override { return false; }
    bool write_single_register(std::uint16_t a, std::uint16_t v) override
    {
        if (a >= m_regs.size()) { return false; }
        m_regs[a] = v;
        return true;
    }
    bool write_multiple_coils(std::uint16_t, const std::vector<bool>&) override
    {
        return false;
    }
    bool write_multiple_registers(std::uint16_t a,
                                  const std::vector<std::uint16_t>& v) override
    {
        if (a + v.size() > m_regs.size()) { return false; }
        for (std::size_t i = 0; i < v.size(); ++i) { m_regs[a + i] = v[i]; }
        return true;
    }

private:
    std::vector<std::uint16_t> m_regs;
};

} // namespace

// ---------------------------------------------------------------------------
// 1. 基本读写
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_master_read_holding_registers)
{
    const auto port_opt = netmech::find_free_port_run(40000, 50000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank = std::make_shared<Bank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = Slave::create_tcp(port, sc, bank, ec);
    REQUIRE(slave != nullptr);
    CHECK(slave->is_running());

    Master::Config mc;
    mc.unit    = 1;
    mc.timeout = 1000ms;
    mc.retries = 0;

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);
    CHECK(master->is_connected());

    std::vector<std::uint16_t> regs;
    const Result r = master->read_holding_registers(5, 3, regs);
    CHECK_MSG(r.ok(), r.describe().c_str());
    REQUIRE(regs.size() == 3u);
    CHECK_EQ(regs[0], 50);    // 5 * 10
    CHECK_EQ(regs[1], 60);
    CHECK_EQ(regs[2], 70);

    master->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 2. 事务 ID 递增（连续请求都能匹配）
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_transaction_ids_increment)
{
    const auto port_opt = netmech::find_free_port_run(40000, 50000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank = std::make_shared<Bank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = Slave::create_tcp(port, sc, bank, ec);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.unit    = 1;
    mc.timeout = 1000ms;
    mc.retries = 0;

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    // 连续 5 次，每次事务 ID 都不同，都要能匹配上
    for (int i = 0; i < 5; ++i) {
        std::vector<std::uint16_t> regs;
        const Result r = master->read_holding_registers(0, 1, regs);
        CHECK_MSG(r.ok(), ("第 " + std::to_string(i) + " 次失败: " + r.describe()).c_str());
    }

    master->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 3. 写单个寄存器
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_write_single_register)
{
    const auto port_opt = netmech::find_free_port_run(40000, 50000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank = std::make_shared<Bank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = Slave::create_tcp(port, sc, bank, ec);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.unit    = 1;
    mc.timeout = 1000ms;
    mc.retries = 0;

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    const Result wr = master->write_single_register(3, 0xABCD);
    CHECK_MSG(wr.ok(), wr.describe().c_str());

    std::vector<std::uint16_t> regs;
    const Result rd = master->read_holding_registers(3, 1, regs);
    REQUIRE(rd.ok());
    REQUIRE(regs.size() == 1u);
    CHECK_EQ(regs[0], 0xABCD);

    master->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 4. 地址越界 → 异常码 02
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_exception_illegal_address)
{
    const auto port_opt = netmech::find_free_port_run(40000, 50000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank = std::make_shared<Bank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = Slave::create_tcp(port, sc, bank, ec);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.unit    = 1;
    mc.timeout = 1000ms;
    mc.retries = 0;

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    std::vector<std::uint16_t> regs;
    const Result r = master->read_holding_registers(200, 1, regs);   // 越界

    CHECK(!r.ok());
    CHECK(r.ec.value() == 0);
    CHECK(r.exception == Exception::illegal_data_address);

    master->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 5. 多个客户端同时读（共享同一个 bank）
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_multiple_clients)
{
    const auto port_opt = netmech::find_free_port_run(40000, 50000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank = std::make_shared<Bank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = Slave::create_tcp(port, sc, bank, ec);
    REQUIRE(slave != nullptr);

    constexpr int kClients = 3;
    constexpr int kReads   = 3;

    std::atomic<int> ok_count{0};
    std::vector<std::thread> threads;

    for (int c = 0; c < kClients; ++c) {
        threads.emplace_back([&, c] {
            Master::Config mc;
            mc.unit    = 1;
            mc.timeout = 2000ms;
            mc.retries = 1;

            std::error_code e;
            auto m = Master::create_tcp("127.0.0.1", port, mc, e);
            if (!m) {
                return;
            }
            for (int i = 0; i < kReads; ++i) {
                std::vector<std::uint16_t> regs;
                const Result r = m->read_holding_registers(
                    static_cast<std::uint16_t>(c), 1, regs);
                if (r.ok() && regs.size() == 1
                    && regs[0] == static_cast<std::uint16_t>(c * 10)) {
                    ++ok_count;
                }
            }
            m->close();
        });
    }
    for (auto& t : threads) {
        t.join();
    }

    CHECK_EQ(ok_count.load(), kClients * kReads);

    slave->stop();
}

// ---------------------------------------------------------------------------
// 鉴权：authorize 返回 false ⇒ 连接被立刻关掉，且计入 rejected_clients
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_slave_authorize_rejects_unknown_peer)
{
    const auto port_opt = netmech::find_free_port_run(40000, 50000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank = std::make_shared<Bank>();

    std::atomic<int>         calls{0};
    std::atomic<bool>        saw_loopback{false};
    Slave::Config            sc;
    sc.authorize = [&](const std::string& ip, std::uint16_t) {
        calls.fetch_add(1);
        if (ip == "127.0.0.1") {
            saw_loopback.store(true);
        }
        return false;   // 一律拒绝
    };

    std::error_code ec;
    auto slave = Slave::create_tcp(port, sc, bank, ec);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.unit    = 1;
    mc.timeout = 300ms;
    mc.retries = 0;

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    // TCP 握手本身会成功（库是在 accept 之后才拒的），所以这里不强制 nullptr

    // 等到库记录了一次拒绝
    for (int i = 0; i < 200 && slave->stats().rejected_clients == 0; ++i) {
        std::this_thread::sleep_for(10ms);
    }
    CHECK_EQ(slave->stats().rejected_clients, 1u);
    CHECK_EQ(calls.load(), 1);      // 每条连接只问一次
    CHECK(saw_loopback.load());     // 回调拿到的是对端 IP 字面量

    // 被拒之后请求必然失败（连接已被关）
    if (master) {
        std::vector<std::uint16_t> regs;
        const Result r = master->read_holding_registers(0, 1, regs);
        CHECK(!r.ok());
        master->close();
    }

    slave->stop();
}

// ---------------------------------------------------------------------------
// 鉴权：authorize 返回 true ⇒ 正常服务（顺带验证回调拿到的 IP 正确）
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_slave_authorize_allows_known_peer)
{
    const auto port_opt = netmech::find_free_port_run(40000, 50000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank = std::make_shared<Bank>();

    std::atomic<int>  calls{0};
    std::atomic<bool> saw_loopback{false};
    Slave::Config     sc;
    sc.authorize = [&](const std::string& ip, std::uint16_t) {
        calls.fetch_add(1);
        saw_loopback.store(ip == "127.0.0.1");
        return true;
    };

    std::error_code ec;
    auto slave = Slave::create_tcp(port, sc, bank, ec);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.unit    = 1;
    mc.timeout = 1000ms;
    mc.retries = 0;

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    std::vector<std::uint16_t> regs;
    const Result r = master->read_holding_registers(5, 2, regs);
    CHECK_MSG(r.ok(), r.describe().c_str());
    REQUIRE(regs.size() == 2u);
    CHECK_EQ(regs[0], 50);

    CHECK_EQ(calls.load(), 1);
    CHECK(saw_loopback.load());
    CHECK_EQ(slave->stats().rejected_clients, 0u);

    master->close();
    slave->stop();
}
