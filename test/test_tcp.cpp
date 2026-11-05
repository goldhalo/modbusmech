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
    /// @param[in] factor 第 i 个寄存器的值 = i * factor
    ///                   （多 unit 的用例靠它区分"这是哪台设备的数据"）
    explicit Bank(int factor = 10) : m_regs(100, 0)
    {
        for (std::size_t i = 0; i < m_regs.size(); ++i) {
            m_regs[i] = static_cast<std::uint16_t>(i * factor);
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

// ---------------------------------------------------------------------------
// 配置的 unit 必须真的进到 MBAP 里
//
// 回归用例：以前 TcpTransport 里有个 `m_unit = 1` 的成员，**从来没有被赋值**，
// 组 MBAP 时却用它 → 无论 Config::unit 设成几，报文里的 unit 恒为 1；
// 而 receive() 又拿 cfg.unit 校验 → unit != 1 时永远失败。
// 之前所有用例都用默认 unit=1，所以一直没抓到。
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_master_uses_configured_unit)
{
    const auto port_opt = netmech::find_free_port_run(40000, 50000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank = std::make_shared<Bank>();

    // 只应答 unit=7：用 bank_for 把"不是 7 的 unit"显式判为不存在。
    //
    //   注意：TCP 从站默认**应答任意 unit**（一个 IP:port 后面可能挂多台设备，
    //   靠 unit 区分），所以这里必须用 bank_for 才能构造出"unit 不对就不理"的场景。
    Slave::Config sc;
    sc.bank_for = [bank](std::uint8_t unit) -> std::shared_ptr<RegisterBank> {
        return unit == 7 ? bank : nullptr;
    };
    std::error_code ec;
    auto slave = Slave::create_tcp(port, sc, bank, ec);
    REQUIRE(slave != nullptr);

    // 主站 unit = 7：能通
    Master::Config mc;
    mc.unit    = 7;
    mc.timeout = 1000ms;
    mc.retries = 0;

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    std::vector<std::uint16_t> regs;
    const Result r = master->read_holding_registers(5, 2, regs);
    CHECK_MSG(r.ok(), r.describe().c_str());
    REQUIRE(regs.size() == 2u);
    CHECK_EQ(regs[0], 50);

    // 改成别的 unit：从站不会应答（超时）。
    // 这一条才真正证明"报文里的 unit 用的是我们配的值" ——
    // 以前 transport 里那个恒为 1 的 m_unit 会让两种情况都失败/都成功。
    master->set_unit(9);
    regs.clear();
    const Result r2 = master->read_holding_registers(5, 2, regs);
    CHECK(!r2.ok());

    master->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 一个 Master / 一条连接，服务多个 unit（一条 485 总线挂多台设备的模型）
//
// 这是"引入项目"时最关键的用法：现场一条总线挂 50 块表，不能给每块表都开一条
// 串口 —— 只能一个 Master，按 unit 轮着问。
// 带 unit 的重载把 unit 变成参数（不碰 set_unit 的共享状态），
// 所以下面这个用例里两次调用拿到的是各自设备的寄存器。
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_master_serves_multiple_units_on_one_connection)
{
    const auto port_opt = netmech::find_free_port_run(40000, 50000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    // 两台"设备"：unit=1 的值是 10x，unit=2 的值是 20x
    auto bank1 = std::make_shared<Bank>(10);   // 值 = i * 10
    auto bank2 = std::make_shared<Bank>(20);   // 值 = i * 20

    Slave::Config sc;
    sc.bank_for = [bank1, bank2](std::uint8_t unit) -> std::shared_ptr<RegisterBank> {
        if (unit == 1) {
            return bank1;
        }
        if (unit == 2) {
            return bank2;
        }
        return nullptr;
    };

    std::error_code ec;
    auto slave = Slave::create_tcp(port, sc, bank1, ec);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.unit    = 1;          // 默认 unit，带 unit 的重载不使用它
    mc.timeout = 1000ms;
    mc.retries = 0;

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    std::vector<std::uint16_t> regs;

    // unit=1
    auto r1 = master->read_holding_registers(std::uint8_t{1}, 5, 2, regs);
    CHECK_MSG(r1.ok(), r1.describe().c_str());
    REQUIRE(regs.size() == 2u);
    CHECK_EQ(regs[0], 50);

    // unit=2（同一条连接）
    auto r2 = master->read_holding_registers(std::uint8_t{2}, 5, 2, regs);
    CHECK_MSG(r2.ok(), r2.describe().c_str());
    REQUIRE(regs.size() == 2u);
    CHECK_EQ(regs[0], 100);

    // unit=3 不存在：从站静默丢弃 → 超时
    auto r3 = master->read_holding_registers(std::uint8_t{3}, 5, 2, regs);
    CHECK(!r3.ok());

    master->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 多线程并发轮询同一总线：内部串行化，不该串包
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_master_concurrent_units_are_serialized)
{
    const auto port_opt = netmech::find_free_port_run(40000, 50000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank1 = std::make_shared<Bank>(10);   // 值 = i * 10
    auto bank2 = std::make_shared<Bank>(20);   // 值 = i * 20

    Slave::Config sc;
    sc.bank_for = [bank1, bank2](std::uint8_t unit) -> std::shared_ptr<RegisterBank> {
        return unit == 1 ? bank1 : (unit == 2 ? bank2 : nullptr);
    };

    std::error_code ec;
    auto slave = Slave::create_tcp(port, sc, bank1, ec);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.unit    = 1;
    mc.timeout = 1000ms;
    mc.retries = 0;

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    std::atomic<int> ok1{0};
    std::atomic<int> ok2{0};

    auto poll = [&](std::uint8_t unit, std::atomic<int>& ok) {
        for (int i = 0; i < 20; ++i) {
            std::vector<std::uint16_t> regs;
            const Result r = master->read_holding_registers(unit, 5, 2, regs);
            if (r.ok() && regs.size() == 2u
                && regs[0] == static_cast<std::uint16_t>(unit * 50)) {
                ok.fetch_add(1);
            }
        }
    };

    std::thread t1([&] { poll(1, ok1); });
    std::thread t2([&] { poll(2, ok2); });
    t1.join();
    t2.join();

    CHECK_EQ(ok1.load(), 20);
    CHECK_EQ(ok2.load(), 20);

    master->close();
    slave->stop();
}
