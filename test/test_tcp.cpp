// ============================================================================
//  test_tcp.cpp —— Modbus TCP 端到端测试
//
//  真起 TCP 监听：Slave 在回环端口上等，Master 连上去读写。
//  TCP 比 RTU 简单（有长度字段、有事务 ID），重点验：
//    * 基本读写
//    * 事务 ID 匹配（并发时尤其重要）
//    * 多个客户端同时读（共享同一个 RegisterBank）
// ============================================================================
#include <modbusmech/async_master.hpp>
#include <modbusmech/master.hpp>
#include <modbusmech/slave.hpp>

#include <netmech/net.hpp>

#include <cstring>
#include <cstdio>
#include <array>
#include <condition_variable>
#include <mutex>
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

/// 用 port=0 让内核分配监听端口，彻底避开"先探测空闲端口再 bind"的 TOCTOU 窗口。
/// 成功后把实际端口写回 port_out，供 Master 连接；失败时把 ec 打出来。
inline std::shared_ptr<Slave> make_test_slave(const Slave::Config& cfg,
                                              std::shared_ptr<RegisterBank> bank,
                                              std::error_code& ec,
                                              std::uint16_t& port_out)
{
    auto s = Slave::create_tcp(0, cfg, std::move(bank), ec);
    if (!s) {
        std::fputs("[test] create_tcp(0) failed: ", stderr);
        std::fputs(ec.message().c_str(), stderr);
        std::fputc(10, stderr);
        port_out = 0;
        return nullptr;
    }
    port_out = s->listening_port();
    if (port_out == 0) {
        std::fputs("[test] listening_port() == 0", stderr);
        std::fputc(10, stderr);
        ec = std::make_error_code(std::errc::address_not_available);
        return nullptr;
    }
    return s;
}

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

    bool read_coils(std::uint8_t /*unit*/, std::uint16_t, std::uint16_t, std::vector<bool>&) override
    {
        return false;
    }
    bool read_discrete_inputs(std::uint8_t /*unit*/, std::uint16_t, std::uint16_t,
                              std::vector<bool>&) override { return false; }
    bool read_holding_registers(std::uint8_t /*unit*/, std::uint16_t a, std::uint16_t n,
                                std::vector<std::uint16_t>& out) override
    {
        if (a + n > m_regs.size()) { return false; }
        out.assign(m_regs.begin() + a, m_regs.begin() + a + n);
        return true;
    }
    bool read_input_registers(std::uint8_t /*unit*/, std::uint16_t, std::uint16_t,
                              std::vector<std::uint16_t>&) override { return false; }
    bool write_single_coil(std::uint8_t /*unit*/, std::uint16_t, bool) override { return false; }
    bool write_single_register(std::uint8_t /*unit*/, std::uint16_t a, std::uint16_t v) override
    {
        if (a >= m_regs.size()) { return false; }
        m_regs[a] = v;
        return true;
    }
    bool write_multiple_coils(std::uint8_t /*unit*/, std::uint16_t, const std::vector<bool>&) override
    {
        return false;
    }
    bool write_multiple_registers(std::uint8_t /*unit*/, std::uint16_t a,
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
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
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
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
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
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
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
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
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
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
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
// 连接建立回调：on_client_connected 返回 false ⇒ 连接被立刻关掉，且计入 rejected_clients
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_slave_authorize_rejects_unknown_peer)
{
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>();

    std::atomic<int>         calls{0};
    std::atomic<int>         disconnects{0};
    std::atomic<bool>        saw_loopback{false};
    Slave::Config            sc;
    sc.on_client_connected = [&](std::uint64_t /*id*/, const std::string& ip, std::uint16_t) {
        calls.fetch_add(1);
        if (ip == "127.0.0.1") {
            saw_loopback.store(true);
        }
        return false;   // 一律拒绝
    };
    sc.on_client_disconnected = [&](std::uint64_t, const std::string&, std::uint16_t) {
        disconnects.fetch_add(1);
    };

    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
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
    CHECK_EQ(disconnects.load(), 0);  // 被拒的连接从未被接管 ⇒ 不触发 on_linkdown

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
// 连接事件回调：connected / disconnected 成对触发，id 一致，覆盖对端关闭
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_slave_client_connected_disconnected_events)
{
    std::uint16_t port = 0;
    auto bank = std::make_shared<Bank>(10);

    std::mutex mu;
    std::vector<std::uint64_t> connected, disconnected;
    std::atomic<bool> saw_loopback{false};

    Slave::Config sc;
    sc.on_client_connected = [&](std::uint64_t id, const std::string& ip, std::uint16_t) {
        if (ip == "127.0.0.1") saw_loopback.store(true);
        std::lock_guard<std::mutex> lk(mu);
        connected.push_back(id);
        return true;
    };
    sc.on_client_disconnected = [&](std::uint64_t id, const std::string&, std::uint16_t) {
        std::lock_guard<std::mutex> lk(mu);
        disconnected.push_back(id);
    };

    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.timeout = 1000ms;
    mc.retries = 0;
    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    for (int i = 0; i < 100; ++i) {
        { std::lock_guard<std::mutex> lk(mu); if (!connected.empty()) break; }
        std::this_thread::sleep_for(10ms);
    }
    { std::lock_guard<std::mutex> lk(mu); REQUIRE(connected.size() == 1u); }
    CHECK(saw_loopback.load());

    master->close();
    slave->stop();
    for (int i = 0; i < 200; ++i) {
        { std::lock_guard<std::mutex> lk(mu); if (!disconnected.empty()) break; }
        std::this_thread::sleep_for(10ms);
    }
    std::lock_guard<std::mutex> lk(mu);
    REQUIRE(disconnected.size() == 1u);
    CHECK_EQ(disconnected[0], connected[0]);
}

// ---------------------------------------------------------------------------
// 连接建立回调：on_client_connected 返回 true ⇒ 正常服务（顺带验证回调拿到的 IP 正确）
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_slave_authorize_allows_known_peer)
{
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>();

    std::atomic<int>  calls{0};
    std::atomic<bool> saw_loopback{false};
    Slave::Config     sc;
    sc.on_client_connected = [&](std::uint64_t /*id*/, const std::string& ip, std::uint16_t) {
        calls.fetch_add(1);
        saw_loopback.store(ip == "127.0.0.1");
        return true;
    };

    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
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
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

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
    auto slave = make_test_slave(sc, bank, ec, port);
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
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

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
    auto slave = make_test_slave(sc, bank1, ec, port);
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
// 多线程并发轮询：**由调用方串行化**（这是库推荐的用法）
//
// 库内部不加锁 —— 485 是半双工，串行化该发生在"谁先发"的决策上
// （EAS 就是一条总线一个线程 + 优先级队列：采集低、控制高）；
// Modbus TCP 有 MBAP 事务标识，协议上允许流水线，库加锁反而堵死这条路。
// 所以这里演示的是**应用侧**怎么写：自己拿一把锁（或者用单线程 + 优先队列）。
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_master_concurrent_polling_with_app_side_lock)
{
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank1 = std::make_shared<Bank>(10);
    auto bank2 = std::make_shared<Bank>(20);

    Slave::Config sc;
    sc.bank_for = [bank1, bank2](std::uint8_t unit) -> std::shared_ptr<RegisterBank> {
        return unit == 1 ? bank1 : (unit == 2 ? bank2 : nullptr);
    };

    std::error_code ec;
    auto slave = make_test_slave(sc, bank1, ec, port);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.unit    = 1;
    mc.timeout = 1000ms;
    mc.retries = 0;

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    // ★ 应用侧的串行化：一把锁（真实系统里是"一个线程 + 优先级队列"）
    std::mutex       bus_mtx;
    std::atomic<int> ok1{0};
    std::atomic<int> ok2{0};

    auto poll = [&](std::uint8_t unit, std::atomic<int>& ok) {
        for (int i = 0; i < 20; ++i) {
            std::vector<std::uint16_t> regs;
            Result                     r;
            {
                std::lock_guard<std::mutex> lk(bus_mtx);   // ← 串行化在这里
                r = master->read_holding_registers(unit, 5, 2, regs);
            }
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

// ===========================================================================
// 短连接 / 连接管理（2026-11 新增）
// ===========================================================================
namespace {

/// 短连接从站：收到一个请求、回一个响应、然后**主动断开**。
///
/// 现场有些 PLC 只支持这种"一问一连接"的模式，主站必须能自动重连。
class OneShotSession final : public netmech::ConnHandler {
public:
    explicit OneShotSession(std::shared_ptr<netmech::Conn> c) : m_conn(std::move(c)) {}

    bool on_open(std::shared_ptr<netmech::Conn> c) override
    {
        m_conn = c;
        return m_conn->read(netmech::Buf{nullptr, m_buf.data(), m_buf.size()});
    }

    void on_read(std::error_code ec, netmech::Buf buf, std::size_t n) override
    {
        if (ec) {
            m_conn->close();
            return;
        }
        m_rx.insert(m_rx.end(), static_cast<const std::uint8_t*>(buf.data),
                    static_cast<const std::uint8_t*>(buf.data) + n);
        for (;;) {
            if (m_rx.size() < 7) {
                break;
            }
            const std::size_t len   = (static_cast<std::size_t>(m_rx[4]) << 8) | m_rx[5];
            const std::size_t total = 6 + len;
            if (m_rx.size() < total) {
                break;
            }

            // 固定回"03 读 2 个寄存器"：tid / unit 原样回填
            std::vector<std::uint8_t> out;
            out.push_back(m_rx[0]);        // 事务号高字节
            out.push_back(m_rx[1]);        // 事务号低字节
            out.push_back(0);
            out.push_back(0);              // 协议 ID
            out.push_back(0);
            out.push_back(7);              // 长度 = unit + fc + bc + 4
            out.push_back(m_rx[6]);        // 单元 ID
            out.push_back(0x03);           // 功能码
            out.push_back(0x04);           // 字节数
            out.push_back(0x12);
            out.push_back(0x34);
            out.push_back(0x56);
            out.push_back(0x78);
            m_rx.erase(m_rx.begin(),
                       m_rx.begin() + static_cast<std::ptrdiff_t>(total));

            m_pending = std::make_shared<std::vector<std::uint8_t>>(std::move(out));
            (void)m_conn->write(netmech::ConstBuf{m_pending, m_pending->data(),
                                                  m_pending->size()});
            m_close_after_write = true;
        }
    }

    void on_write(std::error_code, netmech::ConstBuf, std::size_t) override
    {
        if (m_close_after_write) {
            m_conn->close();   // 短连接：回完就断
        }
    }

    void on_close() override {}

private:
    std::shared_ptr<netmech::Conn>             m_conn;
    std::vector<std::uint8_t>                  m_rx;
    std::shared_ptr<std::vector<std::uint8_t>> m_pending;
    std::array<std::uint8_t, 512>              m_buf{};
    bool                                       m_close_after_write = false;
};

class OneShotLink final : public netmech::Linkable {
public:
    std::shared_ptr<netmech::ConnHandler> on_accept(netmech::Conn& c) override
    {
        return std::make_shared<OneShotSession>(c.shared_from_this());
    }
};

/// 起一个"回完就断"的短连接服务端。
///
/// ⚠ 必须把 Linkable 和 Mechanism 一起返回：Mechanism 只持 weak_ptr<Linkable>，
///    局部 link 销毁后新连接会被当成"无订阅者"立刻关掉。
struct OneShotServer {
    std::shared_ptr<netmech::Mechanism> mech;
    std::shared_ptr<OneShotLink>        link;
    std::uint16_t                       port = 0;   // listen(:0) 后回读的实际端口
};

/// port=0 由内核分配端口，彻底避开"探测空闲端口 → 再 bind"的 TOCTOU 窗口。
/// 回读的实际端口放在 srv.port，供 Master 连接；失败时打印 ec。
OneShotServer start_oneshot_server()
{
    OneShotServer srv;
    srv.mech = netmech::Mechanism::create();
    if (!srv.mech) {
        return srv;
    }
    srv.link = std::make_shared<OneShotLink>();
    std::error_code ec = srv.mech->listen(netmech::Address::any(0), srv.link);
    if (ec) {
        std::fputs("[test] oneshot listen(:0) failed: ", stderr);
        std::fputs(ec.message().c_str(), stderr);
        std::fputc(10, stderr);
        srv.mech.reset();
        srv.link.reset();
        return srv;
    }
    const auto addrs = srv.mech->listening_addresses();
    if (addrs.empty()) {
        std::fputs("[test] oneshot listening_addresses() empty", stderr);
        std::fputc(10, stderr);
        srv.mech.reset();
        srv.link.reset();
        return srv;
    }
    srv.port = addrs.front().port();
    return srv;
}

} // namespace

// ---------------------------------------------------------------------------
// 17. TCP 主站自动重连：从站每回一个响应就断（短连接 PLC）
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_master_auto_reconnect_short_connection)
{
    auto server = start_oneshot_server();
    REQUIRE(server.mech != nullptr);
    REQUIRE(server.port != 0u);
    const std::uint16_t port = server.port;

    Master::Config mc;
    mc.timeout        = 1000ms;
    mc.retries        = 2;      // 连接恰好断在两次请求之间时允许重试（现场也这么配）
    mc.retry_delay    = 10ms;
    mc.auto_reconnect = true;

    std::error_code ec;
    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    int ok = 0;
    for (int i = 0; i < 3; ++i) {
        std::vector<std::uint16_t> regs;
        const auto r = master->read_holding_registers(0, 2, regs);
        if (r.ok() && regs.size() == 2u && regs[0] == 0x1234 && regs[1] == 0x5678) {
            ++ok;
        }
    }
    CHECK_EQ(ok, 3);
    // 三次请求至少重连过两次（第一次不算重连）
    CHECK(master->stats().reconnects >= 2u);

    master->close();
    server.mech->close_all();
}

// ---------------------------------------------------------------------------
// 18. auto_reconnect = false：连接断了之后本 Master 不再可用（改造前行为）
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_master_reconnect_disabled_reports_error)
{
    auto server = start_oneshot_server();
    REQUIRE(server.mech != nullptr);
    REQUIRE(server.port != 0u);
    const std::uint16_t port = server.port;

    Master::Config mc;
    mc.timeout        = 1000ms;
    mc.retries        = 0;
    mc.auto_reconnect = false;

    std::error_code ec;
    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    std::vector<std::uint16_t> regs;
    CHECK(master->read_holding_registers(0, 2, regs).ok());

    // 等对端把连接断掉
    for (int i = 0; i < 200 && master->is_connected(); ++i) {
        std::this_thread::sleep_for(5ms);
    }
    CHECK(!master->is_connected());

    std::vector<std::uint16_t> regs2;
    const auto r = master->read_holding_registers(0, 2, regs2);
    CHECK(!r.ok());
    CHECK(r.ec);   // 传输层错误（未连接）

    master->close();
    server.mech->close_all();
}

// ---------------------------------------------------------------------------
// 19. TCP 从站空闲超时：连上不发请求，超时后由服务端断开
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_slave_idle_timeout_closes_idle_connection)
{
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>(10);

    Slave::Config sc;
    sc.idle_timeout = 100ms;

    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.timeout        = 1000ms;
    mc.retries        = 0;
    mc.auto_reconnect = false;   // 本用例只关心"服务端会不会主动断"

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);
    CHECK(master->is_connected());

    bool dropped = false;
    for (int i = 0; i < 200; ++i) {
        std::this_thread::sleep_for(10ms);
        if (!master->is_connected()) {
            dropped = true;
            break;
        }
    }
    CHECK(dropped);
    // 服务端的会话销毁（clients 计数递减）可能比客户端察觉晚一点点
    for (int i = 0; i < 100 && slave->client_count() != 0; ++i) {
        std::this_thread::sleep_for(5ms);
    }
    CHECK_EQ(slave->client_count(), std::size_t{0});

    master->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 20. TCP 从站连接数上限：超出的连接在 accept 阶段被拒
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_slave_max_clients_rejects_extra)
{
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>(10);

    Slave::Config sc;
    sc.max_clients = 1;

    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.timeout        = 500ms;
    mc.retries        = 0;
    mc.auto_reconnect = false;

    auto m1 = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(m1 != nullptr);
    CHECK(m1->is_connected());

    // 第二条连接：TCP 层能连上，但 accept 阶段被拒 —— 随即被服务端关掉
    auto m2 = Master::create_tcp("127.0.0.1", port, mc, ec);
    if (m2) {
        for (int i = 0; i < 100 && m2->is_connected(); ++i) {
            std::this_thread::sleep_for(5ms);
        }
        CHECK(!m2->is_connected());
        m2->close();
    }
    CHECK_EQ(slave->stats().rejected_clients, std::uint64_t{1});

    m1->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 21. TCP 流水线：max_in_flight > 1 时多个线程可共用一个 Master
//
//     协议允许同一连接上多个事务在途（MBAP 事务号区分）。传输层用
//     `tid → 等待者` 表 + io 线程统一解析来实现，多个调用线程各等自己的 tid。
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_master_pipeline_allows_concurrent_requests)
{
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>(10);

    Slave::Config sc;
    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.timeout        = 1000ms;
    mc.retries        = 0;
    mc.auto_reconnect = false;
    mc.max_in_flight  = 4;      // ← 流水线

    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    std::atomic<int> ok{0};
    std::atomic<int> busy{0};
    auto worker = [&](std::uint16_t addr) {
        for (int i = 0; i < 10; ++i) {
            std::vector<std::uint16_t> regs;
            const auto r = master->read_holding_registers(addr, 2, regs);
            if (r.ok() && regs.size() == 2u) {
                ok.fetch_add(1);
            } else if (r.ec == ModbusErrc::busy) {
                busy.fetch_add(1);
            }
        }
    };

    std::thread t1([&] { worker(0); });
    std::thread t2([&] { worker(10); });
    std::thread t3([&] { worker(20); });
    t1.join();
    t2.join();
    t3.join();

    CHECK_EQ(ok.load(), 30);     // 30 笔全部成功
    CHECK_EQ(busy.load(), 0);    // 4 个在途名额够 3 个线程用

    master->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 24. TCP 从站的 unit 过滤：显式配置 addresses 后，其它 unit 静默丢弃
//
//     以前 TCP 路径完全不看 `Config::addresses`：配了 --units 也不生效，
//     任何 unit 都能问到同一个 bank。现在只有"显式配置了 addresses"时才过滤，
//     默认（addresses 为空）仍保持 TCP 的宽松语义（很多客户端用 0/255）。
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_slave_address_filter_drops_other_units)
{
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>(10);

    Slave::Config sc;
    sc.addresses = {1};        // 只应答 unit 1

    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
    REQUIRE(slave != nullptr);

    Master::Config mc;
    mc.timeout        = 300ms;
    mc.retries        = 0;
    mc.auto_reconnect = false;

    // unit=1：正常应答
    mc.unit = 1;
    auto m1 = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(m1 != nullptr);
    std::vector<std::uint16_t> regs;
    CHECK(m1->read_holding_registers(0, 2, regs).ok());

    // unit=2：不名单里 ⇒ 静默丢弃 ⇒ 超时
    mc.unit = 2;
    auto m2 = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(m2 != nullptr);
    std::vector<std::uint16_t> regs2;
    const auto r = m2->read_holding_registers(0, 2, regs2);
    CHECK(!r.ok());
    CHECK(r.ec == ModbusErrc::timeout);

    m1->close();
    m2->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 异步主站：只接受连接、**永不应答**的从站（用来验证超时回调）
// ---------------------------------------------------------------------------
namespace {

class SilentSession final : public netmech::ConnHandler {
public:
    explicit SilentSession(std::shared_ptr<netmech::Conn> c) : m_conn(std::move(c)) {}

    bool on_open(std::shared_ptr<netmech::Conn> c) override
    {
        m_conn = c;
        return m_conn->read(netmech::Buf{nullptr, m_buf.data(), m_buf.size()});
    }
    void on_read(std::error_code, netmech::Buf, std::size_t) override
    {
        // 故意不应答：让主站超时
        (void)m_conn->read(netmech::Buf{nullptr, m_buf.data(), m_buf.size()});
    }
    void on_write(std::error_code, netmech::ConstBuf, std::size_t) override {}
    void on_close() override {}

private:
    std::shared_ptr<netmech::Conn> m_conn;
    std::array<std::uint8_t, 512>  m_buf{};
};

class SilentLink final : public netmech::Linkable {
public:
    std::shared_ptr<netmech::ConnHandler> on_accept(netmech::Conn& c) override
    {
        return std::make_shared<SilentSession>(c.shared_from_this());
    }
};

struct SilentServer {
    std::shared_ptr<netmech::Mechanism> mech;
    std::shared_ptr<SilentLink>         link;
    std::uint16_t                       port = 0;   // listen(:0) 后回读的实际端口
};

/// port=0 由内核分配端口，彻底避开"探测空闲端口 → 再 bind"的 TOCTOU 窗口。
/// 回读的实际端口放在 srv.port，供 Master 连接；失败时打印 ec。
SilentServer start_silent_server()
{
    SilentServer srv;
    srv.mech = netmech::Mechanism::create();
    if (!srv.mech) {
        return srv;
    }
    srv.link = std::make_shared<SilentLink>();
    std::error_code ec = srv.mech->listen(netmech::Address::any(0), srv.link);
    if (ec) {
        std::fputs("[test] silent listen(:0) failed: ", stderr);
        std::fputs(ec.message().c_str(), stderr);
        std::fputc(10, stderr);
        srv.mech.reset();
        srv.link.reset();
        return srv;
    }
    const auto addrs = srv.mech->listening_addresses();
    if (addrs.empty()) {
        std::fputs("[test] silent listening_addresses() empty", stderr);
        std::fputc(10, stderr);
        srv.mech.reset();
        srv.link.reset();
        return srv;
    }
    srv.port = addrs.front().port();
    return srv;
}

/// 异步回调探针：记录结果与回调所在线程。
struct AsyncProbe {
    std::mutex                     mtx;
    std::condition_variable        cv;
    bool                           done = false;
    Result                         r;
    std::vector<std::uint16_t>     regs;
    std::thread::id                cb_thread{};

    AsyncMaster::ReadRegsFn fn()
    {
        return [this](Result res, std::vector<std::uint16_t> v) {
            std::lock_guard<std::mutex> lk(mtx);
            done      = true;
            r         = res;
            regs      = std::move(v);
            cb_thread = std::this_thread::get_id();
            cv.notify_all();
        };
    }
    bool wait(std::chrono::milliseconds t)
    {
        std::unique_lock<std::mutex> lk(mtx);
        return cv.wait_for(lk, t, [&] { return done; });
    }
};

} // namespace

// ---------------------------------------------------------------------------
// 22. 异步主站：基本读 + 回调线程串行
// ---------------------------------------------------------------------------
NETMECH_TEST(async_master_read_holding_registers)
{
    std::uint16_t port = 0;   // port=0 由内核分配，make_test_slave 回写实际端口

    auto bank = std::make_shared<Bank>(10);

    Slave::Config sc;
    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
    REQUIRE(slave != nullptr);

    AsyncMaster::Config ac;
    ac.timeout     = 1000ms;
    ac.max_workers = 3;

    auto am = AsyncMaster::create_tcp("127.0.0.1", port, ac, ec);
    REQUIRE(am != nullptr);

    AsyncProbe p1, p2;
    am->read_holding_registers(0, 0, 2, p1.fn());
    am->read_holding_registers(0, 10, 2, p2.fn());

    REQUIRE(p1.wait(2000ms));
    REQUIRE(p2.wait(2000ms));
    CHECK(p1.r.ok());
    CHECK_EQ(p1.regs.size(), 2u);
    CHECK_EQ(p1.regs[0], 0x0000);
    CHECK_EQ(p1.regs[1], 0x000A);
    CHECK(p2.r.ok());
    CHECK_EQ(p2.regs[0], 0x0064);   // 10 * 10
    // 两个回调都在同一条派发线程上
    CHECK(p1.cb_thread == p2.cb_thread);

    am->close();
    slave->stop();
}

// ---------------------------------------------------------------------------
// 23. 异步主站：对端不应答时，回调必须按 timeout 触发（用户不再自己兜超时）
// ---------------------------------------------------------------------------
NETMECH_TEST(async_master_timeout_callback_fires)
{
    auto server = start_silent_server();
    REQUIRE(server.mech != nullptr);
    REQUIRE(server.port != 0u);
    const std::uint16_t port = server.port;

    AsyncMaster::Config ac;
    ac.timeout     = 150ms;
    ac.retries     = 0;
    ac.max_workers = 1;

    std::error_code ec;
    auto am = AsyncMaster::create_tcp("127.0.0.1", port, ac, ec);
    REQUIRE(am != nullptr);

    AsyncProbe p;
    const auto t0 = std::chrono::steady_clock::now();
    am->read_holding_registers(0, 0, 2, p.fn());

    REQUIRE(p.wait(3000ms));       // 回调一定会来，不需要用户兜底
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0);

    CHECK(!p.r.ok());
    CHECK(p.r.ec == ModbusErrc::timeout);
    CHECK(elapsed >= 100ms);       // 至少等过 timeout
    CHECK(elapsed < 2500ms);       // 但不能拖太久

    am->close();
    server.mech->close_all();
}

// ---------------------------------------------------------------------------
// 25. AsyncMaster：close 之后提交请求，回调仍必须触发（评审 M1）
//
//     回归：submit_request 把 done 双重移动（task 捕获一次、reject 再
//     移动一次），close 后走 reject 时 done 已被移空 → bad_function_call
//     被派发线程吞掉 → 回调静默丢失，违反"恰好一次"承诺。
// ---------------------------------------------------------------------------
NETMECH_TEST(async_master_submit_after_close_still_calls_back)
{
    std::uint16_t port = 0;
    auto bank = std::make_shared<Bank>(10);
    Slave::Config sc;
    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
    REQUIRE(slave != nullptr);

    AsyncMaster::Config ac;
    ac.timeout     = 500ms;
    ac.max_workers = 2;
    auto am = AsyncMaster::create_tcp("127.0.0.1", port, ac, ec);
    REQUIRE(am != nullptr);

    am->close();
    CHECK(!am->is_running());

    // close 后提交：必须恰好一次回调（not_connected），不能静默丢失
    AsyncProbe p;
    am->read_holding_registers(0, 0, 2, p.fn());
    REQUIRE(p.wait(3000ms));
    CHECK(p.r.ec == ModbusErrc::not_connected);

    // 再提交一笔，同样要回调（验证不是偶然）
    AsyncProbe p2;
    am->write_single_register(0, 0, 0x1234, [&](Result r) {
        // WriteFn 探针：复用 AsyncProbe 的等待机制
        std::lock_guard<std::mutex> lk(p2.mtx);
        p2.done = true;
        p2.r = r;
        p2.cv.notify_all();
    });
    REQUIRE(p2.wait(3000ms));
    CHECK(p2.r.ec == ModbusErrc::not_connected);

    slave->stop();
}

// ---------------------------------------------------------------------------
// 26. AsyncMaster：在回调里调 close() 不得 terminate（评审 M2）
//
//     回归：close() 的 self-id 检查跳过自 join 后留下 joinable 线程，
//     ~Impl 销毁 joinable 的 std::thread → std::terminate 整进程。
//     修复后自线程 detach + 强自持，析构安全。
// ---------------------------------------------------------------------------
NETMECH_TEST(async_master_close_from_callback_is_safe)
{
    std::uint16_t port = 0;
    auto bank = std::make_shared<Bank>(10);
    Slave::Config sc;
    std::error_code ec;
    auto slave = make_test_slave(sc, bank, ec, port);
    REQUIRE(slave != nullptr);

    AsyncMaster::Config ac;
    ac.timeout     = 1000ms;
    ac.max_workers = 2;
    auto am = AsyncMaster::create_tcp("127.0.0.1", port, ac, ec);
    REQUIRE(am != nullptr);

    AsyncProbe p;
    auto inner = p.fn();
    am->read_holding_registers(0, 0, 2,
        [inner, raw = am.get()](Result r, std::vector<std::uint16_t> v) {
            raw->close();   // 在派发线程（回调）里关闭：修复前这里埋下 terminate
            inner(r, std::move(v));
        });
    REQUIRE(p.wait(3000ms));
    CHECK(p.r.ok());

    // close 后再提交：回调仍触发（not_connected）
    AsyncProbe p2;
    am->read_holding_registers(0, 0, 2, p2.fn());
    REQUIRE(p2.wait(3000ms));
    CHECK(p2.r.ec == ModbusErrc::not_connected);

    am.reset();   // 修复前：~Impl 销毁 joinable 线程 → terminate
    slave->stop();
}

// ---------------------------------------------------------------------------
// 27. TCP 重连并发压测：auto_reconnect + 多线程（评审 M3/M4）
//
//     回归：m_conn 无同步读写 + ensure_connected 无串行化，断链后多
//     线程并发重连是 UB（TSan 可抓）且会向对端发起多条连接；旧连接
//     迟到回调无代际守卫，会误杀新连接上的在途事务。
//     短连接服务端（回完就断）+ 串行配额，最大化重连竞态窗口。
// ---------------------------------------------------------------------------
NETMECH_TEST(tcp_reconnect_serialized_under_concurrency)
{
    auto server = start_oneshot_server();
    REQUIRE(server.mech != nullptr);
    REQUIRE(server.port != 0u);
    const std::uint16_t port = server.port;

    Master::Config mc;
    mc.timeout        = 2000ms;
    mc.retries        = 40;      // busy/断链重试：并发争抢配额可能需多次
    mc.retry_delay    = 2ms;
    mc.auto_reconnect = true;
    mc.max_in_flight  = 1;       // 串行配额：突出重连竞态

    std::error_code ec;
    auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
    REQUIRE(master != nullptr);

    std::atomic<int> ok{0};
    auto worker = [&] {
        for (int i = 0; i < 10; ++i) {
            std::vector<std::uint16_t> regs;
            const auto r = master->read_holding_registers(0, 2, regs);
            if (r.ok() && regs.size() == 2u && regs[0] == 0x1234 && regs[1] == 0x5678) {
                ok.fetch_add(1);
            }
        }
    };
    std::thread t1(worker), t2(worker), t3(worker), t4(worker);
    t1.join();
    t2.join();
    t3.join();
    t4.join();

    CHECK_EQ(ok.load(), 40);                      // 全部成功，无丢笔/误杀
    CHECK(master->stats().reconnects >= 20u);     // 短连接：几乎每笔都重连

    master->close();
    server.mech->close_all();
}

// ---------------------------------------------------------------------------
// 28. 主站统计归因：断链/静默丢弃不再混进 timeouts（评审 m6）
// ---------------------------------------------------------------------------
namespace {

/// 连上就关的服务端：制造"请求在途时断链"的接收侧错误。
class DropSession final : public netmech::ConnHandler {
public:
    bool on_open(std::shared_ptr<netmech::Conn> c) override
    {
        c->close();
        return true;
    }
    void on_read(std::error_code, netmech::Buf, std::size_t) override {}
    void on_write(std::error_code, netmech::ConstBuf, std::size_t) override {}
    void on_close() override {}
};

class DropLink final : public netmech::Linkable {
public:
    std::shared_ptr<netmech::ConnHandler> on_accept(netmech::Conn&) override
    {
        return std::make_shared<DropSession>();
    }
};

} // namespace

NETMECH_TEST(tcp_stats_errors_attributed_correctly)
{
    // 场景 A：静默丢弃（不应答）⇒ 真超时，timeouts=1、recv_errors=0
    {
        std::uint16_t port = 0;
        auto bank = std::make_shared<Bank>(10);
        Slave::Config sc;
        sc.addresses = {1};            // 只应答 unit 1
        std::error_code ec;
        auto slave = make_test_slave(sc, bank, ec, port);
        REQUIRE(slave != nullptr);

        Master::Config mc;
        mc.unit    = 2;                // 不在名单 ⇒ 静默丢弃
        mc.timeout = 200ms;
        mc.retries = 0;
        auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
        REQUIRE(master != nullptr);

        std::vector<std::uint16_t> regs;
        const auto r = master->read_holding_registers(0, 2, regs);
        CHECK(r.ec == ModbusErrc::timeout);
        const auto s = master->stats();
        CHECK_EQ(s.timeouts, std::uint64_t{1});
        CHECK_EQ(s.recv_errors, std::uint64_t{0});

        master->close();
        slave->stop();
    }

    // 场景 B：连上就关 ⇒ 非超时错误（send_errors 或 recv_errors），timeouts=0
    {
        auto mech = netmech::Mechanism::create();
        REQUIRE(mech != nullptr);
        auto link = std::make_shared<DropLink>();
        REQUIRE(!mech->listen(netmech::Address::any(0), link));
        const auto addrs = mech->listening_addresses();
        REQUIRE(!addrs.empty());
        const std::uint16_t port = addrs.front().port();

        Master::Config mc;
        mc.timeout = 1000ms;
        mc.retries = 0;
        mc.auto_reconnect = false;
        std::error_code ec;
        auto master = Master::create_tcp("127.0.0.1", port, mc, ec);
        REQUIRE(master != nullptr);

        std::vector<std::uint16_t> regs;
        const auto r = master->read_holding_registers(0, 2, regs);
        CHECK(!r.ok());
        const auto s = master->stats();
        CHECK_EQ(s.timeouts, std::uint64_t{0});   // 修复前：断链被计成超时
        CHECK_EQ(s.send_errors + s.recv_errors, std::uint64_t{1});

        master->close();
        mech->close_all();
    }
}