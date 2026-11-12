// ============================================================================
//  test_rtu.cpp —— RTU 主站端到端测试
//
//  用 POSIX pty 建一对虚拟串口：
//      Master（RTU）  ←→  /dev/pts/N  ←→  pty master（测试线程里手写"假从站"）
//
//  假从站直接读写 pty master，手工解析请求、构造响应——不走库，独立验证。
// ============================================================================
#include <modbusmech/master.hpp>
#include <modbusmech/pdu.hpp>
#include <modbusmech/slave.hpp>

#include <netmech/net.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string>
#include <system_error>
#include <utility>

#include "test_framework.hpp"

#if defined(__linux__)
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#endif

using namespace modbusmech;
using namespace netmech_test;

namespace {

using namespace std::chrono_literals;

#if defined(__linux__)

struct Pty {
    int         master = -1;
    std::string slave;

    bool open_()
    {
        master = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master < 0) { return false; }
        if (::grantpt(master) != 0) { return false; }
        if (::unlockpt(master) != 0) { return false; }
        const char* name = ::ptsname(master);
        if (!name) { return false; }
        slave = name;
        const int fl = ::fcntl(master, F_GETFL, 0);
        if (fl >= 0) { ::fcntl(master, F_SETFL, fl | O_NONBLOCK); }
        return true;
    }

    ~Pty() { if (master >= 0) { ::close(master); } }
};

/// 读满 n 字节（非阻塞轮询）。
bool read_exact(int fd, std::uint8_t* buf, std::size_t n,
                std::chrono::milliseconds timeout = 2000ms)
{
    std::size_t got = 0;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (got < n && std::chrono::steady_clock::now() < deadline) {
        const ssize_t r = ::read(fd, buf + got, n - got);
        if (r > 0) {
            got += static_cast<std::size_t>(r);
        } else {
            std::this_thread::sleep_for(1ms);
        }
    }
    return got == n;
}

/// 追加 CRC 并写回。
void write_adu(int fd, std::uint8_t unit, const std::vector<std::uint8_t>& pdu)
{
    std::vector<std::uint8_t> frame;
    frame.push_back(unit);
    frame.insert(frame.end(), pdu.begin(), pdu.end());
    const std::uint16_t crc = crc16(span<const std::uint8_t>(frame.data(), frame.size()));
    frame.push_back(static_cast<std::uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));
    const ssize_t w = ::write(fd, frame.data(), frame.size());
    (void)w;
}

/// 后台线程跑一个"只会读保持寄存器"的假从站。
class FakeSlave {
public:
    FakeSlave(int fd, std::uint8_t unit, std::vector<std::uint16_t> regs)
        : m_fd(fd), m_unit(unit), m_regs(std::move(regs))
    {
        m_thread = std::thread([this] { run(); });
    }

    ~FakeSlave() { stop(); }

    void stop()
    {
        m_stop.store(true);
        if (m_thread.joinable()) { m_thread.join(); }
    }

    std::atomic<int> handled{0};

private:
    void run()
    {
        std::uint8_t buf[300];
        while (!m_stop.load()) {
            // 先读地址 + 功能码
            if (!read_exact(m_fd, buf, 2, 50ms)) { continue; }

            if (buf[0] != m_unit) { continue; }   // 不是给我的

            const Function f = static_cast<Function>(buf[1]);
            if (f == Function::read_holding_registers) {
                if (!read_exact(m_fd, buf + 2, 6, 500ms)) { continue; }  // addr(2)+cnt(2)+crc(2)
                const std::uint16_t addr = static_cast<std::uint16_t>((buf[2] << 8) | buf[3]);
                const std::uint16_t cnt  = static_cast<std::uint16_t>((buf[4] << 8) | buf[5]);

                std::vector<std::uint8_t> resp;
                if (addr + cnt > m_regs.size()) {
                    resp = {static_cast<std::uint8_t>(0x83), 0x02};   // 异常码 02
                } else {
                    resp.push_back(0x03);
                    resp.push_back(static_cast<std::uint8_t>(cnt * 2));
                    for (std::uint16_t i = 0; i < cnt; ++i) {
                        const std::uint16_t v = m_regs[addr + i];
                        resp.push_back(static_cast<std::uint8_t>(v >> 8));
                        resp.push_back(static_cast<std::uint8_t>(v & 0xFF));
                    }
                }
                write_adu(m_fd, m_unit, resp);
                ++handled;
            } else if (f == Function::write_single_register) {
                if (!read_exact(m_fd, buf + 2, 6, 500ms)) { continue; }
                const std::uint16_t addr = static_cast<std::uint16_t>((buf[2] << 8) | buf[3]);
                const std::uint16_t val  = static_cast<std::uint16_t>((buf[4] << 8) | buf[5]);
                if (addr < m_regs.size()) { m_regs[addr] = val; }
                // 回显请求（去掉 CRC）
                std::vector<std::uint8_t> resp(buf + 1, buf + 6);
                write_adu(m_fd, m_unit, resp);
                ++handled;
            }
        }
    }

    int                       m_fd;
    std::uint8_t              m_unit;
    std::vector<std::uint16_t> m_regs;
    std::atomic<bool>         m_stop{false};
    std::thread               m_thread;
};

#endif

} // namespace

// ---------------------------------------------------------------------------
// 1. 读保持寄存器（正常）
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_master_read_holding_registers)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    FakeSlave slave(pty.master, 1, {0x1111, 0x2222, 0x3333, 0x4444, 0x5555});

    Master::Config cfg;
    cfg.unit    = 1;
    cfg.timeout = 1000ms;
    cfg.retries = 0;

    std::error_code ec;
    auto master = Master::create_rtu(pty.slave, cfg, 9600, ec);
    REQUIRE(master != nullptr);

    std::vector<std::uint16_t> regs;
    const Result r = master->read_holding_registers(1, 3, regs);

    CHECK_MSG(r.ok(), r.describe().c_str());
    REQUIRE(regs.size() == 3u);
    CHECK_EQ(regs[0], 0x2222);
    CHECK_EQ(regs[1], 0x3333);
    CHECK_EQ(regs[2], 0x4444);

    slave.stop();
    master->close();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 2. 从站返回异常码（地址越界）
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_master_gets_exception)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    FakeSlave slave(pty.master, 1, {0x1111, 0x2222});   // 只有 2 个寄存器

    Master::Config cfg;
    cfg.unit    = 1;
    cfg.timeout = 1000ms;
    cfg.retries = 0;

    std::error_code ec;
    auto master = Master::create_rtu(pty.slave, cfg, 9600, ec);
    REQUIRE(master != nullptr);

    std::vector<std::uint16_t> regs;
    const Result r = master->read_holding_registers(0, 5, regs);   // 越界

    CHECK(!r.ok());
    CHECK(r.ec.value() == 0);                                        // 传输成功
    CHECK(r.exception == Exception::illegal_data_address);           // 从站拒绝
    CHECK(!r.retryable());                                           // 协议异常不该重试

    slave.stop();
    master->close();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 3. 写单个寄存器（回显校验）
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_master_write_single_register)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    FakeSlave slave(pty.master, 1, {0x0000, 0x0000, 0x0000});

    Master::Config cfg;
    cfg.unit    = 1;
    cfg.timeout = 1000ms;
    cfg.retries = 0;

    std::error_code ec;
    auto master = Master::create_rtu(pty.slave, cfg, 9600, ec);
    REQUIRE(master != nullptr);

    const Result r = master->write_single_register(1, 0xBEEF);
    CHECK_MSG(r.ok(), r.describe().c_str());

    slave.stop();
    master->close();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 4. 无响应 → 超时
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_master_timeout)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    // 不起假从站，没人应答
    Master::Config cfg;
    cfg.unit    = 1;
    cfg.timeout = 200ms;
    cfg.retries = 0;

    std::error_code ec;
    auto master = Master::create_rtu(pty.slave, cfg, 9600, ec);
    REQUIRE(master != nullptr);

    std::vector<std::uint16_t> regs;
    const auto t0 = std::chrono::steady_clock::now();
    const Result r = master->read_holding_registers(0, 1, regs);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0);

    CHECK(!r.ok());
    CHECK(r.ec == ModbusErrc::timeout);
    CHECK(r.retryable());                       // 传输失败可以重试
    CHECK(elapsed >= 150ms);                    // 确实等过

    master->close();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 5. 统计计数
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_master_stats)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    FakeSlave slave(pty.master, 1, {0x10, 0x20, 0x30});

    Master::Config cfg;
    cfg.unit    = 1;
    cfg.timeout = 1000ms;
    cfg.retries = 0;

    std::error_code ec;
    auto master = Master::create_rtu(pty.slave, cfg, 9600, ec);
    REQUIRE(master != nullptr);

    std::vector<std::uint16_t> regs;
    (void)master->read_holding_registers(0, 3, regs);
    (void)master->read_holding_registers(0, 3, regs);

    const auto st = master->stats();
    CHECK(st.requests >= 2);
    CHECK_EQ(st.succeeded, 2u);
    CHECK_EQ(st.timeouts, 0u);

    slave.stop();
    master->close();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 6. 串口参数校验：非法 data_bits / 越界枚举值 ⇒ invalid_argument
//
//    串口参数以前写死 8N1（8E1/8O1 设备根本连不上），现在由 Config 透传；
//    但必须在**打开串口之前**就拦住非法值，而不是让它落到 termios 上。
//    这条不依赖 pty，Windows 上同样能跑。
// ---------------------------------------------------------------------------
NETMECH_TEST(master_rejects_bad_serial_params)
{
    Master::Config cfg;
    std::error_code ec;

    cfg.data_bits = 9;                              // 只支持 5~8
    auto m = Master::create_rtu("/dev/null", cfg, 9600, ec);
    CHECK(m == nullptr);
    CHECK_EQ(ec, make_error_code(ModbusErrc::invalid_argument));

    cfg.data_bits = 8;
    cfg.parity    = static_cast<SerialParity>(7);   // 越界枚举值
    m = Master::create_rtu("/dev/null", cfg, 9600, ec);
    CHECK(m == nullptr);
    CHECK_EQ(ec, make_error_code(ModbusErrc::invalid_argument));

    cfg.parity    = SerialParity::none;
    cfg.stop_bits = static_cast<SerialStopBits>(5); // 越界枚举值
    m = Master::create_rtu("/dev/null", cfg, 9600, ec);
    CHECK(m == nullptr);
    CHECK_EQ(ec, make_error_code(ModbusErrc::invalid_argument));
}

// ---------------------------------------------------------------------------
// 7. RTU 广播写（unit=0）：发送即返回，从站只执行一次（评审 M5）
//
//    回归：旧实现无 unit==0 分支，广播写傻等超时后按 retries 重发，
//    同一条写被从站执行多次（继电器等控制场景有危害）。
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_broadcast_write_executes_once_without_response)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    Master::Config cfg;
    cfg.unit        = 1;
    cfg.timeout     = 300ms;
    cfg.retries     = 2;      // 修复前：广播会等满超时并重发 2 次
    cfg.retry_delay = 10ms;

    std::error_code ec;
    auto master = Master::create_rtu(pty.slave, cfg, 9600, ec);
    REQUIRE(master != nullptr);

    // 假从站：只收不应答（广播语义），统计收到的帧数
    std::atomic<int> frames{0};
    std::thread listener([&] {
        std::uint8_t buf[8];
        while (frames.load() < 3) {
            if (!read_exact(pty.master, buf, 8, 400ms)) { break; }
            frames.fetch_add(1);
        }
    });

    const auto t0 = std::chrono::steady_clock::now();
    const auto r = master->write_single_register(0, 5, 0x1234);   // unit=0 广播
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0);

    CHECK(r.ok());                // 修复前：傻等超时，r.ec == timeout
    CHECK(elapsed < 200ms);       // 发送即返回，不等响应
    listener.join();
    CHECK_EQ(frames.load(), 1);   // 修复前：1 + retries = 3 帧（重复执行）

    // 广播读：无意义，直接拒绝，不上线
    std::vector<std::uint16_t> regs;
    const auto rr = master->read_holding_registers(0, 0, 1, regs);
    CHECK(rr.ec == ModbusErrc::invalid_argument);

    master->close();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 8. RTU 并发调用返回 busy（评审 m5）
//
//    回归：master.hpp 承诺并发调用拿 busy，但旧实现 RTU 无事务守卫，
//    两帧会交错写上总线（RS-485 物理冲突）。现用事务级独占标志。
// ---------------------------------------------------------------------------
NETMECH_TEST(rtu_concurrent_transaction_returns_busy)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());
    // 不放假从站：第一笔事务必然等满超时，占住总线

    Master::Config cfg;
    cfg.unit    = 1;
    cfg.timeout = 500ms;
    cfg.retries = 0;

    std::error_code ec;
    auto master = Master::create_rtu(pty.slave, cfg, 9600, ec);
    REQUIRE(master != nullptr);

    Result first_r;
    std::thread t1([&] {
        std::vector<std::uint16_t> regs;
        first_r = master->read_holding_registers(0, 1, regs);
    });

    std::this_thread::sleep_for(100ms);   // 等第一笔占住总线

    std::vector<std::uint16_t> regs2;
    const auto t0 = std::chrono::steady_clock::now();
    const auto r2 = master->read_holding_registers(0, 1, regs2);
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0);

    CHECK(r2.ec == ModbusErrc::busy);   // 修复前：无守卫，同样等满超时
    CHECK(elapsed < 200ms);

    t1.join();
    CHECK(first_r.ec == ModbusErrc::timeout);
    master->close();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 9. RTU 从站：bank 回调里调 stop() 不得 terminate（评审 M2 同族）
//
//    回归：Slave::stop() 无 self-id 守卫，帧线程上的 bank 回调里调
//    stop() 会自 join → std::system_error 在 noexcept 里抛出 → terminate。
// ---------------------------------------------------------------------------
namespace {

class StopOnWriteBank final : public RegisterBank {
public:
    /// self 的读写跨线程（主线程赋值 / 帧线程回调读），必须加锁：
    /// 裸 shared_ptr 的跨线程读写是数据竞争（TSan 会抓，同 netmech
    /// test_support.hpp SyncSlot 的教训）。
    void set_self(std::shared_ptr<Slave> s)
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        m_self = std::move(s);
    }

    std::atomic<int> writes{0};

    bool read_coils(std::uint8_t, std::uint16_t, std::uint16_t, std::vector<bool>&) override
    { return false; }
    bool read_discrete_inputs(std::uint8_t, std::uint16_t, std::uint16_t,
                              std::vector<bool>&) override { return false; }
    bool read_holding_registers(std::uint8_t, std::uint16_t, std::uint16_t,
                                std::vector<std::uint16_t>&) override { return false; }
    bool read_input_registers(std::uint8_t, std::uint16_t, std::uint16_t,
                              std::vector<std::uint16_t>&) override { return false; }
    bool write_single_coil(std::uint8_t, std::uint16_t, bool) override { return false; }
    bool write_single_register(std::uint8_t, std::uint16_t, std::uint16_t) override
    {
        writes.fetch_add(1);
        std::shared_ptr<Slave> s;
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            s = m_self;
        }
        if (s) {
            s->stop();   // 在帧线程回调里停机：修复前这里 terminate
        }
        return true;
    }

private:
    std::mutex             m_mtx;
    std::shared_ptr<Slave> m_self;   ///< create_rtu 之后由 set_self 回填
};

} // namespace

NETMECH_TEST(rtu_slave_stop_from_bank_callback_is_safe)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    auto bank = std::make_shared<StopOnWriteBank>();
    Slave::Config sc;
    sc.address  = 1;
    sc.boundary = Slave::Config::FrameBoundary::length;   // pty 一次到齐，length 更稳

    std::error_code ec;
    auto slave = Slave::create_rtu(pty.slave, 9600, sc, bank, ec);
    REQUIRE(slave != nullptr);
    bank->set_self(slave);

    // 手写一条 06 写请求帧（unit=1, addr=0, value=0x1234）
    const std::vector<std::uint8_t> pdu{0x06, 0x00, 0x00, 0x12, 0x34};
    write_adu(pty.master, 1, pdu);

    // 修复前：bank 回调里自 join → terminate，进程直接死
    bool stopped = false;
    for (int i = 0; i < 200; ++i) {
        if (!slave->is_running()) { stopped = true; break; }
        std::this_thread::sleep_for(10ms);
    }
    CHECK(stopped);
    CHECK_EQ(bank->writes.load(), 1);

    slave->stop();   // 幂等
#else
    CHECK(true);
#endif
}
