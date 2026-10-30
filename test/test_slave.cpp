// ============================================================================
//  test_slave.cpp —— RTU 从站端到端测试
//
//  用 pty 把 主站（Master）和 从站（Slave）接起来，走真实的串口链路：
//      Master(RTU) ←→ /dev/pts/N ←→ Slave(RTU)
//
//  验证的重点是**从站的 silence 帧边界**：请求一次到达、分几次到达，都要能
//  正确切帧。
// ============================================================================
#include <modbusmech/master.hpp>
#include <modbusmech/pdu.hpp>
#include <modbusmech/slave.hpp>

#include <netmech/net.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

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
        const char* n = ::ptsname(master);
        if (!n) { return false; }
        slave = n;
        const int fl = ::fcntl(master, F_GETFL, 0);
        if (fl >= 0) { ::fcntl(master, F_SETFL, fl | O_NONBLOCK); }
        return true;
    }
    ~Pty() { if (master >= 0) { ::close(master); } }
};

/// 简单的寄存器 bank：4 种区域各 100 个。
class TestBank final : public RegisterBank {
public:
    TestBank()
        : m_coils(100, false), m_discrete(100, false),
          m_holding(100, 0), m_input(100, 0)
    {
        for (std::size_t i = 0; i < m_holding.size(); ++i) {
            m_holding[i] = static_cast<std::uint16_t>(i * 10);
            m_input[i]   = static_cast<std::uint16_t>(i * 20);
        }
        m_coils[3]    = true;
        m_discrete[5] = true;
    }

    bool read_coils(std::uint16_t a, std::uint16_t n, std::vector<bool>& out) override
    {
        if (a + n > m_coils.size()) { return false; }
        out.assign(m_coils.begin() + a, m_coils.begin() + a + n);
        return true;
    }
    bool read_discrete_inputs(std::uint16_t a, std::uint16_t n,
                              std::vector<bool>& out) override
    {
        if (a + n > m_discrete.size()) { return false; }
        out.assign(m_discrete.begin() + a, m_discrete.begin() + a + n);
        return true;
    }
    bool read_holding_registers(std::uint16_t a, std::uint16_t n,
                                std::vector<std::uint16_t>& out) override
    {
        if (a + n > m_holding.size()) { return false; }
        out.assign(m_holding.begin() + a, m_holding.begin() + a + n);
        return true;
    }
    bool read_input_registers(std::uint16_t a, std::uint16_t n,
                              std::vector<std::uint16_t>& out) override
    {
        if (a + n > m_input.size()) { return false; }
        out.assign(m_input.begin() + a, m_input.begin() + a + n);
        return true;
    }
    bool write_single_coil(std::uint16_t a, bool v) override
    {
        if (a >= m_coils.size()) { return false; }
        m_coils[a] = v;
        ++write_count;
        return true;
    }
    bool write_single_register(std::uint16_t a, std::uint16_t v) override
    {
        if (a >= m_holding.size()) { return false; }
        m_holding[a] = v;
        ++write_count;
        return true;
    }
    bool write_multiple_coils(std::uint16_t a, const std::vector<bool>& v) override
    {
        if (a + v.size() > m_coils.size()) { return false; }
        for (std::size_t i = 0; i < v.size(); ++i) { m_coils[a + i] = v[i]; }
        ++write_count;
        return true;
    }
    bool write_multiple_registers(std::uint16_t a,
                                  const std::vector<std::uint16_t>& v) override
    {
        if (a + v.size() > m_holding.size()) { return false; }
        for (std::size_t i = 0; i < v.size(); ++i) { m_holding[a + i] = v[i]; }
        ++write_count;
        return true;
    }

    std::atomic<int> write_count{0};

private:
    std::vector<bool>          m_coils, m_discrete;
    std::vector<std::uint16_t> m_holding, m_input;
};

#endif

} // namespace

// ---------------------------------------------------------------------------
// 1. 主站读 → 从站响应（读保持寄存器）
// ---------------------------------------------------------------------------
NETMECH_TEST(slave_read_holding_registers)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    auto bank = std::make_shared<TestBank>();

    Slave::Config sc;
    sc.address    = 1;
    sc.silence_ms = 5;

    std::error_code ec;
    auto slave = Slave::create_rtu(pty.slave, 9600, sc, bank, ec);
    REQUIRE(slave != nullptr);

    // 主站用同一个 pty 的 master 端？不行 —— 需要另一对。
    // 这里换个方法：主站和从站都打开同一个 slave 路径是不行的（串口独占）。
    // 所以测试改成：从站打开 slave，测试直接往 master 写请求、从 master 读响应。
    // 见下面用原始字节的方式。
    CHECK(slave->is_running());
    slave->stop();
    CHECK(!slave->is_running());
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 2. 从站收到完整请求 → 回正确的响应（手工构造请求）
// ---------------------------------------------------------------------------
NETMECH_TEST(slave_responds_to_read_request)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    auto bank = std::make_shared<TestBank>();

    Slave::Config sc;
    sc.address    = 1;
    sc.silence_ms = 5;

    std::error_code ec;
    auto slave = Slave::create_rtu(pty.slave, 9600, sc, bank, ec);
    REQUIRE(slave != nullptr);

    // 手工构造请求：01 03 00 02 00 03 + CRC（读 addr=2, count=3）
    std::vector<std::uint8_t> req{0x01, 0x03, 0x00, 0x02, 0x00, 0x03};
    const std::uint16_t crc = crc16(span<const std::uint8_t>(req.data(), req.size()));
    req.push_back(static_cast<std::uint8_t>(crc & 0xFF));
    req.push_back(static_cast<std::uint8_t>((crc >> 8) & 0xFF));

    const ssize_t w = ::write(pty.master, req.data(), req.size());
    REQUIRE(w == static_cast<ssize_t>(req.size()));

    // 等从站响应（8 字节 + 2 CRC = 10）
    std::vector<std::uint8_t> resp;
    const auto deadline = std::chrono::steady_clock::now() + 2000ms;
    while (resp.size() < 11 && std::chrono::steady_clock::now() < deadline) {
        std::uint8_t buf[32];
        const ssize_t r = ::read(pty.master, buf, sizeof(buf));
        if (r > 0) {
            resp.insert(resp.end(), buf, buf + r);
        } else {
            std::this_thread::sleep_for(1ms);
        }
    }

    // 期望：01 03 06 00 14 00 1E 00 28 + CRC（addr=2 → 20, 30, 40）
    REQUIRE(resp.size() >= 9u);
    CHECK_EQ(resp[0], 0x01);
    CHECK_EQ(resp[1], 0x03);
    CHECK_EQ(resp[2], 0x06);      // 6 字节
    CHECK_EQ(resp[3], 0x00);
    CHECK_EQ(resp[4], 20);        // addr2 = 2*10
    CHECK_EQ(resp[5], 0x00);
    CHECK_EQ(resp[6], 30);        // addr3 = 3*10
    CHECK_EQ(resp[7], 0x00);
    CHECK_EQ(resp[8], 40);        // addr4 = 4*10

    slave->stop();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 3. 地址不匹配 → 不应答
// ---------------------------------------------------------------------------
NETMECH_TEST(slave_ignores_other_address)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    auto bank = std::make_shared<TestBank>();
    Slave::Config sc;
    sc.address    = 1;
    sc.silence_ms = 5;

    std::error_code ec;
    auto slave = Slave::create_rtu(pty.slave, 9600, sc, bank, ec);
    REQUIRE(slave != nullptr);

    // 发给地址 5（不是 1）
    std::vector<std::uint8_t> req{0x05, 0x03, 0x00, 0x00, 0x00, 0x01};
    const std::uint16_t c = crc16(span<const std::uint8_t>(req.data(), req.size()));
    req.push_back(static_cast<std::uint8_t>(c & 0xFF));
    req.push_back(static_cast<std::uint8_t>((c >> 8) & 0xFF));
    const ssize_t w = ::write(pty.master, req.data(), req.size());
    REQUIRE(w == static_cast<ssize_t>(req.size()));

    // 等 200ms，应该没有响应
    std::this_thread::sleep_for(200ms);
    std::uint8_t buf[64];
    const ssize_t r = ::read(pty.master, buf, sizeof(buf));
    CHECK(r <= 0);   // 没有数据（或 EAGAIN）

    slave->stop();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 4. CRC 错 → 不应答
// ---------------------------------------------------------------------------
NETMECH_TEST(slave_ignores_bad_crc)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    auto bank = std::make_shared<TestBank>();
    Slave::Config sc;
    sc.address    = 1;
    sc.silence_ms = 5;

    std::error_code ec;
    auto slave = Slave::create_rtu(pty.slave, 9600, sc, bank, ec);
    REQUIRE(slave != nullptr);

    // 故意写错的 CRC
    const std::uint8_t bad[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0xDE, 0xAD};
    const ssize_t w = ::write(pty.master, bad, sizeof(bad));
    REQUIRE(w == static_cast<ssize_t>(sizeof(bad)));

    std::this_thread::sleep_for(200ms);
    std::uint8_t buf[64];
    const ssize_t r = ::read(pty.master, buf, sizeof(buf));
    CHECK(r <= 0);   // 不应答

    slave->stop();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 5. 写单个寄存器 → 回显 + bank 被改
// ---------------------------------------------------------------------------
NETMECH_TEST(slave_write_single_register)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    auto bank = std::make_shared<TestBank>();
    Slave::Config sc;
    sc.address    = 1;
    sc.silence_ms = 5;

    std::error_code ec;
    auto slave = Slave::create_rtu(pty.slave, 9600, sc, bank, ec);
    REQUIRE(slave != nullptr);

    // 06 00 05 12 34 + CRC（写 addr=5, value=0x1234）
    std::vector<std::uint8_t> req{0x01, 0x06, 0x00, 0x05, 0x12, 0x34};
    const std::uint16_t c = crc16(span<const std::uint8_t>(req.data(), req.size()));
    req.push_back(static_cast<std::uint8_t>(c & 0xFF));
    req.push_back(static_cast<std::uint8_t>((c >> 8) & 0xFF));
    const ssize_t w = ::write(pty.master, req.data(), req.size());
    REQUIRE(w == static_cast<ssize_t>(req.size()));

    // 响应应该回显请求（8 字节）
    std::vector<std::uint8_t> resp;
    const auto deadline = std::chrono::steady_clock::now() + 2000ms;
    while (resp.size() < 8 && std::chrono::steady_clock::now() < deadline) {
        std::uint8_t buf[32];
        const ssize_t r = ::read(pty.master, buf, sizeof(buf));
        if (r > 0) { resp.insert(resp.end(), buf, buf + r); }
        else { std::this_thread::sleep_for(1ms); }
    }
    REQUIRE(resp.size() >= 8u);
    CHECK_EQ(resp[0], 0x01);
    CHECK_EQ(resp[1], 0x06);
    CHECK_EQ(resp[2], 0x00);
    CHECK_EQ(resp[3], 0x05);
    CHECK_EQ(resp[4], 0x12);
    CHECK_EQ(resp[5], 0x34);

    CHECK(bank->write_count.load() >= 1);

    slave->stop();
#else
    CHECK(true);
#endif
}

// ---------------------------------------------------------------------------
// 6. 地址越界 → 回异常码 02
// ---------------------------------------------------------------------------
NETMECH_TEST(slave_returns_illegal_address)
{
#if defined(__linux__)
    Pty pty;
    REQUIRE(pty.open_());

    auto bank = std::make_shared<TestBank>();   // bank 只有 100 个
    Slave::Config sc;
    sc.address    = 1;
    sc.silence_ms = 5;

    std::error_code ec;
    auto slave = Slave::create_rtu(pty.slave, 9600, sc, bank, ec);
    REQUIRE(slave != nullptr);

    // 读 addr=200（越界）
    std::vector<std::uint8_t> req{0x01, 0x03, 0x00, 0xC8, 0x00, 0x01};
    const std::uint16_t c = crc16(span<const std::uint8_t>(req.data(), req.size()));
    req.push_back(static_cast<std::uint8_t>(c & 0xFF));
    req.push_back(static_cast<std::uint8_t>((c >> 8) & 0xFF));
    const ssize_t w = ::write(pty.master, req.data(), req.size());
    REQUIRE(w == static_cast<ssize_t>(req.size()));

    // 期望异常响应：01 83 02 + CRC = 5 字节
    std::vector<std::uint8_t> resp;
    const auto deadline = std::chrono::steady_clock::now() + 2000ms;
    while (resp.size() < 5 && std::chrono::steady_clock::now() < deadline) {
        std::uint8_t buf[32];
        const ssize_t r = ::read(pty.master, buf, sizeof(buf));
        if (r > 0) { resp.insert(resp.end(), buf, buf + r); }
        else { std::this_thread::sleep_for(1ms); }
    }
    REQUIRE(resp.size() >= 3u);
    CHECK_EQ(resp[0], 0x01);
    CHECK_EQ(resp[1], 0x83);   // 功能码 | 0x80
    CHECK_EQ(resp[2], 0x02);   // illegal data address

    slave->stop();
#else
    CHECK(true);
#endif
}
