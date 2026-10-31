// =============================================================================
//  interop_test —— 与 libmodbus 的互通对照
// =============================================================================
//  两个方向都测：
//    1. libmodbus 当从站 ← 我们的 Master 读
//    2. 我们的从站 → libmodbus 当主站读
//
//  这是**验证工具**，不是主测试套件的一部分（依赖 libmodbus，而它是 LGPL）。
//  它不链接进 modbusmech，只在需要时单独编译运行。
//
//  用法：cmake 编译后 ./interop_test
//  退出码 0 = 两个方向都通过。
// =============================================================================
#include <modbusmech/master.hpp>
#include <modbusmech/slave.hpp>

#include <netmech/net.hpp>

#include <modbus.h>          // libmodbus

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <thread>
#include <vector>

using namespace modbusmech;

namespace {

using namespace std::chrono_literals;

constexpr std::uint16_t kPort = 5020;

int g_fail = 0;

void check(bool cond, const char* what)
{
    std::printf("  [%s] %s\n", cond ? " ok " : "FAIL", what);
    if (!cond) { ++g_fail; }
}

/// 给我们的从站用的 bank：holding[5]+ = 500+，input[5]+ = 700+。
class OurBank final : public RegisterBank {
public:
    bool read_coils(std::uint16_t, std::uint16_t, std::vector<bool>&) override { return false; }
    bool read_discrete_inputs(std::uint16_t, std::uint16_t,
                              std::vector<bool>&) override { return false; }
    bool read_holding_registers(std::uint16_t a, std::uint16_t n,
                                std::vector<std::uint16_t>& out) override
    {
        if (a + n > 100) { return false; }
        out.resize(n);
        for (std::uint16_t i = 0; i < n; ++i) {
            out[i] = static_cast<std::uint16_t>(500 + a + i);
        }
        return true;
    }
    bool read_input_registers(std::uint16_t a, std::uint16_t n,
                              std::vector<std::uint16_t>& out) override
    {
        if (a + n > 100) { return false; }
        out.resize(n);
        for (std::uint16_t i = 0; i < n; ++i) {
            out[i] = static_cast<std::uint16_t>(700 + a + i);
        }
        return true;
    }
    bool write_single_coil(std::uint16_t, bool) override { return false; }
    bool write_single_register(std::uint16_t, std::uint16_t) override { return true; }
    bool write_multiple_coils(std::uint16_t, const std::vector<bool>&) override { return false; }
    bool write_multiple_registers(std::uint16_t, const std::vector<std::uint16_t>&) override
    {
        return true;
    }
};

// =============================================================================
//  方向 1：libmodbus 从站 ← 我们的 Master
// =============================================================================

void test_libmodbus_slave_our_master()
{
    std::printf("--- 1. libmodbus 从站 ← 我们的 Master ---\n");

    // 起 libmodbus 的 TCP 从站（后台线程）
    std::atomic<bool> stop{false};
    std::atomic<int>  accepted{0};

    std::thread server([&] {
        modbus_t* ctx = modbus_new_tcp("127.0.0.1", kPort);
        if (!ctx) { return; }

        // 4 种区域各 100 个
        modbus_mapping_t* mb = modbus_mapping_new(100, 100, 100, 100);
        if (!mb) { modbus_free(ctx); return; }

        // holding[0..] = 1000+i，input[0..] = 2000+i
        for (int i = 0; i < 100; ++i) {
            mb->tab_registers[i] = static_cast<std::uint16_t>(1000 + i);
            mb->tab_input_registers[i] = static_cast<std::uint16_t>(2000 + i);
        }

        int s = modbus_tcp_listen(ctx, 1);
        if (s < 0) { modbus_mapping_free(mb); modbus_free(ctx); return; }

        modbus_tcp_accept(ctx, &s);
        accepted.store(1);

        std::uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        while (!stop.load()) {
            const int rc = modbus_receive(ctx, query);
            if (rc > 0) {
                modbus_reply(ctx, query, rc, mb);
            } else if (rc == -1) {
                // 连接断开或超时：再 accept 一次
                if (!stop.load()) {
                    modbus_tcp_accept(ctx, &s);
                }
            }
        }

        if (s >= 0) { close(s); }
        modbus_mapping_free(mb);
        modbus_free(ctx);
    });

    // 等从站起来
    for (int i = 0; i < 200 && !accepted.load(); ++i) {
        std::this_thread::sleep_for(10ms);
    }

    Master::Config mc;
    mc.unit    = 1;
    mc.timeout = 2000ms;
    mc.retries = 1;

    std::error_code ec;
    auto master = Master::create_tcp("127.0.0.1", kPort, mc, ec);
    if (!master) {
        check(false, "连接到 libmodbus 从站");
        stop.store(true);
        server.join();
        return;
    }
    check(true, "连接到 libmodbus 从站");

    // 读保持寄存器
    {
        std::vector<std::uint16_t> regs;
        const Result r = master->read_holding_registers(5, 3, regs);
        check(r.ok(), ("读保持寄存器: " + r.describe()).c_str());
        if (r.ok() && regs.size() == 3) {
            check(regs[0] == 1005 && regs[1] == 1006 && regs[2] == 1007,
                  "读到的值正确（1005/1006/1007）");
        }
    }

    // 读输入寄存器
    {
        std::vector<std::uint16_t> regs;
        const Result r = master->read_input_registers(2, 2, regs);
        check(r.ok(), ("读输入寄存器: " + r.describe()).c_str());
        if (r.ok() && regs.size() == 2) {
            check(regs[0] == 2002 && regs[1] == 2003, "值正确（2002/2003）");
        }
    }

    // 写单个寄存器
    {
        const Result r = master->write_single_register(10, 0x1234);
        check(r.ok(), ("写单寄存器: " + r.describe()).c_str());
    }

    // 写多个寄存器
    {
        const Result r = master->write_multiple_registers(20, {0xAAAA, 0xBBBB});
        check(r.ok(), ("写多寄存器: " + r.describe()).c_str());
    }

    master->close();
    stop.store(true);
    if (server.joinable()) { server.join(); }
}

// =============================================================================
//  方向 2：我们的从站 → libmodbus 主站
// =============================================================================

void test_our_slave_libmodbus_master()
{
    std::printf("--- 2. 我们的从站 → libmodbus 主站 ---\n");

    auto bank = std::make_shared<OurBank>();
    Slave::Config sc;
    std::error_code ec;
    auto slave = Slave::create_tcp(kPort + 1, sc, bank, ec);
    if (!slave) {
        check(false, "启动我们的 TCP 从站");
        return;
    }
    check(true, "启动我们的 TCP 从站");
    std::this_thread::sleep_for(100ms);

    modbus_t* ctx = modbus_new_tcp("127.0.0.1", kPort + 1);
    if (!ctx) {
        check(false, "libmodbus 创建上下文");
        slave->stop();
        return;
    }
    if (modbus_connect(ctx) != 0) {
        check(false, "libmodbus 连接我们的从站");
        modbus_free(ctx);
        slave->stop();
        return;
    }
    check(true, "libmodbus 连接我们的从站");

    modbus_set_slave(ctx, 1);

    // 读保持寄存器
    {
        std::uint16_t regs[3] = {};
        const int rc = modbus_read_registers(ctx, 5, 3, regs);
        check(rc == 3, "libmodbus 读保持寄存器（返回 3 个）");
        if (rc == 3) {
            check(regs[0] == 505 && regs[1] == 506 && regs[2] == 507,
                  "值正确（505/506/507）");
        }
    }

    // 读输入寄存器
    {
        std::uint16_t regs[2] = {};
        const int rc = modbus_read_input_registers(ctx, 3, 2, regs);
        check(rc == 2, "libmodbus 读输入寄存器");
        if (rc == 2) {
            check(regs[0] == 703 && regs[1] == 704, "值正确（703/704）");
        }
    }

    // 写单个寄存器
    {
        const int rc = modbus_write_register(ctx, 10, 0x5678);
        check(rc == 1, "libmodbus 写单寄存器");
    }

    // 越界读 → 应该拿到异常码
    {
        std::uint16_t regs[1] = {};
        const int rc = modbus_read_registers(ctx, 200, 1, regs);
        check(rc == -1, "越界读被拒绝（返回 -1）");
        check(errno == EMBXILADD, "异常码是 illegal data address");
    }

    modbus_close(ctx);
    modbus_free(ctx);
    slave->stop();
}

} // namespace

int main()
{
    std::printf("modbusmech <-> libmodbus 互通测试\n\n");

    test_libmodbus_our_dummy_placeholder:

    test_libmodbus_slave_our_master();
    std::this_thread::sleep_for(200ms);
    test_our_slave_libmodbus_master();

    std::printf("\n%s\n", g_fail == 0 ? "PASS" : "FAIL");
    return g_fail == 0 ? 0 : 1;
}
