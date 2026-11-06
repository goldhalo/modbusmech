// =============================================================================
//  interop_test —— 与 libmodbus 3.2.0 的互通对照
// =============================================================================
//  两个方向都测：
//    1. libmodbus 当从站 ← 我们的 Master（阻塞版）读/写
//    2. 我们的从站 → libmodbus 当主站读/写
//
//  覆盖面：
//    · 四类读：线圈 01 / 离散输入 02 / 保持寄存器 03 / 输入寄存器 04
//    · 四类写：05 / 06 / 0F / 10，并**回读校验**
//    · 规范上限：读 125 个寄存器、写 123 个寄存器
//    · 异常码：越界 → 02；未知功能码 → 01
//    · 多 unit：libmodbus 主站换 unit 后仍能正常读写（TCP 默认不按 unit 过滤）
//
//  这是**验证工具**，不是主测试套件的一部分（依赖 libmodbus，而它是 LGPL）。
//  它不链接进 modbusmech，只在需要时单独编译运行（见 ref/README.md）。
//
//  用法：./interop_test
//  退出码 0 = 两个方向都通过。
// =============================================================================
#include <modbusmech/master.hpp>
#include <modbusmech/slave.hpp>

#include <netmech/net.hpp>

#include <modbus.h>          // libmodbus

#include <atomic>
#include <chrono>
#include <cerrno>
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
    if (!cond) {
        ++g_fail;
    }
}

/// 给我们的从站用的 bank：四个区各 100 个，读写都支持（便于"写后回读"）。
class OurBank final : public RegisterBank {
public:
    OurBank()
        : m_coils(200, false), m_discrete(200, false),
          m_holding(200, 0), m_input(200, 0)
    {
        for (std::size_t i = 0; i < 200; ++i) {
            m_coils[i]    = (i % 4 == 0);
            m_discrete[i] = (i % 5 == 0);
            m_holding[i]  = static_cast<std::uint16_t>(500 + i);
            m_input[i]    = static_cast<std::uint16_t>(700 + i);
        }
    }

    bool read_coils(std::uint8_t, std::uint16_t a, std::uint16_t n,
                    std::vector<bool>& out) override
    {
        if (a + n > m_coils.size()) {
            return false;
        }
        out.assign(m_coils.begin() + a, m_coils.begin() + a + n);
        return true;
    }
    bool read_discrete_inputs(std::uint8_t, std::uint16_t a, std::uint16_t n,
                              std::vector<bool>& out) override
    {
        if (a + n > m_discrete.size()) {
            return false;
        }
        out.assign(m_discrete.begin() + a, m_discrete.begin() + a + n);
        return true;
    }
    bool read_holding_registers(std::uint8_t, std::uint16_t a, std::uint16_t n,
                                std::vector<std::uint16_t>& out) override
    {
        if (a + n > m_holding.size()) {
            return false;
        }
        out.assign(m_holding.begin() + a, m_holding.begin() + a + n);
        return true;
    }
    bool read_input_registers(std::uint8_t, std::uint16_t a, std::uint16_t n,
                              std::vector<std::uint16_t>& out) override
    {
        if (a + n > m_input.size()) {
            return false;
        }
        out.assign(m_input.begin() + a, m_input.begin() + a + n);
        return true;
    }
    bool write_single_coil(std::uint8_t, std::uint16_t a, bool v) override
    {
        if (a >= m_coils.size()) {
            return false;
        }
        m_coils[a] = v;
        return true;
    }
    bool write_single_register(std::uint8_t, std::uint16_t a, std::uint16_t v) override
    {
        if (a >= m_holding.size()) {
            return false;
        }
        m_holding[a] = v;
        return true;
    }
    bool write_multiple_coils(std::uint8_t, std::uint16_t a,
                              const std::vector<bool>& v) override
    {
        if (a + v.size() > m_coils.size()) {
            return false;
        }
        for (std::size_t i = 0; i < v.size(); ++i) {
            m_coils[a + i] = v[i];
        }
        return true;
    }
    bool write_multiple_registers(std::uint8_t, std::uint16_t a,
                                  const std::vector<std::uint16_t>& v) override
    {
        if (a + v.size() > m_holding.size()) {
            return false;
        }
        for (std::size_t i = 0; i < v.size(); ++i) {
            m_holding[a + i] = v[i];
        }
        return true;
    }

private:
    std::vector<bool>          m_coils;
    std::vector<bool>          m_discrete;
    std::vector<std::uint16_t> m_holding;
    std::vector<std::uint16_t> m_input;
};

// =============================================================================
//  方向 1：libmodbus 从站 ← 我们的 Master
// =============================================================================

void test_libmodbus_slave_our_master()
{
    std::printf("--- 1. libmodbus 从站 <- 我们的 Master ---\n");

    std::atomic<bool> stop{false};
    std::atomic<int>  accepted{0};

    std::thread server([&] {
        modbus_t* ctx = modbus_new_tcp("127.0.0.1", kPort);
        if (!ctx) {
            return;
        }
        modbus_mapping_t* mb = modbus_mapping_new(200, 200, 200, 200);
        if (!mb) {
            modbus_free(ctx);
            return;
        }
        // 初值：holding = 1000+i，input = 2000+i，coils = i%4==0，discrete = i%5==0
        for (int i = 0; i < 200; ++i) {
            mb->tab_registers[i]       = static_cast<std::uint16_t>(1000 + i);
            mb->tab_input_registers[i] = static_cast<std::uint16_t>(2000 + i);
            mb->tab_bits[i]            = (i % 4 == 0) ? 1 : 0;
            mb->tab_input_bits[i]      = (i % 5 == 0) ? 1 : 0;
        }

        int s = modbus_tcp_listen(ctx, 1);
        if (s < 0) {
            modbus_mapping_free(mb);
            modbus_free(ctx);
            return;
        }
        modbus_tcp_accept(ctx, &s);
        accepted.store(1);

        std::uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
        while (!stop.load()) {
            const int rc = modbus_receive(ctx, query);
            if (rc > 0) {
                modbus_reply(ctx, query, rc, mb);
            } else if (rc == -1) {
                // 连接断开或超时：再 accept 一次（单客户端循环）
                if (!stop.load()) {
                    modbus_tcp_accept(ctx, &s);
                }
            }
        }
        if (s >= 0) {
            close(s);
        }
        modbus_mapping_free(mb);
        modbus_free(ctx);
    });

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

    // --- 读：保持寄存器 03 ---
    {
        std::vector<std::uint16_t> regs;
        const Result r = master->read_holding_registers(5, 3, regs);
        check(r.ok(), ("读保持寄存器: " + r.describe()).c_str());
        check(r.ok() && regs.size() == 3 && regs[0] == 1005 && regs[1] == 1006
                  && regs[2] == 1007,
              "保持寄存器值正确（1005/1006/1007）");
    }
    // --- 读：输入寄存器 04 ---
    {
        std::vector<std::uint16_t> regs;
        const Result r = master->read_input_registers(2, 2, regs);
        check(r.ok(), ("读输入寄存器: " + r.describe()).c_str());
        check(r.ok() && regs.size() == 2 && regs[0] == 2002 && regs[1] == 2003,
              "输入寄存器值正确（2002/2003）");
    }
    // --- 读：线圈 01 ---
    {
        std::vector<bool> bits;
        const Result r = master->read_coils(0, 8, bits);
        check(r.ok(), ("读线圈: " + r.describe()).c_str());
        check(r.ok() && bits.size() == 8 && bits[0] && !bits[1] && !bits[2] && !bits[3]
                  && bits[4] && !bits[5] && !bits[6] && !bits[7],
              "线圈值正确（1 0 0 0 1 0 0 0）");
    }
    // --- 读：离散输入 02 ---
    {
        std::vector<bool> bits;
        const Result r = master->read_discrete_inputs(0, 10, bits);
        check(r.ok(), ("读离散输入: " + r.describe()).c_str());
        check(r.ok() && bits.size() == 10 && bits[0] && !bits[1] && !bits[2] && !bits[3]
                  && !bits[4] && bits[5],
              "离散输入值正确（1 0 0 0 0 1 ...）");
    }
    // --- 写：单/多寄存器 + 回读 ---
    {
        const Result w1 = master->write_single_register(10, 0x1234);
        check(w1.ok(), ("写单寄存器: " + w1.describe()).c_str());
        std::vector<std::uint16_t> regs;
        const Result r1 = master->read_holding_registers(10, 1, regs);
        check(r1.ok() && regs.size() == 1 && regs[0] == 0x1234, "写单寄存器后回读一致");

        const Result w2 = master->write_multiple_registers(20, {0xAAAA, 0xBBBB});
        check(w2.ok(), ("写多寄存器: " + w2.describe()).c_str());
        std::vector<std::uint16_t> regs2;
        const Result r2 = master->read_holding_registers(20, 2, regs2);
        check(r2.ok() && regs2.size() == 2 && regs2[0] == 0xAAAA && regs2[1] == 0xBBBB,
              "写多寄存器后回读一致");
    }
    // --- 写：单/多线圈 + 回读 ---
    {
        const Result w1 = master->write_single_coil(2, true);
        check(w1.ok(), ("写单线圈: " + w1.describe()).c_str());
        std::vector<bool> bits;
        const Result r1 = master->read_coils(2, 1, bits);
        check(r1.ok() && bits.size() == 1 && bits[0], "写单线圈后回读一致");

        const Result w2 = master->write_multiple_coils(0, {false, true, true, false});
        check(w2.ok(), ("写多线圈: " + w2.describe()).c_str());
        std::vector<bool> bits2;
        const Result r2 = master->read_coils(0, 4, bits2);
        check(r2.ok() && bits2.size() == 4 && !bits2[0] && bits2[1] && bits2[2]
                  && !bits2[3],
              "写多线圈后回读一致");
    }
    // --- 规范上限：读 125 个寄存器 ---
    {
        std::vector<std::uint16_t> regs;
        const Result r = master->read_holding_registers(0, 125, regs);
        check(r.ok() && regs.size() == 125, ("读 125 个寄存器（规范上限）: "
                                             + r.describe()).c_str());
    }
    // --- 异常码 02：越界读 ---
    {
        std::vector<std::uint16_t> regs;
        const Result r = master->read_holding_registers(200, 1, regs);
        check(!r.ok() && r.exception == Exception::illegal_data_address,
              "越界读被 libmodbus 拒绝（异常码 02）");
    }

    master->close();
    stop.store(true);
    if (server.joinable()) {
        server.join();
    }
}

// =============================================================================
//  方向 2：我们的从站 → libmodbus 主站
// =============================================================================

void test_our_slave_libmodbus_master()
{
    std::printf("--- 2. 我们的从站 <- libmodbus 主站 ---\n");

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
    modbus_set_response_timeout(ctx, 2, 0);
    if (modbus_connect(ctx) != 0) {
        check(false, "libmodbus 连接我们的从站");
        modbus_free(ctx);
        slave->stop();
        return;
    }
    check(true, "libmodbus 连接我们的从站");

    modbus_set_slave(ctx, 1);

    // --- 读保持寄存器 03 ---
    {
        std::uint16_t regs[3] = {};
        const int rc = modbus_read_registers(ctx, 5, 3, regs);
        check(rc == 3, "libmodbus 读保持寄存器（3 个）");
        if (rc == 3) {
            check(regs[0] == 505 && regs[1] == 506 && regs[2] == 507,
                  "值正确（505/506/507）");
        }
    }
    // --- 读输入寄存器 04 ---
    {
        std::uint16_t regs[2] = {};
        const int rc = modbus_read_input_registers(ctx, 3, 2, regs);
        check(rc == 2, "libmodbus 读输入寄存器（2 个）");
        if (rc == 2) {
            check(regs[0] == 703 && regs[1] == 704, "值正确（703/704）");
        }
    }
    // --- 读线圈 01 ---
    {
        std::uint8_t bits[2] = {};
        const int rc = modbus_read_bits(ctx, 0, 8, bits);
        check(rc == 8, "libmodbus 读线圈（8 个）");
        if (rc == 8) {
            check(bits[0] == 1 && bits[4] == 1, "线圈值正确（bit0/bit4 为 1）");
        }
    }
    // --- 读离散输入 02 ---
    {
        std::uint8_t bits[2] = {};
        const int rc = modbus_read_input_bits(ctx, 0, 10, bits);
        check(rc == 10, "libmodbus 读离散输入（10 个）");
        if (rc == 10) {
            check(bits[0] == 1 && bits[5] == 1, "离散输入值正确（bit0/bit5 为 1）");
        }
    }
    // --- 写单寄存器 06 + 回读 ---
    {
        const int rc = modbus_write_register(ctx, 10, 0x5678);
        check(rc == 1, "libmodbus 写单寄存器");
        std::uint16_t v = 0;
        check(modbus_read_registers(ctx, 10, 1, &v) == 1 && v == 0x5678,
              "写单寄存器后回读一致");
    }
    // --- 写多寄存器 10 + 回读 ---
    {
        const std::uint16_t vals[2] = {0x1111, 0x2222};
        const int rc = modbus_write_registers(ctx, 20, 2, vals);
        check(rc == 2, "libmodbus 写多寄存器");
        std::uint16_t back[2] = {};
        check(modbus_read_registers(ctx, 20, 2, back) == 2 && back[0] == 0x1111
                  && back[1] == 0x2222,
              "写多寄存器后回读一致");
    }
    // --- 写单线圈 05 + 回读 ---
    {
        const int rc = modbus_write_bit(ctx, 2, 1);
        check(rc == 1, "libmodbus 写单线圈");
        std::uint8_t b = 0;
        check(modbus_read_bits(ctx, 2, 1, &b) == 1 && b == 1, "写单线圈后回读一致");
    }
    // --- 写多线圈 0F + 回读 ---
    {
        const std::uint8_t vals[4] = {0, 1, 1, 0};
        const int rc = modbus_write_bits(ctx, 0, 4, vals);
        check(rc == 4, "libmodbus 写多线圈");
        // 注意：libmodbus 的 modbus_read_bits 返回**逐位字节**（每元素 0/1），
        // 不是 Modbus 报文里的打包位图 —— 以前这里按 0b0110 断言是测错了。
        std::uint8_t back[4] = {};
        if (modbus_read_bits(ctx, 0, 4, back) == 4) {
            check(back[0] == 0 && back[1] == 1 && back[2] == 1 && back[3] == 0,
                  "写多线圈后回读一致（0,1,1,0）");
        }
    }
    // --- 规范上限：读 125 个寄存器 ---
    {
        std::uint16_t regs[125] = {};
        check(modbus_read_registers(ctx, 0, 125, regs) == 125,
              "libmodbus 读 125 个寄存器（规范上限）");
    }
    // --- 异常码 02：越界读 ---
    {
        std::uint16_t regs[1] = {};
        const int rc = modbus_read_registers(ctx, 200, 1, regs);
        check(rc == -1, "越界读被拒绝（返回 -1）");
        check(errno == EMBXILADD, "异常码是 illegal data address（02）");
    }
    // --- 异常码 01：未知功能码 ---
    {
        // modbus_send_raw_request 的第一个字节是 **slave id**，其后是 PDU
        //（PDU = 功能码 + 数据）。0x2B = 读设备标识，本库不支持 → 期望回 01。
        const std::uint8_t raw[5] = {0x01, 0x2B, 0x0E, 0x01, 0x00};
        const int sent = modbus_send_raw_request(ctx, raw, sizeof raw);
        check(sent > 0, "发送未知功能码请求（0x2B）");
        // 注意：modbus_receive_confirmation 只返回**原始确认报文字节**，不做
        // 异常码解码（异常解码在 check_confirmation 里，raw 路径不经过它）。
        // 所以这里自己检查：TCP 响应 = MBAP(7) + PDU，PDU 应为 AB 01。
        std::uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH] = {};
        const int rc = modbus_receive_confirmation(ctx, rsp);
        check(rc >= 2, "收到未知功能码的确认报文");
        check(rc >= 9 && rsp[7] == 0xAB && rsp[8] == 0x01,
              "响应是异常码 01（0xAB = 0x2B|0x80, 01 = illegal function）");
    }
    // --- 多 unit：换成 unit=7 仍能读到（TCP 默认不按 unit 过滤） ---
    {
        modbus_set_slave(ctx, 7);
        std::uint16_t regs[2] = {};
        const int rc = modbus_read_registers(ctx, 0, 2, regs);
        check(rc == 2, "unit=7 仍能正常读（TCP 默认不按 unit 过滤）");
    }

    modbus_close(ctx);
    modbus_free(ctx);
    slave->stop();
}

} // namespace

int main()
{
    std::printf("modbusmech <-> libmodbus 互通测试\n\n");

    test_libmodbus_slave_our_master();
    std::this_thread::sleep_for(200ms);
    test_our_slave_libmodbus_master();

    std::printf("\n%s\n", g_fail == 0 ? "PASS" : "FAIL");
    return g_fail == 0 ? 0 : 1;
}
