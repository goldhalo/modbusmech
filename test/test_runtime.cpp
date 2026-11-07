// ============================================================================
//  test_runtime.cpp —— 注入 Runtime（多池 / 与其它协议库共享 io 线程）
//
//  验证：
//    * Master / Slave / AsyncMaster 的 create_* 支持注入 netmech::Runtime
//    * 注入后读写正常，且 Runtime::shutdown() 之后请求失败（不再自动重连/投递）
// ============================================================================
#include <modbusmech/async_master.hpp>
#include <modbusmech/master.hpp>
#include <modbusmech/slave.hpp>

#include <netmech/runtime.hpp>
#include <netmech/udp.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <system_error>
#include <thread>
#include <vector>

#include "test_framework.hpp"

using namespace modbusmech;
using namespace std::chrono_literals;

namespace {

/// 全 8 个方法的确定性 RegisterBank（读返回"地址"本身，便于断言）。
class TestBank final : public RegisterBank {
public:
    bool read_coils(std::uint8_t, std::uint16_t address, std::uint16_t count,
                    std::vector<bool>& out) override
    {
        out.assign(count, false);
        return true;
    }
    bool read_discrete_inputs(std::uint8_t, std::uint16_t address,
                              std::uint16_t count, std::vector<bool>& out) override
    {
        out.assign(count, false);
        return true;
    }
    bool read_holding_registers(std::uint8_t, std::uint16_t address,
                                std::uint16_t count,
                                std::vector<std::uint16_t>& out) override
    {
        out.resize(count);
        for (std::uint16_t i = 0; i < count; ++i) {
            out[i] = static_cast<std::uint16_t>(address + i);
        }
        return true;
    }
    bool read_input_registers(std::uint8_t, std::uint16_t address,
                              std::uint16_t count,
                              std::vector<std::uint16_t>& out) override
    {
        out.resize(count);
        for (std::uint16_t i = 0; i < count; ++i) {
            out[i] = static_cast<std::uint16_t>(address + i);
        }
        return true;
    }
    bool write_single_coil(std::uint8_t, std::uint16_t, bool) override { return true; }
    bool write_single_register(std::uint8_t, std::uint16_t, std::uint16_t) override { return true; }
    bool write_multiple_coils(std::uint8_t, std::uint16_t,
                              const std::vector<bool>&) override { return true; }
    bool write_multiple_registers(std::uint8_t, std::uint16_t,
                                  const std::vector<std::uint16_t>&) override { return true; }
};

} // namespace

NETMECH_TEST(modbusmech_uses_injected_runtime)
{
    // 应用自己定线程数：1 条 io 线程的独立池
    netmech::Runtime::Options opt;
    opt.io_threads = 1;
    auto rt = netmech::Runtime::create(opt);
    REQUIRE(rt != nullptr);
    CHECK_EQ(rt->io_threads(), std::size_t{1});
    CHECK(rt->running());

    const auto port_opt = netmech::find_free_port_run(46000, 47000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank = std::make_shared<TestBank>();
    std::error_code ec;
    auto slave = Slave::create_tcp(port, {}, bank, ec, rt);
    REQUIRE(slave != nullptr);
    CHECK(!ec);

    Master::Config mc;
    mc.timeout        = 1000ms;
    mc.retries        = 0;
    mc.auto_reconnect = false;
    auto master = Master::create_tcp("127.0.0.1", port, mc, ec, rt);
    REQUIRE(master != nullptr);

    std::vector<std::uint16_t> regs;
    const auto r = master->read_holding_registers(5, 2, regs);
    REQUIRE(r.ok());
    REQUIRE(regs.size() == 2u);
    CHECK_EQ(regs[0], std::uint16_t{5});
    CHECK_EQ(regs[1], std::uint16_t{6});

    // 先收对象，再停运行时（否则 accept 关闭会投递到已停的池、端口释放延迟）
    master->close();
    slave->stop();

    // 关闭注入的运行时：之后新建的对象无法工作（listen/connect 返回 not_initialized）
    rt->shutdown();
    CHECK(!rt->running());
    std::error_code ec2;
    auto slave_after = Slave::create_tcp(port, {}, bank, ec2, rt);
    CHECK(slave_after == nullptr);
}

NETMECH_TEST(async_master_uses_injected_runtime)
{
    netmech::Runtime::Options opt;
    opt.io_threads = 1;
    auto rt = netmech::Runtime::create(opt);
    REQUIRE(rt != nullptr);

    const auto port_opt = netmech::find_free_port_run(46000, 47000, 1, 1);
    REQUIRE(port_opt.has_value());
    const std::uint16_t port = static_cast<std::uint16_t>(*port_opt);

    auto bank = std::make_shared<TestBank>();
    std::error_code ec;
    auto slave = Slave::create_tcp(port, {}, bank, ec, rt);
    REQUIRE(slave != nullptr);

    AsyncMaster::Config ac;
    auto async = AsyncMaster::create_tcp("127.0.0.1", port, ac, ec, rt);
    REQUIRE(async != nullptr);

    std::atomic<int> done{0};
    std::atomic<int> ok{0};
    async->read_holding_registers(1, 0, 2,
                                  [&](Result r, std::vector<std::uint16_t> regs) {
                                      if (r.ok() && regs.size() == 2u) {
                                          ok.fetch_add(1);
                                      }
                                      done.fetch_add(1);
                                  });

    for (int i = 0; i < 300 && done.load() == 0; ++i) {
        std::this_thread::sleep_for(10ms);
    }
    CHECK_EQ(done.load(), 1);
    CHECK_EQ(ok.load(), 1);

    async->close();
    slave->stop();
    rt->shutdown();
}
