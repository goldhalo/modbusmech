// =============================================================================
//  modbusmech_master —— 主站示例程序
// =============================================================================
//  形态参考 D:\ai-work\0910\ModbusClient.hpp（RTU/TCP 两种传输 + 读写包装 +
//  异步/同步两条路），底层换成 modbusmech。
//
//  用法：
//    modbusmech_master tcp <host> <port> [选项] <命令> [参数...]
//    modbusmech_master rtu <device> <baud> [选项] <命令> [参数...]
//
//  选项（可放在传输参数之后、命令之前）：
//    --unit N              从站地址 / Unit ID（默认 1）
//    --timeout MS          单次请求超时（默认 1000）
//    --retries N           传输失败重试次数（默认 0）
//    --connect-timeout MS  建连超时（默认 3000，TCP）
//    --auto-reconnect      连接断了之后按需重连（默认关）
//    --inflight N          TCP 在途请求上限（默认 1；>1 走流水线）
//    --parity N|E|O        RTU 校验位（默认 N）
//    --databits N          RTU 数据位（默认 8）
//    --stopbits N          RTU 停止位（默认 1）
//    --async               用异步主站（回调 + 内置超时）而不是阻塞调用
//
//  命令：
//    rh  <addr> <count>              读保持寄存器（03）
//    ri  <addr> <count>              读输入寄存器（04）
//    rc  <addr> <count>              读线圈（01）
//    rd  <addr> <count>              读离散输入（02）
//    wr  <addr> <value>              写单个寄存器（06）
//    wrs <addr> <v1> [v2 ...]        写多个寄存器（10）
//    wc  <addr> <0|1>                写单个线圈（05）
//    wcs <addr> <0|1> [0|1 ...]      写多个线圈（0F）
//    poll <addr> <count> <times> [interval_ms]
//                                    连续读保持寄存器（互测用：观察丢包/超时）
//
//  例：
//    modbusmech_master tcp 127.0.0.1 502 --unit 1 rh 0 10
//    modbusmech_master tcp 127.0.0.1 502 --inflight 4 poll 0 10 100 100
//    modbusmech_master rtu COM3 9600 --parity E rh 0 10
//    modbusmech_master tcp 127.0.0.1 502 --async rh 0 10
// =============================================================================
#include <modbusmech/async_master.hpp>
#include <modbusmech/master.hpp>

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

using namespace modbusmech;

namespace {

using namespace std::chrono_literals;

struct Options {
    bool                     tcp = true;
    std::string              host;
    std::uint16_t            port = 502;
    std::string              device;
    std::uint32_t            baud = 9600;

    std::uint8_t             unit = 1;
    std::chrono::milliseconds timeout{1000};
    std::chrono::milliseconds connect_timeout{3000};
    int                      retries = 0;
    bool                     auto_reconnect = false;
    std::uint32_t            in_flight = 1;
    SerialParity             parity = SerialParity::none;
    std::uint8_t             data_bits = 8;
    SerialStopBits           stop_bits = SerialStopBits::one;
    bool                     async = false;
};

void usage()
{
    std::printf(
        "用法:\n"
        "  modbusmech_master tcp <host> <port> [选项] <命令> [参数...]\n"
        "  modbusmech_master rtu <device> <baud> [选项] <命令> [参数...]\n"
        "\n选项:\n"
        "  --unit N              从站地址 / Unit ID（默认 1）\n"
        "  --timeout MS          单次请求超时（默认 1000）\n"
        "  --retries N           传输失败重试次数（默认 0）\n"
        "  --connect-timeout MS  建连超时（默认 3000）\n"
        "  --auto-reconnect      断链后按需重连（默认关）\n"
        "  --inflight N          TCP 在途上限（默认 1）\n"
        "  --parity N|E|O        RTU 校验位（默认 N）\n"
        "  --databits N          RTU 数据位（默认 8）\n"
        "  --stopbits N          RTU 停止位（默认 1）\n"
        "  --async               用异步主站（回调 + 内置超时）\n"
        "\n命令:\n"
        "  rh  <addr> <count>            读保持寄存器(03)\n"
        "  ri  <addr> <count>            读输入寄存器(04)\n"
        "  rc  <addr> <count>            读线圈(01)\n"
        "  rd  <addr> <count>            读离散输入(02)\n"
        "  wr  <addr> <value>            写单寄存器(06)\n"
        "  wrs <addr> <v1> [v2 ...]      写多寄存器(10)\n"
        "  wc  <addr> <0|1>              写单线圈(05)\n"
        "  wcs <addr> <0|1> [0|1 ...]    写多线圈(0F)\n"
        "  poll <addr> <count> <times> [interval_ms]  连续读(03)\n");
}

bool parse_u32(const char* s, std::uint32_t& out)
{
    if (s == nullptr || *s == '\0') {
        return false;
    }
    char* end = nullptr;
    const unsigned long v = std::strtoul(s, &end, 0);
    if (end == nullptr || *end != '\0') {
        return false;
    }
    out = static_cast<std::uint32_t>(v);
    return true;
}

/// 打印一次请求的结果。
void print_result(const char* what, const Result& r,
                  const std::vector<std::uint16_t>& regs,
                  const std::vector<bool>& bits)
{
    if (!r.ok()) {
        std::printf("%s: FAIL  %s\n", what, r.describe().c_str());
        return;
    }
    std::printf("%s: OK   ", what);
    if (!regs.empty()) {
        for (std::size_t i = 0; i < regs.size(); ++i) {
            std::printf("%s0x%04X", i == 0 ? "" : " ", static_cast<unsigned>(regs[i]));
        }
        std::printf("\n");
        return;
    }
    if (!bits.empty()) {
        for (std::size_t i = 0; i < bits.size(); ++i) {
            std::printf("%s%d", i == 0 ? "" : " ", bits[i] ? 1 : 0);
        }
        std::printf("\n");
        return;
    }
    std::printf("(done)\n");
}

// ---------------------------------------------------------------------------
//  阻塞主站
// ---------------------------------------------------------------------------

std::shared_ptr<Master> make_master(const Options& o, std::error_code& ec)
{
    Master::Config cfg;
    cfg.unit            = o.unit;
    cfg.timeout         = o.timeout;
    cfg.retries         = o.retries;
    cfg.connect_timeout = o.connect_timeout;
    cfg.auto_reconnect  = o.auto_reconnect;
    cfg.max_in_flight   = o.in_flight;
    cfg.parity          = o.parity;
    cfg.data_bits       = o.data_bits;
    cfg.stop_bits       = o.stop_bits;

    if (o.tcp) {
        return Master::create_tcp(o.host, o.port, cfg, ec);
    }
    return Master::create_rtu(o.device, cfg, o.baud, ec);
}

// ---------------------------------------------------------------------------
//  异步主站：把回调结果搬回主线程打印
// ---------------------------------------------------------------------------

struct AsyncWait {
    std::mutex                mtx;
    std::condition_variable   cv;
    bool                      done = false;
    Result                    result;
    std::vector<std::uint16_t> regs;
    std::vector<bool>          bits;
};

void wait_ready(AsyncWait& w, std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lk(w.mtx);
    w.cv.wait_for(lk, timeout, [&] { return w.done; });
}

std::shared_ptr<AsyncMaster> make_async(const Options& o, std::error_code& ec)
{
    AsyncMaster::Config cfg;
    cfg.unit            = o.unit;
    cfg.timeout         = o.timeout;
    cfg.max_workers     = o.in_flight == 0 ? 1 : o.in_flight;
    cfg.auto_reconnect  = o.auto_reconnect;
    cfg.connect_timeout = o.connect_timeout;
    cfg.retries         = o.retries;
    if (!o.tcp) {
        ec = std::make_error_code(std::errc::operation_not_supported);
        return nullptr;   // 异步主站目前只做 TCP
    }
    return AsyncMaster::create_tcp(o.host, o.port, cfg, ec);
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        usage();
        return 2;
    }

    Options o;

    // ---- 传输参数 ----
    int i = 1;
    const std::string transport = argv[i++];
    if (transport == "tcp") {
        o.tcp = true;
        if (i + 1 >= argc) { usage(); return 2; }
        o.host = argv[i++];
        std::uint32_t p = 0;
        if (!parse_u32(argv[i++], p) || p == 0 || p > 65535) { usage(); return 2; }
        o.port = static_cast<std::uint16_t>(p);
    } else if (transport == "rtu") {
        o.tcp = false;
        if (i + 1 >= argc) { usage(); return 2; }
        o.device = argv[i++];
        std::uint32_t b = 0;
        if (!parse_u32(argv[i++], b) || b == 0) { usage(); return 2; }
        o.baud = b;
    } else {
        usage();
        return 2;
    }

    // ---- 选项 ----
    while (i < argc && std::strncmp(argv[i], "--", 2) == 0) {
        const std::string opt = argv[i++];
        if (opt == "--async") {
            o.async = true;
        } else if (opt == "--auto-reconnect") {
            o.auto_reconnect = true;
        } else if (opt == "--unit" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v) || v > 255) { return 2; }
            o.unit = static_cast<std::uint8_t>(v);
        } else if (opt == "--timeout" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            o.timeout = std::chrono::milliseconds(v);
        } else if (opt == "--connect-timeout" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            o.connect_timeout = std::chrono::milliseconds(v);
        } else if (opt == "--retries" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            o.retries = static_cast<int>(v);
        } else if (opt == "--inflight" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            o.in_flight = v == 0 ? 1 : v;
        } else if (opt == "--databits" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            o.data_bits = static_cast<std::uint8_t>(v);
        } else if (opt == "--stopbits" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            o.stop_bits = v == 2 ? SerialStopBits::two : SerialStopBits::one;
        } else if (opt == "--parity" && i < argc) {
            const std::string p = argv[i++];
            if (p == "N" || p == "n" || p == "none") {
                o.parity = SerialParity::none;
            } else if (p == "E" || p == "e" || p == "even") {
                o.parity = SerialParity::even;
            } else if (p == "O" || p == "o" || p == "odd") {
                o.parity = SerialParity::odd;
            } else {
                std::printf("非法 --parity: %s\n", p.c_str());
                return 2;
            }
        } else {
            std::printf("未知选项: %s\n", opt.c_str());
            usage();
            return 2;
        }
    }

    if (i >= argc) {
        usage();
        return 2;
    }
    const std::string cmd = argv[i++];

    // ---- 建连 ----
    std::error_code ec;
    std::shared_ptr<Master>      master;
    std::shared_ptr<AsyncMaster> async_master;
    if (o.async) {
        async_master = make_async(o, ec);
        if (!async_master) {
            std::printf("创建异步主站失败: %s\n", ec.message().c_str());
            return 1;
        }
    } else {
        master = make_master(o, ec);
        if (!master) {
            std::printf("创建主站失败: %s\n", ec.message().c_str());
            return 1;
        }
    }

    // ---- 同步命令 ----
    auto need = [&](int n) { return (argc - i) >= n; };

    auto run_sync = [&](auto&& fn) {
        std::vector<std::uint16_t> regs;
        std::vector<bool>          bits;
        const Result r = fn(regs, bits);
        print_result(cmd.c_str(), r, regs, bits);
        return r.ok() ? 0 : 1;
    };

    if (cmd == "poll") {
        if (!need(3)) { usage(); return 2; }
        const auto addr  = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        const auto count = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        const int  times = std::atoi(argv[i++]);
        std::uint32_t interval = 100;
        if (i < argc) { interval = static_cast<std::uint32_t>(std::strtoul(argv[i++], nullptr, 0)); }

        int ok = 0, fail = 0;
        const auto t0 = std::chrono::steady_clock::now();
        for (int k = 0; k < times; ++k) {
            std::vector<std::uint16_t> regs;
            const Result r = master->read_holding_registers(addr, count, regs);
            if (r.ok()) {
                ++ok;
                std::printf("[%3d] 0x%04X x%d 首值=0x%04X\n", k, addr, count,
                            regs.empty() ? 0u : static_cast<unsigned>(regs[0]));
            } else {
                ++fail;
                std::printf("[%3d] 失败: %s\n", k, r.describe().c_str());
            }
            if (interval > 0 && k + 1 < times) {
                std::this_thread::sleep_for(std::chrono::milliseconds(interval));
            }
        }
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - t0).count();
        const auto st = master->stats();
        std::printf("完成: %d 成功 / %d 失败 / 用时 %lld ms（%.1f 次/秒）\n"
                    "统计: requests=%llu succeeded=%llu timeouts=%llu crc_errors=%llu "
                    "send_errors=%llu exceptions=%llu reconnects=%llu\n",
                    ok, fail, static_cast<long long>(ms),
                    ms > 0 ? (ok * 1000.0 / static_cast<double>(ms)) : 0.0,
                    static_cast<unsigned long long>(st.requests),
                    static_cast<unsigned long long>(st.succeeded),
                    static_cast<unsigned long long>(st.timeouts),
                    static_cast<unsigned long long>(st.crc_errors),
                    static_cast<unsigned long long>(st.send_errors),
                    static_cast<unsigned long long>(st.exceptions),
                    static_cast<unsigned long long>(st.reconnects));
        master->close();
        return fail == 0 ? 0 : 1;
    }

    if (!o.async) {
        if (cmd == "rh" && need(2)) {
            const auto addr  = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            const auto count = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            return run_sync([&](auto& regs, auto&) {
                return master->read_holding_registers(addr, count, regs);
            });
        }
        if (cmd == "ri" && need(2)) {
            const auto addr  = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            const auto count = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            return run_sync([&](auto& regs, auto&) {
                return master->read_input_registers(addr, count, regs);
            });
        }
        if (cmd == "rc" && need(2)) {
            const auto addr  = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            const auto count = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            return run_sync([&](auto&, auto& bits) {
                return master->read_coils(addr, count, bits);
            });
        }
        if (cmd == "rd" && need(2)) {
            const auto addr  = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            const auto count = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            return run_sync([&](auto&, auto& bits) {
                return master->read_discrete_inputs(addr, count, bits);
            });
        }
        if (cmd == "wr" && need(2)) {
            const auto addr = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            const auto v    = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            return run_sync([&](auto&, auto&) { return master->write_single_register(addr, v); });
        }
        if (cmd == "wrs" && need(2)) {
            const auto addr = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            std::vector<std::uint16_t> vals;
            while (i < argc) {
                vals.push_back(static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0)));
            }
            return run_sync([&](auto&, auto&) {
                return master->write_multiple_registers(addr, vals);
            });
        }
        if (cmd == "wc" && need(2)) {
            const auto addr = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            const bool on   = std::atoi(argv[i++]) != 0;
            return run_sync([&](auto&, auto&) { return master->write_single_coil(addr, on); });
        }
        if (cmd == "wcs" && need(2)) {
            const auto addr = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
            std::vector<bool> vals;
            while (i < argc) {
                vals.push_back(std::atoi(argv[i++]) != 0);
            }
            return run_sync([&](auto&, auto&) {
                return master->write_multiple_coils(addr, vals);
            });
        }
        std::printf("未知命令: %s\n", cmd.c_str());
        usage();
        return 2;
    }

    // ---- 异步命令（演示回调 + 内置超时）----
    AsyncWait w;
    auto on_regs = [&w](Result r, std::vector<std::uint16_t> regs) {
        std::lock_guard<std::mutex> lk(w.mtx);
        w.done   = true;
        w.result = r;
        w.regs   = std::move(regs);
        w.cv.notify_all();
    };
    auto on_bits = [&w](Result r, std::vector<bool> bits) {
        std::lock_guard<std::mutex> lk(w.mtx);
        w.done   = true;
        w.result = r;
        w.bits   = std::move(bits);
        w.cv.notify_all();
    };
    auto on_done = [&w](Result r) {
        std::lock_guard<std::mutex> lk(w.mtx);
        w.done   = true;
        w.result = r;
        w.cv.notify_all();
    };

    const std::uint8_t unit = o.unit;
    if (cmd == "rh" && need(2)) {
        const auto addr  = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        const auto count = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        async_master->read_holding_registers(unit, addr, count, on_regs);
    } else if (cmd == "ri" && need(2)) {
        const auto addr  = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        const auto count = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        async_master->read_input_registers(unit, addr, count, on_regs);
    } else if (cmd == "rc" && need(2)) {
        const auto addr  = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        const auto count = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        async_master->read_coils(unit, addr, count, on_bits);
    } else if (cmd == "rd" && need(2)) {
        const auto addr  = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        const auto count = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        async_master->read_discrete_inputs(unit, addr, count, on_bits);
    } else if (cmd == "wr" && need(2)) {
        const auto addr = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        const auto v    = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        async_master->write_single_register(unit, addr, v, on_done);
    } else if (cmd == "wrs" && need(2)) {
        const auto addr = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        std::vector<std::uint16_t> vals;
        while (i < argc) {
            vals.push_back(static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0)));
        }
        async_master->write_multiple_registers(unit, addr, vals, on_done);
    } else if (cmd == "wc" && need(2)) {
        const auto addr = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        const bool on   = std::atoi(argv[i++]) != 0;
        async_master->write_single_coil(unit, addr, on, on_done);
    } else if (cmd == "wcs" && need(2)) {
        const auto addr = static_cast<std::uint16_t>(std::strtoul(argv[i++], nullptr, 0));
        std::vector<bool> vals;
        while (i < argc) {
            vals.push_back(std::atoi(argv[i++]) != 0);
        }
        async_master->write_multiple_coils(unit, addr, vals, on_done);
    } else {
        std::printf("异步模式不支持该命令: %s\n", cmd.c_str());
        usage();
        return 2;
    }

    // 回调由 AsyncMaster 负责超时：这里多等 1s 纯粹是打印用。
    wait_ready(w, o.timeout + std::chrono::seconds(1));
    print_result(cmd.c_str(), w.result, w.regs, w.bits);
    async_master->close();
    return w.result.ok() ? 0 : 1;
}
