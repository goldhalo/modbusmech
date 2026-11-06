// =============================================================================
//  modbusmech_slave —— 从站示例程序（互测用）
// =============================================================================
//  用法：
//    modbusmech_slave tcp <port> [选项]
//    modbusmech_slave rtu <device> <baud> [选项]
//
//  选项：
//    --unit N               从站地址 / Unit ID（默认 1）
//    --units 1,2,3          本实例应答的**全部** unit（一个串口/端口模拟多台设备）
//    --regs N               每个寄存器区的大小（默认 1000）
//    --idle SEC             TCP 空闲连接超时（默认 0 = 不断开）
//    --max-clients N        TCP 最大客户端数（默认 0 = 不限）
//    --silence MS           RTU 帧间静默（默认按波特率算）
//    --boundary silence|length  RTU 帧边界判定方式（默认 silence）
//    --parity N|E|O         RTU 校验位（默认 N）
//    --databits N           RTU 数据位（默认 8）
//    --stopbits N           RTU 停止位（默认 1）
//    --quiet                不打印每个请求（只打印启动信息和异常）
//
//  寄存器初值（便于和第三方主站对表）：
//    holding[i]  = i
//    input[i]    = 1000 + i
//    coils[i]    = (i % 4 == 0)
//    discrete[i] = (i % 5 == 0)
//
//  例：
//    modbusmech_slave tcp 502
//    modbusmech_slave tcp 1502 --units 1,2 --regs 100
//    modbusmech_slave rtu COM3 9600 --parity E --boundary length
//    modbusmech_slave rtu /dev/ttyUSB0 9600 --silence 5
// =============================================================================
#include <modbusmech/slave.hpp>

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <set>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

using namespace modbusmech;

namespace {

using namespace std::chrono_literals;

/// 演示用的寄存器组：四个区，全部在内存里。
///
/// 注意每个回调的第一个参数是 `unit` —— 多设备/透传场景靠它区分是哪台设备在问，
/// 单设备时忽略即可。
class DemoBank final : public RegisterBank {
public:
    explicit DemoBank(std::size_t n, bool quiet)
        : m_coils(n, false), m_discrete(n, false),
          m_holding(n, 0), m_input(n, 0), m_quiet(quiet)
    {
        for (std::size_t i = 0; i < n; ++i) {
            m_holding[i] = static_cast<std::uint16_t>(i);
            m_input[i]   = static_cast<std::uint16_t>(1000 + i);
            m_coils[i]    = (i % 4 == 0);
            m_discrete[i] = (i % 5 == 0);
        }
    }

    bool read_coils(std::uint8_t unit, std::uint16_t addr, std::uint16_t cnt,
                    std::vector<bool>& out) override
    {
        log_read(unit, 0x01, addr, cnt);
        if (static_cast<std::size_t>(addr) + cnt > m_coils.size()) {
            return false;   // → 回异常码 02
        }
        out.assign(m_coils.begin() + addr, m_coils.begin() + addr + cnt);
        return true;
    }

    bool read_discrete_inputs(std::uint8_t unit, std::uint16_t addr, std::uint16_t cnt,
                              std::vector<bool>& out) override
    {
        log_read(unit, 0x02, addr, cnt);
        if (static_cast<std::size_t>(addr) + cnt > m_discrete.size()) {
            return false;
        }
        out.assign(m_discrete.begin() + addr, m_discrete.begin() + addr + cnt);
        return true;
    }

    bool read_holding_registers(std::uint8_t unit, std::uint16_t addr, std::uint16_t cnt,
                                std::vector<std::uint16_t>& out) override
    {
        log_read(unit, 0x03, addr, cnt);
        if (static_cast<std::size_t>(addr) + cnt > m_holding.size()) {
            return false;
        }
        out.assign(m_holding.begin() + addr, m_holding.begin() + addr + cnt);
        return true;
    }

    bool read_input_registers(std::uint8_t unit, std::uint16_t addr, std::uint16_t cnt,
                              std::vector<std::uint16_t>& out) override
    {
        log_read(unit, 0x04, addr, cnt);
        if (static_cast<std::size_t>(addr) + cnt > m_input.size()) {
            return false;
        }
        out.assign(m_input.begin() + addr, m_input.begin() + addr + cnt);
        return true;
    }

    bool write_single_coil(std::uint8_t unit, std::uint16_t addr, bool value) override
    {
        log_write(unit, 0x05, addr, value ? 1u : 0u);
        if (addr >= m_coils.size()) {
            return false;
        }
        m_coils[addr] = value;
        return true;
    }

    bool write_single_register(std::uint8_t unit, std::uint16_t addr,
                               std::uint16_t value) override
    {
        log_write(unit, 0x06, addr, value);
        if (addr >= m_holding.size()) {
            return false;
        }
        m_holding[addr] = value;
        return true;
    }

    bool write_multiple_coils(std::uint8_t unit, std::uint16_t addr,
                              const std::vector<bool>& values) override
    {
        log_write(unit, 0x0F, addr, static_cast<std::uint32_t>(values.size()));
        if (static_cast<std::size_t>(addr) + values.size() > m_coils.size()) {
            return false;
        }
        for (std::size_t i = 0; i < values.size(); ++i) {
            m_coils[addr + i] = values[i];
        }
        return true;
    }

    bool write_multiple_registers(std::uint8_t unit, std::uint16_t addr,
                                  const std::vector<std::uint16_t>& values) override
    {
        log_write(unit, 0x10, addr, static_cast<std::uint32_t>(values.size()));
        if (static_cast<std::size_t>(addr) + values.size() > m_holding.size()) {
            return false;
        }
        for (std::size_t i = 0; i < values.size(); ++i) {
            m_holding[addr + i] = values[i];
        }
        return true;
    }

private:
    void log_read(std::uint8_t unit, std::uint8_t fc, std::uint16_t addr,
                  std::uint16_t cnt) const
    {
        if (m_quiet) {
            return;
        }
        std::printf("[req ] unit=%u fc=0x%02X addr=0x%04X cnt=%u\n",
                    static_cast<unsigned>(unit), static_cast<unsigned>(fc),
                    static_cast<unsigned>(addr), static_cast<unsigned>(cnt));
    }

    void log_write(std::uint8_t unit, std::uint8_t fc, std::uint16_t addr,
                   std::uint32_t v) const
    {
        if (m_quiet) {
            return;
        }
        std::printf("[wr  ] unit=%u fc=0x%02X addr=0x%04X value/cnt=%u\n",
                    static_cast<unsigned>(unit), static_cast<unsigned>(fc),
                    static_cast<unsigned>(addr), static_cast<unsigned>(v));
    }

    std::vector<bool>          m_coils;
    std::vector<bool>          m_discrete;
    std::vector<std::uint16_t> m_holding;
    std::vector<std::uint16_t> m_input;
    bool                       m_quiet = false;
};

void usage()
{
    std::printf(
        "用法:\n"
        "  modbusmech_slave tcp <port> [选项]\n"
        "  modbusmech_slave rtu <device> <baud> [选项]\n"
        "\n选项:\n"
        "  --unit N              从站地址 / Unit ID（默认 1）\n"
        "  --units 1,2,3         本实例应答的全部 unit\n"
        "  --regs N              每个寄存器区大小（默认 1000）\n"
        "  --idle SEC            TCP 空闲连接超时（默认 0）\n"
        "  --max-clients N       TCP 最大客户端数（默认 0）\n"
        "  --silence MS          RTU 帧间静默（默认按波特率算）\n"
        "  --boundary silence|length  RTU 帧边界（默认 silence）\n"
        "  --parity N|E|O        RTU 校验位（默认 N）\n"
        "  --databits N          RTU 数据位（默认 8）\n"
        "  --stopbits N          RTU 停止位（默认 1）\n"
        "  --quiet               不打印每个请求\n");
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

bool parse_units(const char* s, std::set<std::uint8_t>& out)
{
    std::string       str = s;
    std::size_t       pos = 0;
    while (pos <= str.size()) {
        const std::size_t comma = str.find(',', pos);
        const std::string part = str.substr(pos, comma == std::string::npos
                                                   ? std::string::npos
                                                   : comma - pos);
        if (part.empty()) {
            return false;
        }
        std::uint32_t v = 0;
        if (!parse_u32(part.c_str(), v) || v > 247) {
            return false;
        }
        out.insert(static_cast<std::uint8_t>(v));
        if (comma == std::string::npos) {
            break;
        }
        pos = comma + 1;
    }
    return !out.empty();
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        usage();
        return 2;
    }

    bool                      tcp = true;
    std::string               host;
    std::uint16_t             port = 502;
    std::string               device;
    std::uint32_t             baud = 9600;

    Slave::Config             cfg;
    std::size_t               regs = 1000;
    bool                      quiet = false;

    int i = 1;
    const std::string transport = argv[i++];
    if (transport == "tcp") {
        tcp = true;
        std::uint32_t p = 0;
        if (i >= argc || !parse_u32(argv[i++], p) || p == 0 || p > 65535) {
            usage();
            return 2;
        }
        port = static_cast<std::uint16_t>(p);
    } else if (transport == "rtu") {
        tcp = false;
        if (i + 1 >= argc) {
            usage();
            return 2;
        }
        device = argv[i++];
        std::uint32_t b = 0;
        if (!parse_u32(argv[i++], b) || b == 0) {
            usage();
            return 2;
        }
        baud = b;
    } else {
        usage();
        return 2;
    }

    while (i < argc) {
        const std::string opt = argv[i++];
        if (opt == "--quiet") {
            quiet = true;
        } else if (opt == "--unit" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v) || v > 247) { return 2; }
            cfg.address = static_cast<std::uint8_t>(v);
        } else if (opt == "--units" && i < argc) {
            if (!parse_units(argv[i++], cfg.addresses)) {
                std::printf("非法 --units\n");
                return 2;
            }
        } else if (opt == "--regs" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v) || v == 0 || v > 65535) { return 2; }
            regs = v;
        } else if (opt == "--idle" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            cfg.idle_timeout = std::chrono::seconds(v);
        } else if (opt == "--max-clients" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            cfg.max_clients = v;
        } else if (opt == "--silence" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            cfg.silence_ms = v;
        } else if (opt == "--boundary" && i < argc) {
            const std::string b = argv[i++];
            if (b == "length") {
                cfg.boundary = Slave::Config::FrameBoundary::length;
            } else if (b == "silence") {
                cfg.boundary = Slave::Config::FrameBoundary::silence;
            } else {
                std::printf("非法 --boundary: %s\n", b.c_str());
                return 2;
            }
        } else if (opt == "--databits" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            cfg.data_bits = static_cast<std::uint8_t>(v);
        } else if (opt == "--stopbits" && i < argc) {
            std::uint32_t v = 0;
            if (!parse_u32(argv[i++], v)) { return 2; }
            cfg.stop_bits = v == 2 ? SerialStopBits::two : SerialStopBits::one;
        } else if (opt == "--parity" && i < argc) {
            const std::string p = argv[i++];
            if (p == "N" || p == "n" || p == "none") {
                cfg.parity = SerialParity::none;
            } else if (p == "E" || p == "e" || p == "even") {
                cfg.parity = SerialParity::even;
            } else if (p == "O" || p == "o" || p == "odd") {
                cfg.parity = SerialParity::odd;
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

    auto bank = std::make_shared<DemoBank>(regs, quiet);

    std::error_code ec;
    std::shared_ptr<Slave> slave;
    if (tcp) {
        slave = Slave::create_tcp(port, cfg, bank, ec);
        if (!slave) {
            std::printf("创建 TCP 从站失败: %s\n", ec.message().c_str());
            return 1;
        }
        std::printf("TCP 从站已启动: 0.0.0.0:%u  unit=%u  regs=%zu\n",
                    static_cast<unsigned>(port), static_cast<unsigned>(cfg.address), regs);
    } else {
        slave = Slave::create_rtu(device, baud, cfg, bank, ec);
        if (!slave) {
            std::printf("创建 RTU 从站失败: %s\n", ec.message().c_str());
            return 1;
        }
        std::printf("RTU 从站已启动: %s @ %u  unit=%u  regs=%zu\n",
                    device.c_str(), static_cast<unsigned>(baud),
                    static_cast<unsigned>(cfg.address), regs);
    }

    std::printf("初值: holding[i]=i, input[i]=1000+i, coils[i]=(i%%4==0), "
                "discrete[i]=(i%%5==0)\n");
    std::printf("按 Ctrl+C 退出（退出前打印统计）\n");

    // 主循环：等 Ctrl+C。真正的请求处理都在 netmech 的 io 线程上。
    for (;;) {
        std::this_thread::sleep_for(1s);
        if (!slave->is_running()) {
            std::printf("从站已停止\n");
            break;
        }
    }

    const auto st = slave->stats();
    std::printf("\n统计: requests=%llu exceptions=%llu bad_frames=%llu "
                "rejected_clients=%llu send_failures=%llu\n",
                static_cast<unsigned long long>(st.requests),
                static_cast<unsigned long long>(st.exceptions),
                static_cast<unsigned long long>(st.bad_frames),
                static_cast<unsigned long long>(st.rejected_clients),
                static_cast<unsigned long long>(st.send_failures));
    slave->stop();
    return 0;
}
