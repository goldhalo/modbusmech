# modbusmech

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)

> Modbus RTU / TCP 的主站与从站，网络层用 [netmech](../netmech)。

---

## 定位

| 做 | 不做 |
|---|---|
| PDU 编解码（8 个功能码） | 定时轮询、批量合并（应用的事） |
| RTU（串口）、TCP 两种传输 | 设备模板、点位映射（应用的事） |
| 主站（同步请求）和从站（寄存器回调） | 认证、加密（Modbus 本身没有） |
| CRC16 / MBAP / 帧校验 | 网关路由 |

---

## 30 秒上手

### 主站

```cpp
#include <modbusmech/master.hpp>

modbusmech::Master::Config cfg;
cfg.unit    = 1;                          // 从站地址
cfg.timeout = std::chrono::milliseconds{500};
cfg.retries = 1;

std::error_code ec;
auto master = modbusmech::Master::create_rtu("COM5", cfg, 9600, ec);

std::vector<std::uint16_t> regs;
auto r = master->read_holding_registers(0x0000, 10, regs);
if (r.ok()) {
    // regs 里是读到的 10 个寄存器
} else if (r.exception != modbusmech::Exception::none) {
    // 从站拒绝了：地址越界 / 功能码不支持 …
} else {
    // 传输失败：超时 / 串口错误
}
```

### 从站

```cpp
#include <modbusmech/slave.hpp>

class MyBank : public modbusmech::RegisterBank {
    std::vector<std::uint16_t> m_regs = std::vector<std::uint16_t>(1000, 0);

    bool read_holding_registers(std::uint16_t addr, std::uint16_t cnt,
                                std::vector<std::uint16_t>& out) override {
        if (addr + cnt > m_regs.size()) return false;   // → 自动回异常码 02
        out.assign(m_regs.begin() + addr, m_regs.begin() + addr + cnt);
        return true;
    }
    // 其它 7 个方法同理
};

modbusmech::Slave::Config cfg;
cfg.address = 1;
std::error_code ec;
auto slave = modbusmech::Slave::create_rtu("COM3", 9600, cfg,
                                           std::make_shared<MyBank>(), ec);
```

---

## 三个必须知道的点

### ① `Result` 区分"传输失败"和"从站拒绝"

```cpp
struct Result {
    std::error_code ec;              // 传输失败：超时、串口错误、连接断了
    Exception       exception;       // 从站拒绝：地址越界、功能码不支持
    bool ok() const;
    bool retryable() const;          // 传输失败 / 从站忙 → 可重试
};
```

**这个区分很重要**：传输失败可以盲目重试；协议异常重试也没用（除了
`server_device_busy`）。libmodbus 把两者混在 `errno` 里，判断起来别扭。

### ② 寄存器由应用提供，库不持有数据

不用 `modbus_mapping_t` 那种静态数组，而是让应用实现 `RegisterBank`：

- 应用的数据**不用同步两份**
- 写操作能被应用感知（可以触发副作用，比如真的去操作继电器）

### ③ 主站 API 是**阻塞**的

串口本质是"一问一答"，阻塞语义最自然。TCP 想并发就开多个 `Master` 实例。

---

## 为什么不用 libmodbus

| 项 | libmodbus | modbusmech |
|---|---|---|
| **许可证** | LGPL-2.1+ | **MIT** |
| 语言 | C | C++17 |
| 网络层 | 自己写（`select` + `read/write`） | **netmech**（TCP/串口共用 io_context） |
| 寄存器 | `modbus_mapping_t`（静态数组） | **`RegisterBank` 回调** |
| 帧边界 | `select` 超时近似 | **显式配置**（silence / length） |
| TCP 多客户端 | 不支持 | **支持** |
| 代码来源 | — | **参照规范独立实现**，不复制 |

本库实现依据 **Modbus Application Protocol V1.1b3**（公开规范），
行为用 libmodbus 做对照测试验证，但**不包含任何 libmodbus 代码**。

---

## 协议限制（规范规定）

| 操作 | 单次上限 |
|---|---|
| 读线圈 / 离散输入 | 2000 个 |
| 读保持 / 输入寄存器 | 125 个 |
| 写多个线圈 | 1968 个 |
| 写多个寄存器 | 123 个 |

超限会被 `encode_pdu` 拒绝（返回 `invalid_argument`）。

---

## 构建

```bash
cmake -S . -B build -DMODBUSMECH_NETMECH_DIR=/path/to/netmech
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

依赖：C++17、CMake ≥ 3.16、[netmech](../netmech)。

---

## 目录结构

```
include/modbusmech/   公共头（5 个）
  common.hpp          功能码 / 异常码 / Result / 错误码
  span.hpp            轻量 span
  pdu.hpp             PDU 编解码（纯函数，无 IO）
  master.hpp          主站
  slave.hpp           从站 + RegisterBank

src/
  pdu.cpp             编解码 + CRC16（纯函数）
  rtu.cpp             RTU 传输（串口 + 帧边界 + CRC）
  master.cpp          主站逻辑（组帧 / 超时 / 重试）

test/                 23 用例 / 93 断言
docs/design/          概要设计
```

---

## 致谢

本项目的实现与文档在 **DeepSeek-V4.1** 的协助下完成 —— 架构决策与验收由作者负责。
详见 [AUTHORS.md](AUTHORS.md)。

---

## License

MIT，见 [LICENSE](LICENSE)。
