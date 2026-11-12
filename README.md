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

    // ⚠ 每个回调的第一个参数是 unit（RTU 的从站地址 / TCP 的 Unit ID）。
    //   正常单设备从站忽略它即可；做多设备或协议透传、需要知道"这是谁在问"时用它。
    bool read_holding_registers(std::uint8_t unit, std::uint16_t addr, std::uint16_t cnt,
                                std::vector<std::uint16_t>& out) override {
        (void)unit;
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

#### TCP 从站：连接事件回调

Modbus 协议本身**没有**认证。库只把"谁连进来了 / 谁断开了"告诉使用端，
**接受还是拒绝、白名单怎么判、在线列表怎么存，全由使用端决定**：
`Config::on_client_connected` 在 accept 时回调，返回 `false` 即拒收；
`Config::on_client_disconnected` 在 netmech `Linkable::on_linkdown()` 里回调
（库保证每个被接管的连接恰好一次、及时投递，不依赖 Conn 析构；被拒绝的
连接不调用）。两个回调带回同一个 `netmech::Conn::id()`，应用可用它维护
在线客户端列表。

```cpp
modbusmech::Slave::Config cfg;
cfg.address = 1;

// 白名单 + 在线列表（回调在 io 线程，应用自己加锁）
cfg.on_client_connected = [&](std::uint64_t id,
                              const std::string& ip, std::uint16_t port) {
    if (ip != "192.168.1.10" && ip != "192.168.1.11") return false;
    std::lock_guard<std::mutex> lk(mu);
    clients[id] = {ip, port};
    return true;
};
cfg.on_client_disconnected = [&](std::uint64_t id,
                                 const std::string&, std::uint16_t) {
    std::lock_guard<std::mutex> lk(mu);
    clients.erase(id);
};

auto slave = modbusmech::Slave::create_tcp(502, cfg, bank, ec);
// 被拒了多少条，看统计：slave->stats().rejected_clients
```

几点注意：

- **只看 IP/端口**：accept 时还没收到任何 Modbus 帧，所以拿不到 unit id；
  要按 unit 区分请用 `Config::addresses` / `bank_for`。
- **回调在 io 线程上跑**，不能阻塞（查库、读文件都会拖住同时刻的其它连接）。
- `on_client_connected` 抛异常会被吞掉并按**拒绝**处理；
  `on_client_disconnected` 在 io 线程调用，异常同样被吞掉。
- 不设置 ⇒ 接受所有连接；被拒的连接**不会**触发 disconnected。
- RTU 是点对点串口，这两个回调**只对 `create_tcp` 生效**。
- `create_tcp(0, …)` 可由系统分配端口，用 `listening_port()` 读回实际端口，
  避免"先探测空闲端口再 bind"的 TOCTOU 竞态（测试 / CI 推荐）。

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

## 现场常见配置

### 短连接设备（PLC 回完一个响应就断开）

```cpp
modbusmech::Master::Config cfg;
cfg.auto_reconnect = true;      // 默认 **false**：连接断了就报错
cfg.connect_timeout = std::chrono::milliseconds{3000};
```

默认（`false`）语义是"连接断了就报错"，让调用方立刻知道链路出问题。
打开 `auto_reconnect` 后：**请求发出前发现没有连接就先建连**，建连失败/超时
**直接返回失败**（不会在传输层里循环重试）。注意：短连接设备开着它也没用——
对端不发包照样会断，它只是把"每次请求都自己重连"这件事挪进库里。

### TCP 流水线（同一连接多事务在途）

```cpp
modbusmech::Master::Config cfg;
cfg.max_in_flight = 4;   // 默认 1（与 RTU 一样串行）
```

协议允许同一连接上多个请求同时在途（靠 MBAP 事务号匹配）。`max_in_flight > 1`
时**同一个 `Master` 可以被多个线程并发使用**，吞吐不再受"一问一答"限制；
超过上限的并发请求返回 `ModbusErrc::busy`。

⚠ 不是所有从站都支持流水线（很多 PLC/网关一次只处理一个请求），先拿真机验证。

### 异步主站（回调 + 内置超时）

不想让采集线程被阻塞、又不想自己写超时处理时用它：

```cpp
#include <modbusmech/async_master.hpp>

modbusmech::AsyncMaster::Config ac;
ac.timeout     = std::chrono::milliseconds{500};   // 超时后回调会带 timeout 触发
ac.max_workers = 4;                                // 并发在途请求数

std::error_code ec;
auto am = modbusmech::AsyncMaster::create_tcp("192.168.1.10", 502, ac, ec);

am->read_holding_registers(1, 0x0000, 10,
    [](modbusmech::Result r, std::vector<std::uint16_t> regs) {
        if (r.ok()) { /* 用 regs */ }
        else        { /* r.ec 里是 timeout / not_connected / ... */ }
    });
```

- **回调恰好触发一次**，超时由库负责，调用方不需要自己兜；
- 所有回调在**同一条派发线程**上串行执行（回调里不要阻塞）；
- 内部是 N 条工作线程驱动阻塞版 `Master`，同一连接上做流水线。

### 串口校验位（8E1 / 8O1）

Modbus 规范默认是 **8E1**，但很多设备是 8N1/8O1。默认值保持 8N1（与以前一致），
需要时显式配：

```cpp
modbusmech::Master::Config cfg;
cfg.data_bits = 8;
cfg.parity    = modbusmech::SerialParity::even;     // 8E1
cfg.stop_bits = modbusmech::SerialStopBits::one;
```

> **Windows RTU 帧边界提示**：silence 模式依赖高精度定时器，Windows 默认
> 分辨率约 15.6ms，低波特率（9600 的 3.5 字符 ≈ 4ms）下可能粘连/误切帧。
> Windows 上推荐从站用 `Config::boundary = FrameBoundary::length`，或显式
> 调 `silence_ms`；主站侧按功能码推长度收帧，不受此影响。

### TCP 从站的连接管理

```cpp
modbusmech::Slave::Config sc;
sc.idle_timeout = std::chrono::seconds(120);   // 空闲连接自动断开（0 = 不断）
sc.max_clients  = 8;                           // 连接数上限（0 = 不限）
```

被拒的连接计入 `slave->stats().rejected_clients`；响应写不出去（队列满/已断）计入
`send_failures` 并主动断开该连接。

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

### 示例程序

默认随构建产出两个可执行文件（`-DMODBUSMECH_BUILD_EXAMPLES=OFF` 可关）：

```bash
./build/bin/modbusmech_slave  tcp 1502                    # 从站：模拟一台设备
./build/bin/modbusmech_master tcp 127.0.0.1 1502 rh 0 10  # 主站：读 10 个保持寄存器
```

- `modbusmech_master`：`rh/ri/rc/rd/wr/wrs/wc/wcs/poll` 子命令；`--async`（回调 + 内置超时）、
  `--inflight N`（TCP 流水线）、`--unit`、`--parity E`（8E1）、`--auto-reconnect` 等选项。
- `modbusmech_slave`：`tcp/rtu` 两种传输；`--units`（一个实例模拟多台）、`--idle`/`--max-clients`
  （连接管理）、`--boundary silence|length`（RTU 帧边界）、`--regs`（区大小）。

两个示例互相当对端就能跑通全部功能；**和 libmodbus / mthings 的互测步骤见
[docs/interop.md](docs/interop.md)**（含寄存器初值表、判定标准与常见坑）。

---

## 目录结构

```
include/modbusmech/   公共头
  common.hpp          功能码 / 异常码 / Result / 错误码 / 串口参数
  span.hpp            轻量 span
  pdu.hpp             PDU 编解码（纯函数，无 IO）
  master.hpp          主站（阻塞）
  async_master.hpp    主站（异步回调 + 内置超时）
  slave.hpp           从站 + RegisterBank

src/
  pdu.cpp             编解码 + CRC16（纯函数）
  rtu.cpp             RTU 传输（串口 + 帧边界 + CRC）
  tcp.cpp             TCP 传输（MBAP + 事务号 + 流水线 + 按需重连）
  slave.cpp           从站（RTU 单会话 / TCP 多客户端）
  master.cpp          主站逻辑（组帧 / 超时 / 重试）
  async_master.cpp    异步主站（工作线程 + 派发线程）

example/              示例程序（master / slave）
test/                 单元测试
docs/design/          概要设计 / TCP 非阻塞评估
docs/interop.md       互测手册（libmodbus / mthings）
```

---

## 在其他项目里使用（find_package）

先装 netmech（本仓的子工程，`EXCLUDE_FROM_ALL` 不会跟着装），再装本仓：

```bash
# 1) netmech
cmake -S ../netmech -B ../netmech/build && cmake --build ../netmech/build -j
cmake --install ../netmech/build

# 2) modbusmech（默认前缀：Linux /usr/local，Windows C:/Program Files/modbusmech）
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --install build
```

然后在自己的工程里：

```cmake
find_package(modbusmech REQUIRED)
target_link_libraries(app PRIVATE modbusmech::modbusmech)
```

**不用再手写 `libmodbusmech.a + libnetmech.a + pthread` 那一串** ——
`modbusmechConfig.cmake` 里 `find_dependency(netmech)` 会把它找出来，
链接库、include 路径都由导入的 target 带着走。

安装前缀不在默认位置时给 `-DCMAKE_PREFIX_PATH=<前缀>`（多个前缀用分号隔开）。

---

## 致谢

本项目的实现与文档在 **DeepSeek-V4.1** 的协助下完成 —— 架构决策与验收由作者负责。
详见 [AUTHORS.md](AUTHORS.md)。

---

## License

MIT，见 [LICENSE](LICENSE)。

## 发布为动态库（`-DBUILD_SHARED_LIBS=ON`）

modbusmech 默认构建静态库；也可以构建动态库。要点：

1. **netmech 的形态随本库联动**：`BUILD_SHARED_LIBS=ON` 时 CMake 会强制
   `NETMECH_LIBRARY_TYPE=SHARED`（避免同进程多份 io 运行时副本）。因此
   **共享形态下 netmech 是独立的动态库，发布时必须随附 netmech 的
   二进制（.so/.dll）及其 MIT LICENSE**；静态形态下 netmech 才静态链入
   本库、对外不需 netmech 二进制（但仍需保留其 MIT 许可文本）。
2. **io 线程归属**：同进程其它模块若各自静态链接 netmech，会各有一套
   线程池（**跨副本混用同一个 io_context 会崩**）；多模块共用请让 netmech
   以共享库形式被共用（见下节）。
3. **符号卫生**：Linux/macOS 构建共享库时用 `-Wl,--exclude-libs,ALL` 隐藏
   静态链入的依赖符号（共享形态下 netmech 是独立 .so，不受此选项影响）。
   Windows 导出宏（`MODBUSMECH_API`，export.hpp）**已实现**：库内 dllexport、
   使用方 dllimport，CMake 自动接线 `MODBUSMECH_SHARED`/`MODBUSMECH_BUILDING_LIBRARY`。
4. **卸载（dlclose）前**必须 `close()` 全部对象并 `netmech::shutdown()`：
   io 线程运行在 `.so` 的代码里，卸载后线程再执行会崩。进程正常退出时有兜底
   （静态析构里 join 线程）。
5. netmech 必须以 PIC 编译（`NETMECH_PIC` 默认 ON）。

## 与多个协议库同进程部署（共享 netmech）

modbusmech 与 httpmech / rtspmech / rtpmech 同时在一个进程里时，推荐
**netmech 用共享库**、各协议库也用共享库并链接共享 netmech：

```cmake
# 所有库都这样配置（以 modbusmech 为例）
cmake -S modbusmech -B build -DBUILD_SHARED_LIBS=ON -DNETMECH_LIBRARY_TYPE=SHARED
```

- 进程内只有一份 netmech → 一份 io 线程池；
- 线程数由应用在 `main` 最早期 `netmech::initialize(N)` 决定，之后所有协议库共用；
- 退出前先 `close()` 各库对象，再 `netmech::shutdown()`。
- **需要多套隔离池 / 不用默认池**：modbusmech 的 `create_*` 都支持注入
  `std::shared_ptr<netmech::Runtime>`（默认 `nullptr` = 进程级默认运行时）：

  ```cpp
  netmech::Runtime::Options o;
  o.io_threads = 4;                      // 应用自己定
  auto rt = netmech::Runtime::create(o);

  auto slave  = modbusmech::Slave::create_tcp(1502, sc, bank, ec, rt);
  auto master = modbusmech::Master::create_tcp("127.0.0.1", 1502, mc, ec, rt);
  auto async  = modbusmech::AsyncMaster::create_tcp(host, port, ac, ec, rt);

  // ... 使用 ...

  rt->shutdown();                        // 只停这一套池
  ```

  RTU 入口（`create_rtu`）同样支持；未传 runtime 时行为与以前完全一致。

实现细节与验证数据见 netmech `docs/packaging.md` 的「多库共享 netmech」一节。
