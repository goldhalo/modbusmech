# modbusmech 接入计划（IPMS / EAS 侧）

> 决策已定：**A-② 带 unit 的重载** · **B-① 每串口一个采集线程** · **C-① 先接从站**
>
> 状态：A-② 已实现（`3142ca6`），本文是 C-① 与 B-① 的落地步骤。

## 0. 为什么是这个顺序

| 阶段 | 做什么 | 为什么排这个位置 |
|---|---|---|
| **1. 从站（Slave）** | 用 modbusmech 把 ECS 的现有点表暴露成 Modbus 从站 | **本机就能端到端验证**（MThings 当主站读）；不碰现场设备，出问题损失为零 |
| **2. 主站（Master）** | 用它替换 MocModbusRtu 去采现场设备 | 价值最大，但要上真机，必须等第 1 阶段证明库本身可靠 |

第 1 阶段还有一个副产品：它把"点表 ↔ 寄存器"的映射**先固定下来**，第 2 阶段采回来的数据直接套同一张表，不用写两遍映射。

---

## 1. 阶段一：从站接入（本机验证）

### 1.1 核心映射：`RegisterBank` ← 现有点表

modbusmech 的从站不持有数据，它每次请求都回调 `RegisterBank`。所以要做的是一个**适配器**，把 Modbus 地址翻译成 EAS 的点位：

```cpp
// 位置建议：eas/adapter/modbus/point_bank.hpp
// 职责：Modbus 地址/功能码 → 点表点位 → 当前值
class PointBank final : public modbusmech::RegisterBank {
public:
    // 点表在构造时按设备准备好（地址段 → 点位列表），运行期只查表
    explicit PointBank(std::shared_ptr<const DevicePointMap> map) : m_map(std::move(map)) {}

    bool read_holding_registers(std::uint16_t addr, std::uint16_t count,
                                std::vector<std::uint16_t>& out) override
    {
        // 契约：成功时 out.size() **必须** == count；返回 false ⇒ 库自动回异常码 02
        out.resize(count);
        for (std::uint16_t i = 0; i < count; ++i) {
            const PointId id = m_map->lookup(addr + i);      // 越界 ⇒ 返回 invalid
            if (id == kInvalidPoint) {
                return false;                                 // → 异常码 02（地址越界）
            }
            out[i] = m_snapshot.read_u16(id);                 // 见 1.2 的线程问题
        }
        return true;
    }

    // 读类：read_coils / read_discrete_inputs / read_input_registers 同理
    // 写类：write_single_coil / write_single_register 等 —— 若本阶段只读，
    //       直接 return false（库回异常码 01/02，主站侧看到的是"不支持"）
private:
    std::shared_ptr<const DevicePointMap> m_map;
    PointSnapshot                         m_snapshot;
};
```

**要点**

- `RegisterBank` 的 8 个方法**都在 netmech 的 io 线程上被调用**（TCP 从站时，不同客户端甚至可能在不同线程）→ `PointBank` 必须线程安全，或者只读一份不可变快照。
- 返回 `false` 会自动变成异常码 **02（非法数据地址）**，不要自己拼异常帧。
- 每次尽量让主站**批量读**（`count` 大一点），别让它逐点读 —— 485 上每帧的固定开销远大于数据本身。

### 1.2 线程与数据来源

```
ECS 采集/业务线程  ──写──▶  PointSnapshot（双缓冲 / seqlock）
                                    │ 读（无锁、无阻塞）
netmech io 线程 ──RegisterBank 回调──┘
```

不要直接在回调里查数据库或等业务锁：回调阻塞会拖住该 io 线程上的**所有**客户端。

### 1.3 起从站（两种形态）

```cpp
// 形态 A：TCP 从站（最常用 —— 给第三方 SCADA / 组态软件读）
modbusmech::Slave::Config cfg;
cfg.bank_for = [bank](std::uint8_t unit) -> std::shared_ptr<modbusmech::RegisterBank> {
    // 一个 IP:port 后面挂多台"设备"，按 unit 分派；
    // 返回 nullptr ⇒ 该 unit 视为不存在（静默丢弃，不回应）
    return unit == 1 ? bank : nullptr;
};

std::error_code ec;
auto slave = modbusmech::Slave::create_tcp(502, cfg, bank, ec);

// 形态 B：RTU 从站（给现场 485 主站当设备）
//   silence_ms 默认 0 = 按波特率自动算（≤19200 用 3.5 字符，>19200 用 1.75ms）
auto rtu = modbusmech::Slave::create_rtu("/dev/ttyS3", 9600, cfg, bank, ec);
```

**要不要鉴权**：`cfg.authorize = [](const std::string& ip, std::uint16_t) { return ip == "…"; }`。
Modbus 协议本身没有认证，这是唯一能控"谁能连进来"的地方（返回 `false` 直接关连接，一帧都不读）。

### 1.4 验证清单

| 步骤 | 期望 |
|---|---|
| MThings 以 TCP 主站连 502，读保持寄存器 | 数据与 ECS 页面一致 |
| 读一个**没定义**的地址 | 回异常码 02（不是超时、不是错数据） |
| 两个客户端同时连 | 都正常；`slave->stats().requests` 两者累加 |
| 配置 `authorize` 拒绝某 IP | 连接立刻被关，`stats().rejected_clients` +1 |
| RTU：MThings 用 9600 读 | 稳定；若不稳先调 `silence_ms`（从 5ms 起） |

---

## 2. 阶段二：主站接入（替换 MocModbusRtu）

### 2.1 线程模型（B-①）

**一条 485 总线 = 一个 `Master` = 一个采集线程**。物理上总线只能一问一答，开多线程没有收益，只会增加出错面。

```cpp
// 位置建议：eas/adapter/modbus/poll_worker.cpp
class PollWorker {
public:
    // devices：这条总线上的从站（unit + 点位表）
    void run()
    {
        while (!m_stop.load()) {
            for (const auto& dev : m_devices) {
                std::vector<std::uint16_t> regs;
                // ★ 用带 unit 的重载：不碰 set_unit 的共享状态
                const auto r = m_master->read_holding_registers(dev.unit, dev.addr,
                                                                dev.count, regs);
                if (r.ok()) {
                    m_sink.publish(dev, regs);          // 写快照，不阻塞
                    ++dev.ok_count;
                } else if (r.exception != modbusmech::Exception::none) {
                    // 从站明确拒绝（地址越界/功能码不支持）：配置问题，记一次就好
                    m_sink.mark_exception(dev, r.exception);
                } else {
                    // 传输失败（超时 / CRC / 串口掉线）：设备离线
                    m_sink.mark_offline(dev, r.ec);
                }
            }
            std::this_thread::sleep_until(m_next_round);
        }
    }
private:
    std::shared_ptr<modbusmech::Master> m_master;   // 一条总线一个
    std::vector<DevicePoll>             m_devices;
};
```

### 2.2 ★ 一个必须先算清楚的账：50 台设备 1 秒轮询**做不到**

9600bps、每台读一次保持寄存器：

```
请求   8 字节  +  响应 ≈ 5 + 2×N 字节      （N = 寄存器个数）
一帧   约 23 字节  ≈ 24 ms（含间隔与切换延迟）
50 台  ≈ 1.2 s ～ 1.5 s   ← 已经超过 1 秒
```

所以现场必须**分级轮询**，这也是先做从站、把点位映射定下来的另一个理由（哪些点重要，映射表里就有）：

| 级别 | 周期 | 内容 |
|---|---|---|
| 快 | 1 s | 电流/功率/SOC 这类实时量（每台 1 次批量读） |
| 慢 | 5–10 s | 电度、累计量、参数 |
| 按需 | 手动/告警触发 | 配置参数、诊断寄存器 |

实现上就是 `PollWorker` 里维护两个设备队列，按不同周期各跑一轮。

> 想再快只有两条路：提波特率（19200/38400，先确认现场线材与设备支持）或拆总线（一条 485 挂 25 台）。

### 2.3 建立 Master

```cpp
modbusmech::Master::Config cfg;
cfg.timeout     = std::chrono::milliseconds(500);   // 9600 下一帧 ≈25ms，500ms 足够
cfg.retries     = 1;
cfg.retry_delay = std::chrono::milliseconds(50);

std::error_code ec;
// 485
auto master = modbusmech::Master::create_rtu("/dev/ttyS3", cfg, 9600, ec);
// 或 TCP（Modbus TCP 设备 / 网关）
// auto master = modbusmech::Master::create_tcp("192.168.1.10", 502, cfg, ec);
```

### 2.4 与现有代码的对接点（迁移清单）

| 现在（MocModbusRtu） | 换成 | 注意 |
|---|---|---|
| `modbus_new_rtu()` / `modbus_connect()` | `Master::create_rtu()` | 每条总线一个实例；失败码在 `ec` 里 |
| `modbus_set_slave()` | **不再需要** | 用带 unit 的重载，每次请求显式传 |
| `modbus_read_registers()` | `read_holding_registers(unit, addr, cnt, out)` | 返回 `Result`，不是 int |
| `modbus_read_input_registers()` | `read_input_registers(...)` | 同上 |
| `modbus_write_register()` / `write_bit()` | `write_single_register(...)` / `write_single_coil(...)` | 同上 |
| `modbus_strerror(errno)` | `Result::ec.message()` / `describe()` | 错误码是 `std::error_code` |
| 自己算 CRC / 组帧 | —— | 库内部负责 |
| `modbus_set_response_timeout()` | `Config::timeout` | |

**行为差异（必须知道）**

1. **`Result` 区分两类失败**：`r.ok()` / `r.exception != none`（从站拒绝，配置问题）/ 其他（传输失败，设备离线）。旧的 `int` 返回值把两者混在一起，上层就没法区分"设备坏了"和"点表配错了"。
2. **写操作失败可能是"写成功但响应丢了"**：重试前请先读回确认（库不做幂等保护）。
3. **阻塞**：`read_*` 会阻塞到超时（默认按 `cfg.timeout`）。别在 io 线程/UI 线程上调。

---

## 3. 两个阶段之间要固定的东西

- **点位表 → 寄存器地址**的映射（从站要按它应答，主站要按它解析）→ 建议存成一份 JSON/表，两侧共用。
- **异常语义**：从站返回 `false` = 地址越界（异常码 02）；主站侧看到异常码 02 就知道是"点表配错了"，不是"设备坏了"。
- **点位质量**：主站侧每次成功/失败都要落到快照的"质量位"上（设备离线时页面不能显示上一次的旧值）。

## 4. 未决 / 需要项目侧确认

1. **从站的点位是"原始寄存器"还是"工程值"？** 现在 `RegisterBank` 交的是 `uint16_t`，缩放/单位换算要么在 bank 里做（主站拿到工程值×10 这种），要么交给主站侧做。建议**从站只给原始值**，缩放放在点表里 —— 否则不同主站的缩放约定会把数据搞乱。
2. **32 位量（float / int32）的字节序**：Modbus 没有规定，现场常见 ABCD/CDAB/BADC/DCBA 四种。需要在点表里带一个 `wordOrder` 字段，从站与主站两侧都用它。
3. **是否要给从站加"只读"限制**：`PointBank` 的写方法现在可以返回 `false`（= 不支持），也可以实现。取决于是否允许外部 SCADA 反控。
