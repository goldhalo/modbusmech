# modbusmech 概要设计

> 状态：讨论稿。接口见 `include/modbusmech/`。

---

## 1. 定位

Modbus 主站 / 从站的协议实现，网络层用 [netmech](../netmech)（TCP + 串口）。

| 做 | 不做 |
|---|---|
| PDU 编解码（8 个功能码） | 定时轮询、批量合并（应用的事） |
| RTU / TCP 两种传输 | 设备模板、点位映射（应用的事） |
| 主站（同步请求）和从站（寄存器回调） | 认证、加密（Modbus 本身没有） |
| CRC16 / MBAP / 帧校验 | 网关路由 |

---

## 2. 为什么不用 libmodbus

libmodbus 是成熟的实现，但有两个问题：

### ① 许可证是 LGPL-2.1+

复制它的代码，modbusmech 就是衍生作品，必须整体 LGPL。本库要保持 MIT。

### ② 网络层是自己写的阻塞实现

```c
// libmodbus 的 IO 模型
FD_SET(ctx->s, &rset);
select(ctx->s + 1, &rset, NULL, NULL, tv);   // 阻塞等
read(ctx->s, rsp, len);                       // 阻塞读
```

这跟 [netmech](../netmech) 的异步模型是两套东西。硬换的话要改 `modbus_t`
的 `s` 字段、`backend` 表的 `select` 签名、`_modbus_receive_msg` 的等待逻辑。

### 所以：**照协议行为重写，不复制代码**

协议本身是公开规范（Modbus Application Protocol V1.1b3），实现可以独立。
用 libmodbus 做"对照测试"验证行为一致（跑同一批字节流，结果必须一样）。

---

## 3. 分层

```
应用（采集轮询、点位映射、设备模板）
    │
    ├─ Master  ──┐
    │            ├─ PDU 编解码（纯函数）
    ├─ Slave  ───┘
    │
    ├─ RTU 传输（帧边界 + CRC + netmech 串口）
    └─ TCP 传输（MBAP + netmech Conn）
```

**PDU 层是纯的**：输入字节、输出结构，没有 IO、没有超时、没有重试。
所有协议细节的正确性都靠这一层的测试兜住。

---

## 4. PDU 层

### 4.1 编解码

```cpp
// 请求结构 → 字节
std::vector<std::uint8_t> pdu;
encode_pdu(Function::read_holding_registers, {0x0000, 10}, pdu);
// → 03 00 00 00 0A

// 字节 → 请求结构（从站用）
DecodedPdu d;
decode_pdu(span{pdu.data(), pdu.size()}, d);
// d.function == read_holding_registers, d.address == 0, d.count == 10
```

### 4.2 功能码与格式

| 功能码 | 请求 | 响应 |
|---|---|---|
| 01 读线圈 | `01 addr(2) cnt(2)` | `01 byte_count(1) data(N)` |
| 02 读离散输入 | 同上 | 同上 |
| 03 读保持寄存器 | `03 addr(2) cnt(2)` | `03 byte_count(1) data(2N)` |
| 04 读输入寄存器 | 同上 | 同上 |
| 05 写单线圈 | `05 addr(2) FF00/0000` | 回显请求 |
| 06 写单寄存器 | `06 addr(2) value(2)` | 回显请求 |
| 0F 写多线圈 | `0F addr(2) cnt(2) bc(1) data` | `0F addr(2) cnt(2)` |
| 10 写多寄存器 | `10 addr(2) cnt(2) bc(1) data` | `10 addr(2) cnt(2)` |

**异常响应**：`function | 0x80` + 异常码（1 字节）。

### 4.3 位与字节的映射

读位（01/02）的响应里，每字节 8 个位，**低位在前**：

```
第 0 个线圈 → data[0] 的 bit0
第 1 个线圈 → data[0] 的 bit1
...
第 7 个线圈 → data[0] 的 bit7
第 8 个线圈 → data[1] 的 bit0
```

最后一个字节用 0 补齐（不要求对齐）。

### 4.4 CRC16

多项式 `0xA001`（反向的 0x8005），初值 `0xFFFF`，不取反。
**小端写入帧尾**（低字节在前）。

---

## 5. RTU 传输

### 5.1 帧结构

```
地址(1) | PDU(N, 1..253) | CRC16(2)
```

### 5.2 帧边界（最大的难点）

RTU 没有长度字段，靠**静默间隔**判断帧结束：

```
帧1                    ≥3.5 字符静默              帧2
[addr|pdu|crc]                                 [addr|pdu|crc]
```

| 波特率 | 3.5 字符时间 |
|---|---|
| 9600 | ≈ 4.0 ms |
| 19200 | ≈ 2.0 ms |
| 115200 | ≈ 0.3 ms（规范说 >19200 用固定 1.75ms） |

**实现上有两条路**：

| 方式 | 说明 | 问题 |
|---|---|---|
| **silence**（规范做法） | 收到数据后启动 4ms 定时器，超时即认为帧结束 | Windows 默认定时器分辨率 15.6ms，要 `timeBeginPeriod(1)` |
| **length**（实用做法） | 按功能码推算期望长度，读够就停 | 遇到未知功能码/异常帧会卡住 |

`Slave::Config::boundary` 让应用选。**默认 silence**（符合规范），
但文档里会写明"精度受平台定时器限制"。

**主站侧不需要这个**：主站知道自己在等什么响应，按长度读即可。

### 5.3 主站的收发流程

```
1. 组帧：unit | pdu | crc
2. 清空接收缓冲（丢掉上次的残留）
3. port->write(帧)
4. 等响应：
   a. 等第一个字节（timeout）
   b. 根据功能码算总长度
   c. 等够剩余字节
5. CRC 校验
6. 地址校验（必须等于请求的 unit）
7. 解析 PDU
```

**关键**：第 2 步"清空接收缓冲"很重要——上一次超时的残留帧如果不清，会跟
这次的响应混在一起。

### 5.4 从站的接收流程

```
1. 累积收到的字节
2. 判断帧是否完整：
   - silence 模式：静默 3.5 字符 → 完整
   - length 模式：按功能码算够 → 完整
3. CRC 校验（失败直接丢弃，从站不应答）
4. 地址校验（不是自己且不是广播 → 丢弃）
5. 解析 PDU → 调 RegisterBank
6. 组响应 → 写回
```

**广播（地址 0）**：执行但不响应。

---

## 6. TCP 传输

### 6.1 帧结构（MBAP + PDU）

```
事务ID(2) | 协议ID(2)=0 | 长度(2) | 单元ID(1) | PDU(N)
└─────────── MBAP 头 7 字节 ──────────┘
```

- **事务 ID**：主站自增，响应必须原样回填。用来匹配请求/响应（TCP 可能乱序）
- **协议 ID**：恒为 0
- **长度**：后面还有多少字节（单元 ID + PDU）

### 6.2 比 RTU 简单的地方

- 有长度字段 → 帧边界不用猜
- 有事务 ID → 天然支持多请求并发
- 没有 CRC（TCP 自己保证完整性）

### 6.3 从站支持多客户端

`Slave::create_tcp` 用 `netmech::Mechanism::listen`，每个 accept 的连接一个
独立的会话对象，但**共享同一个 `RegisterBank`**。

应用要保证 bank 线程安全（不同客户端可能在**不同的 io 线程**上并发访问）。

---

## 7. 主站的同步语义

所有 `Master::read_*` / `write_*` 都是**阻塞**的。

### 7.1 为什么阻塞

串口本质是"一问一答"：一个串口上同时挂 50 个电表，也只能一个一个问。
阻塞语义最自然，也最接近 libmodbus 的习惯（迁移成本低）。

### 7.2 内部怎么实现

netmech 是异步的，所以要"异步转同步"：

```cpp
Result read_holding_registers(addr, count, out) {
    for (int attempt = 0; attempt <= cfg.retries; ++attempt) {
        // 1. 组帧、发送
        // 2. 条件变量等响应（超时退出）
        // 3. 校验 → 成功则返回
        if (ok) return success;
        sleep(cfg.retry_delay);
    }
    return timeout;
}
```

条件变量由 `on_read` 回调 signal。

**代价**：调用线程被占住。采集 50 个电表时，本来就是串行轮询，可接受。

### 7.3 TCP 想并发怎么办

开多个 `Master` 实例（每个一条连接），各自在自己的线程里阻塞。
netmech 的 io 线程是共享的，所以 N 个 Master 不会变成 N 条 io 线程。

---

## 8. 从站的寄存器接口

### 8.1 为什么用回调而不是数组

libmodbus 用 `modbus_mapping_t`（四个静态数组）。问题：

- 应用的数据要**同步两份**（自己的结构 + 库里的数组）
- 写操作库直接改数组，应用感知不到（无法触发副作用）
- 内存翻倍（50 个电表 × 每个几百个寄存器）

**改成回调**：

```cpp
class MyBank : public RegisterBank {
    bool read_holding_registers(uint16_t addr, uint16_t count,
                                std::vector<uint16_t>& out) override {
        if (addr + count > m_regs.size()) return false;   // → 异常码 02
        out.assign(m_regs.begin() + addr, m_regs.begin() + addr + count);
        return true;
    }

    bool write_single_register(uint16_t addr, uint16_t value) override {
        if (addr >= m_regs.size()) return false;
        m_regs[addr] = value;
        on_register_changed(addr, value);   // ← 应用可以触发副作用
        return true;
    }
    // ...
};
```

### 8.2 返回值的约定

| 返回 | 库的行为 |
|---|---|
| `true` 且 `out` 填好 | 正常响应 |
| `false` | 回异常码 02（地址越界） |
| 抛异常 | 库捕获 → 回异常码 04（从站故障），并记 `stats.exceptions` |

---

## 9. 实施步骤

| 步 | 内容 | 结束标志 |
|---|---|---|
| 1 | `common.hpp` + `pdu.hpp` 的编解码 + CRC16 | 纯函数测试全过（不涉及网络） |
| 2 | RTU 主站（帧边界 + 超时 + 重试） | 对着从站（自测/模拟器）读写成功 |
| 3 | RTU 从站（silence 帧边界 + RegisterBank） | 主站↔从站端到端互通 |
| 4 | TCP 主站（MBAP + 事务 ID） | 对着 libmodbus 的 TCP 从站读写成功 |
| 5 | TCP 从站（多客户端） | 多个主站同时读 |
| 6 | 对照测试 | 同一批字节流，与 libmodbus 的结果逐字节一致 |

**先做 1**：PDU 层是纯函数，不涉及任何网络，最容易测，也最容易发现协议细节的错误。

---

## 10. 风险

| 风险 | 影响 | 处置 |
|---|---|---|
| RTU 帧边界判断不准（silence 精度） | 收到半个帧 / 多帧粘连 | 提供 length 模式兜底；实测调整 |
| Windows 定时器精度 15.6ms | 3.5 字符静默（4ms）判不准 | `timeBeginPeriod(1)`；或默认用 length 模式 |
| 多客户端共享 bank 的并发 | 数据竞争 | 文档明确要求应用加锁；可选提供包装器 |
| 从站遇到畸形请求 | 崩溃 | 所有解析函数都返回 error_code，不抛异常 |
| 与真实设备的兼容性 | 某些设备不守规范 | 保留"宽松模式"（不校验某些字段） |

---

## 11. 与 libmodbus 的差异（"该写明的"）

| 项 | libmodbus | modbusmech |
|---|---|---|
| 许可证 | LGPL-2.1+ | **MIT** |
| 语言 | C | C++17 |
| 网络层 | 自己写（select + read/write） | **netmech**（TCP + 串口共用 io_context） |
| 同步模型 | 阻塞（select 超时） | 阻塞（条件变量 + 超时） |
| 寄存器 | `modbus_mapping_t`（静态数组） | `RegisterBank`（**应用回调**） |
| 帧边界 | `select` 超时 ≈ 静默 | 显式配置（silence / length） |
| 多客户端 | 不支持 | TCP 从站**支持** |
| 代码来源 | — | **参照规范独立实现**，不复制 libmodbus 代码 |

**这一节要写进 README**：既是许可合规的说明，也让使用者知道行为差异在哪。
