# Modbus TCP 非阻塞方案评估

> 状态：评估稿（未改代码）。目标读者：库作者。
> 结论先说：**现状不是缺陷，是"把异步底座降级成同步用"的取舍**。要不要改取决于现场形态——下面是判定条件、三种方案和实测方案。

---

## 1. 现状：一条连接只有一个在途事务

### 1.1 调用链

```
应用线程
  └─ Master::read_holding_registers()            ← 阻塞
       └─ Impl::transaction()                     ← 重试循环
            ├─ Transport::send(pdu, unit)         ← 组装 MBAP，写 Conn
            └─ Transport::receive(out, timeout)   ← 条件变量等响应
                 └─ TcpTransport::on_bytes()      ← netmech io 线程回调
```

- `TcpTransport` 维护 `m_pending_tid`（`src/tcp.hpp:97-98`）：**同一时刻只记录一个**事务号。
- `TcpTransport::receive()` 只认 `tid == m_pending_tid`（`src/tcp.cpp:322-332`）；不匹配的整帧吃掉并返回 `bad_transaction`。
- 每个 `TcpTransport` 建一个独立的 `netmech::Mechanism` + 一条 TCP 连接（`src/tcp.cpp:174-186`）。
- `Master` 内部**不加锁**，`master.hpp:89-100` 明确写死「一个 Master 只从一个线程用」。

### 1.2 实际并发度

| 维度 | 现状 |
|---|---|
| 同一 TCP 连接上的在途请求 | **1**（受 `m_pending_tid` 限制） |
| 同一连接上的吞吐上限 | ≈ `1 / RTT`。RTT=10ms ⇒ 100 请求/秒/连接 |
| 并发手段 | 多开 `Master` 实例（每实例一条连接 + 一个阻塞线程） |
| 线程成本 | 每个 `Master` 占 1 个应用线程（默认 8MB 栈虚拟保留） |
| io 线程 | netmech 的进程级池（共享，与 Master 数无关） |

50 台设备集中采集的两种写法：
1. 50 个 Master（50 条连接 + 50 条线程）；或
2. 1 个 Master + 显式 `unit` 参数轮询 50 台（1 条线程，但吞吐仍是 100 请求/秒）。

**关键点**：MBAP 的事务号本来就是为流水线设计的（`docs/design/overview-design.md` §6.2 自己也写了"有事务 ID → 天然支持多请求并发"），而当前的实现把这个能力关掉了。

---

## 2. 值不值得改：判定条件

改造成本不低，先按现场形态判断：

| 现场形态 | 建议 |
|---|---|
| 每台设备一条连接、低频轮询（≤10 次/秒） | **保持现状**。改造成本收不回来 |
| 多台设备共享一条连接 / 网关后接一大片从站，需要 **几百~几千请求/秒** | 做**方案 B**（见下），收益直接体现在吞吐 |
| 单进程要承载上千连接、线程数是瓶颈 | 做**方案 C**（异步 API），或"B + 每线程多连接" |
| 只是嫌阻塞 API 用起来别扭 | 不改。阻塞语义对 RTU/485 场景是合理的（`CHANGELOG` 里"串口本质一问一答"的论证成立） |

**必须先测**：`RTT` 和"设备是否支持流水线"决定了 B 有没有收益。没有实测数据就改 = 白改（见 §5 测量方案）。

---

## 3. 三种方案

### 方案 A：保持现状（基准）

- 优点：API 简单、与 libmodbus 习惯一致、**设备兼容性最好**（永远只有一个请求在途）。
- 缺点：线程多、单连接吞吐卡在 1/RTT。

### 方案 B：解除"单在途"限制（阻塞 API 不变）★ 推荐先做

**思路**：把 `m_pending_tid` 换成 `tid → Pending` 表，让**多个调用线程**各自阻塞在自己的 TID 上。对外 API 一个字不改。

```
调用线程 1 ─┐
调用线程 2 ─┼→ send(tid=k) → 插表 → 各自 cv.wait(tid=k)
调用线程 3 ─┘
                    io 线程 on_bytes → 解析整帧 → 查表 → 唤醒对应 Pending
```

改动点：

| 位置 | 改动 |
|---|---|
| `tcp.cpp` | ① `on_bytes` 里做**完整帧解析**（现在是 `receive()` 里做）；② 按 TID 查表分发；③ 无主的迟到帧丢弃并记 `bad_transaction`；④ 断开时唤醒全部 Pending |
| `master.cpp` | `transaction()` 不再直接 `receive()`，改为"发→等自己那一 TID"；`resync()` 语义不变 |
| `tcp.hpp` | `m_pending_tid` → `std::unordered_map<std::uint16_t, std::shared_ptr<Pending>>` + `std::mutex` |
| `master.hpp` | 新增 `Config::max_in_flight`（**默认 1**，见下）；删掉"一个 Master 只从一个线程用"的限制 |
| 测试 | 三组：突发 N 请求（断言按 TID 一一对应）、迟到响应（断言丢弃不误配）、多线程并发 |

关键设计点（这几条决定成败）：

1. **解析必须搬到 io 线程**。现在"哪个线程调 `receive()` 谁解析"；多线程同时在途时，解析只能有一个执行者 —— 放进 `on_bytes`（io 线程），`m_rx` 只在该线程访问。
2. **`max_in_flight` 默认 1**。很多 Modbus TCP 设备/网关其实是"单请求串行处理"，收到第二个请求会丢或延后。默认值 1 = 与现在行为逐字相同；用户在实测确认设备支持后才调到 4/8/16。**这一条让 B 从"行为变更"降级为"可选增强"**。
3. **超时后的 TID 复用**。超时把 TID 从表里摘掉；迟到响应查不到 → 丢弃。这等价于现在 `bad_transaction` 的语义，不会有"误唤醒别人的等待者"。
4. **`Conn::write` 配额**。并发发送时写队列可能满（netmech 返回 false）→ 必须把刚插入的 Pending 摘掉，否则等待者要空等到超时。建议新增 `ModbusErrc::busy` 区分"没发出去"和"发出去了但没响应"。
5. **`Result` 语义不变**：每个调用线程拿到的仍是同步的 `Result`，错误分类（传输失败 vs 协议异常）不变。

- 收益：同一连接吞吐 × N（N = 实测可用的在途数）；线程数可以降到 1（一个线程轮询 50 台设备）。
- 代价：`tcp.cpp` 约 200 行改写 + 约 100 行测试；**设备兼容性风险由 `max_in_flight` 开关兜住**。
- 风险：解析逻辑搬家容易引入"半帧/粘帧"回归 —— 现有 `test_tcp.cpp` 的 5 个用例 + 新增 3 个用例要覆盖。

### 方案 C：异步回调 API（不阻塞）

**思路**：新增 `AsyncMaster`（或 `Master::read_holding_registers_async`），回调在 io 线程触发：

```cpp
using ResponseFn = std::function<void(Result, std::vector<std::uint16_t>)>;

// 立刻返回，不占调用线程
async->read_holding_registers(unit, addr, cnt, [](Result r, std::vector<std::uint16_t> regs) {
    if (r.ok()) { /* io 线程上下文 */ }
});
```

- 收益：零阻塞线程、天然流水线、与 netmech 的事件模型一致；可以做到"单线程管上千连接"。
- 代价/风险：
  - API 面积翻倍（8 个功能码 × 2 种风格）；建议只提供回调式 + 一个通用 `request()`，不铺满。
  - 回调在 io 线程 → 用户回调里不能阻塞（与 `Slave` 的 `on_client_connected` 同一条约束）。
  - buffer 生命周期（响应缓冲由库持有到回调返回，用 `Buf::owner` 语义）。
  - 用户的采集逻辑要从"顺序写代码"改成"状态机/回调"，迁移成本真实存在。
- 工作量：新增 `async_master.hpp/.cpp` 约 500~600 行 + 测试；3~5 人日。
- **可以建立在 B 之上**：B 做完后，C 基本只是"把 `transaction()` 的等待换成回调"，不需要再改解析层。

### 方案 D：C++20 协程

书写体验最好，但**本工程是 C++17**（`CMakeLists.txt:4-6`），且 CI 有 C++20 构造门禁（`.github/workflows/ci.yml` 的 `co_await` 检查）。**不可用**，除非整体升标准 —— 那是另一个决策（涉及 netmech 与全部下游）。

---

## 4. 建议

1. **先测**（§5）。如果现场 RTT 大且设备支持流水线 → 做 B。
2. **B 先于 C**：B 的 API 不变、风险可控、收益立竿见影；C 可以后续在 B 的解析层上加。
3. **B 的默认值必须是 `max_in_flight = 1`**：默认行为与今天完全一致，流水线由用户显式开启并承担设备兼容性风险。
4. **C 只做常用功能码**（01/03/04/06 + 通用 `request`），不铺满 8 个 × 2 风格。
5. **RTU 侧不要动**：485 是物理半双工，一问一答就是最优，"非阻塞"只会把复杂度推给应用。

---

## 5. 测量方案（改造前必须先跑）

**环境**：本机回环，被测从站直接用本库 `Slave::create_tcp`（避免引入第三方变量）。

**对照四组**：

| 组 | 拓扑 | 在途 | 线程 |
|---|---|---|---|
| A1 | 1 连接 | 1 | 1 |
| A2 | 8 连接 | 1/连接 | 8 |
| B | 1 连接 | 8 | 1（B 实现后） |
| C | 1 连接 | 无上限 | 1（C 实现后） |

**脚本**：每组发 10000 次 `03 读 10 个寄存器`，记录：

- `ops/s`、`P50/P99 延迟`；
- 进程线程数、RSS；
- `Master::stats()` 的 `timeouts`（非 0 说明对端没跟上）。

**预期（未实测，写在这里供事后对照）**：A1 受限于 1/RTT；B 在 RTT 主导时应接近线性提升到设备上限；若设备串行处理，B 的 `timeouts` 会明显上升 —— 这正是 `max_in_flight` 该调回 1 的信号。

**现场验证（必做）**：任何真实 Modbus TCP 设备都要单测一遍"同时发 2 个请求是否都能收到响应"。这一条没有捷径。

---

## 6. 与审查报告的关系

- 本评估**不修改任何代码**（属方案讨论）。
- 方案 B 完成后，审查报告里 **P1-7**（`m_conn` 跨线程无同步读写）自然被一并解决：解析与分发都在 io 线程，`m_conn` 只在建连后只读。
- 方案 B 不改变 RTU 路径的任何行为。

---

## 7. TCP 从站：与 B/C 的关系、现状评估、MBAP 与元信息

### 7.1 B/C 只针对主站

**是的。** B（解除单在途）和 C（异步 API）解决的是"**主站**如何复用一条连接发多个请求"，从站侧不需要：

- 从站本来就是事件驱动：`on_read` 收字节 → `process()` 切帧 → `handle_pdu` → 回写，没有"异步转同步"这一步，也就没有 B 要解决的问题；
- 从站没有 TID 记账（只需把请求的 tid **原样回填**）；
- 从站不发起请求，所以没有重试、没有超时等待；
- 粘包/半包天然支持（MBAP 有长度字段）。

所以从站确实**比主站好处理得多**——但它有自己的难点（7.3），只是难点不在"并发复用连接"上。

### 7.2 从站现状（收包 → 解析 → 回调 → 回应）

```
netmech on_read(io 线程)
  └─ TcpSession::on_read        累积到 m_rx（>2×ADU 直接断连，防 DoS）
       └─ process()             按 MBAP 的 len 循环切帧（支持粘包/半包）
            ├─ pick_bank(unit)  多从站分派（可抛异常，已收敛）
            ├─ handle_pdu()     PDU 解析 → RegisterBank 回调 → 组响应
            └─ write_conn()     回填 tid/unit 后写回
```

评价：结构正确；**顺序响应**（一帧处理完再处理下一帧），符合绝大多数现场客户端预期。

### 7.3 从站真正的难点（比"好处理"多出来的部分）

| # | 问题 | 现状 | 影响 | 建议 |
|---|---|---|---|---|
| 1 | **bank 回调在 io 线程同步执行** | 是 | netmech 是 N 个 io_context / 每 context 1 线程，多个会话**共享**同一线程。一个慢 bank（访问真实设备、查库）会卡住同线程上的**其它客户端** | 先文档约束「bank 回调要快（<1ms 量级）」；确有慢操作再考虑卸载到 `netmech::WorkerPool`——但那会打乱响应顺序（Modbus TCP 允许按 tid 乱序，现场客户端未必接受） |
| 2 | **没有空闲超时** | 无 | 客户端连上不发数据就永久占一个 session（1KB 缓冲 + handler）；长期运行会积累半开连接 | 加 `Config::idle_timeout`（用 netmech 的 Timer，在某条连接 N 秒无请求时断开） |
| 3 | **没有最大连接数** | 无 | 异常/恶意客户端可把 fd 与内存吃光 | 加 `Config::max_clients`：accept 时超限直接拒（复用 `on_client_connected` 的拒绝路径） |
| 4 | **响应写回背压** | `write_conn` 忽略 `Conn::write` 返回值 | netmech 写队列满（默认 2MB/256 笔）时响应**静默丢弃**，客户端只会超时 | 写失败 → 断开该连接（对端立刻能重连重来），并计入统计 |
| 5 | **半帧 + 断开** | 随 session 一起丢弃 | 无 | 不需要改（新连接 = 新 session = 空 m_rx，正确） |
| 6 | **多会话共享 bank** | 文档要求应用加锁 | 数据竞争由应用负责（与设计文档一致） | 不需要改；可考虑提供一个加锁包装器 |

### 7.4 MBAP 头的处理是否恰当

逐项核对（`src/slave.cpp` 的 `TcpSession::process`）：

| 检查项 | 现行实现 | 评价 |
|---|---|---|
| 协议 ID 必须为 0 | `proto != 0` → 断连 | ✅ 合理。非 0 说明不是 Modbus 帧，长度字段也不可信，断开比猜测安全 |
| 长度下限 | `len < 2` → 断连 | ✅（unit + 至少 1 字节 PDU） |
| 长度上限 | `total = 6 + len > 260` → 断连 | ✅（Modbus TCP ADU 上限 260） |
| 半包 | `m_rx.size() >= 7` 才解析；`< total` 就等 | ✅ |
| 粘包 | `while` 循环逐帧处理 | ✅ |
| PDU 取法 | `m_rx[7 .. total]` | ✅ |
| 事务 ID 回填 | 原样回填请求的 tid | ✅（规范要求） |
| Unit ID 回填 | 原样回填请求的 unit | ✅（规范要求；网关按它路由） |
| Unit ID 用于分派 | `pick_bank(unit)` | ✅ |
| 广播 `unit=0`（TCP） | 正常处理并**回应** | ⚠️ 与 RTU 不同（RTU 广播只执行不回应）。TCP 本身没有广播概念，单播回应是合理的；但若设备/网关把 unit=0 当广播，需要在文档里写明本库的行为 |
| 未知功能码 | 原来静默丢弃（`decode_pdu` 判畸形） | ❌ 规范要求回 `01`。**本轮已修**（见 7.6） |
| 多帧合并 | 不支持（Modbus 本身不允许） | ✅ |

结论：**MBAP 头处理基本恰当**（长度校验、粘包/半包、tid/unit 回填都对）；真正的缺口是"未知功能码静默丢弃"（已修）和上面 7.3 的运维类缺口（超时/连接数/背压）。

### 7.5 事务 ID 与 Unit ID 是否传递给用户？

**现状：都不传。**

- **Unit ID**：只在"选 bank"这一步通过 `Config::bank_for(unit)` 暴露；`RegisterBank` 的 8 个回调里拿不到 unit。
- **Transaction ID**：完全内部（从请求原样回填到响应），用户拿不到。

**评估**：

- 对"普通从站"，tid 是传输层实现细节，**不建议**塞进 `RegisterBank`（libmodbus 也不给）。
- 但有三类需求确实需要元信息：
  1. 一个 bank 服务多个 unit，想按 unit 区分地址空间（现在只能用 `bank_for` 拆成多个 bank 实例）；
  2. 审计/日志（记录"谁在读哪个地址"，tid 用于和设备日志对齐）；
  3. 网关（按 unit 转发到串口/其它设备）。

**三种做法（按侵入性排序）**：

| 方案 | 做法 | 代价 | 适用 |
|---|---|---|---|
| **A 不加（推荐先这样）** | `bank_for` 覆盖 (1)；(2)(3) 属于应用/网关层 | 0 | 绝大多数从站 |
| **B 加一个无状态帧级钩子** | `Config::on_request`：`std::function<void(const RequestInfo&)>`，`RequestInfo{unit, transaction_id, function, address, count, broadcast}`，在 io 线程按帧顺序回调，只读通知、不能改结果 | 公共 API +1 个回调 + 1 个结构体（源兼容）；**不能**用它给 bank 传状态（bank 跨会话共享，会竞争） | 需要审计/日志/网关 |
| **C 改 RegisterBank 签名** | 8 个方法都加 `const RequestInfo&` | 破坏 ABI 与所有下游实现 | 不推荐 |

**建议**：先 A；若确需 (2)(3)，做 B（改动小、不破坏现有实现）。

### 7.6 本轮顺带修掉的两个从站缺陷

1. **未知功能码静默丢弃 → 回 `01`**（`slave.cpp` 的 `handle_pdu`）：`decode_pdu` 对未知功能码返回 `invalid_argument`，原来的 `handle_pdu` 把它当畸形帧丢掉，主站只能等超时。现在先判 `is_known_function()`，不认识就回 `功能码|0x80 + 01`。测试：`slave_unknown_function_returns_01`。
2. **`bank_for` 抛异常会打死从站**（`slave.cpp` 的 `pick_bank`）：RTU 侧异常打断帧线程（外层兜底后停摆）、TCP 侧关连接。现在按"该 unit 不存在"收敛（返回空 + `bad_frames+1`），从站继续服务。测试：`slave_bank_for_exception_does_not_kill_slave`。

### 7.7 后续已知缺口（未做）

- **`RegisterBank` 现在会收到 `unit`**（已实现，见 7.5 的结论：unit 必须给）。
  `transaction_id` 仍未暴露——若将来需要审计/tid 对齐，按 7.5 的方案 B 加
  `Config::on_request` 钩子即可（不破坏 bank 签名）。
- **`TcpSession` 的引用环**：netmech 的 `finish_handler` 会保留
  `retired_handler` 快照直到 `Conn::Impl` 析构，而会话持 `shared_ptr<Conn>`
  —— 两者互持，`clients` 永不归零。已在 `on_close()` 里 `m_conn.reset()`
  修掉（netmech 的 README 示例也要求 on_close 里 reset）。

### 7.8 结论更新（2026-11，方案 B + C 已落地）

- **方案 B（多事务流水线）已实现**：TCP 传输层改成 `tid → 等待者` 表，解析统一
  搬到 io 线程（`TcpTransport::on_bytes`）。`Master::Config::max_in_flight`
  默认 1（与 RTU 一样串行、兼容性最好），调到大于 1 后**同一 Master 可被多个
  线程并发使用**，超过上限返回 `ModbusErrc::busy`。
- **方案 C（异步 API）已实现为"工作线程 + 派发线程"形态**（不是零线程事件
  驱动）：`AsyncMaster`（`include/modbusmech/async_master.hpp`）内部 N 条工作
  线程驱动阻塞版 Master，回调统一在一条派发线程上串行执行；**每个请求自带
  `Config::timeout`，超时由库负责触发回调**（`Result{ec = timeout}`），
  调用方不需要再兜超时——这满足"回调要和阻塞一样能做到超时"的要求。
- **重连策略**：`auto_reconnect` 默认 **false**（连接断了就报错）。打开后是
  "请求时无连接就建连、建连失败/超时立即返回失败"的按需语义，不做后台重试。
- 仍未做（可选）：完全零线程的事件驱动 API（需要把 AsyncMaster 的工作线程换成
  在 io 线程直接驱动传输层）；`Config::on_request` 元信息钩子。



