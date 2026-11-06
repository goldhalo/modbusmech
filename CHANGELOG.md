# Changelog

## [未发布]

### 新增

**示例程序 + 互测手册**

- `example/master_main.cpp` → `modbusmech_master`：`rh/ri/rc/rd/wr/wrs/wc/wcs/poll`
  子命令；`--async`（回调 + 内置超时）、`--inflight N`（TCP 流水线）、`--unit`、
  `--parity E`（8E1）、`--auto-reconnect`、`--connect-timeout` 等选项
- `example/slave_main.cpp` → `modbusmech_slave`：`tcp/rtu` 两种传输；
  `--units`（一个实例模拟多台从站）、`--idle`/`--max-clients`（连接管理）、
  `--boundary silence|length`（RTU 帧边界）、`--regs`；内置确定性初值的 DemoBank
- `docs/interop.md`：与 libmodbus / mthings 的互测步骤、寄存器初值表、
  10 条判定标准与常见坑
- CMake 增加 `MODBUSMECH_BUILD_EXAMPLES`（默认 ON）

**异步主站 `AsyncMaster`**

- `include/modbusmech/async_master.hpp`：8 个功能码的回调式 API；
  **每个请求自带 `Config::timeout`，超时由库触发回调**（回调恰好一次、
  全在同一条派发线程上串行执行），调用方不需要自己兜超时

**TCP 多事务流水线**

- `Master::Config::max_in_flight`（默认 1）：>1 时同一 TCP 连接上允许多个
  请求在途（MBAP 事务号匹配），同一个 `Master` 可被多个线程并发使用；
  超过上限返回新增的 `ModbusErrc::busy`
- 传输层接口改为 `send(pdu, unit, tid&)` / `receive(out, timeout, unit, tid)`，
  解析统一在 io 线程完成

**从站连接管理**

- `Slave::Config::idle_timeout`：TCP 空闲连接自动断开
- `Slave::Config::max_clients`：连接数上限（超出在 accept 阶段拒绝）
- `Slave::Stats::send_failures`：响应写不出去（队满/已断）的计数，且会主动断开连接

**CMake 安装与导出（find_package）**

- `cmake --install` 现在装出完整的一套：头文件、静态库、`lib/cmake/modbusmech/` 下的
  `modbusmechConfig.cmake` / `modbusmechConfigVersion.cmake` / `modbusmechTargets.cmake`，
  以及 `share/doc/modbusmech` 下的 LICENSE / NOTICE
- 使用者只需要两行：
  ```cmake
  find_package(modbusmech REQUIRED)
  target_link_libraries(app PRIVATE modbusmech::modbusmech)
  ```
- Config 里 `find_dependency(netmech)`：`libmodbusmech.a + libnetmech.a + pthread`
  这一串不用再手写

- 默认安装前缀：Linux `/usr/local`，Windows `C:/Program Files/modbusmech`

### 变更（**破坏性**）

- **`RegisterBank` 的 8 个回调新增第一个参数 `unit`**（RTU 从站地址 / TCP Unit ID）：
  多从站模拟与协议透传场景靠它区分设备。所有实现方需要改签名并重新编译
- `Master::Config::auto_reconnect` 默认值改为 **false**（连接断了就报错）；
  打开后是"请求时无连接就建连、建连失败/超时直接返回失败"的按需语义
- `Master::Config::connect_timeout` 新增（默认 3s）

### 修复

- `RegisterBank` 抛异常导致 RTU 从站进程 `std::terminate` / TCP 会话僵死 →
  统一收敛成异常码 04
- 未知功能码被静默丢弃 → 按规范回 `01`（illegal function）
- `bank_for` 抛异常会打死从站 → 按"该 unit 不存在"处理
- RTU 主站发送帧 / RTU 从站响应帧的 use-after-free（改用 owner 托管缓冲）
- 从站会话引用环导致 `client_count()` 永不归零
- TCP 从站不校验 `Config::addresses`（配了 `--units` 也不生效）
- 对端关闭时已收到的完整响应被丢弃（短连接设备随机超时）

---

## [0.9.0] — 首个版本

Modbus RTU / TCP 的协议实现。网络层用 netmech，协议层参照公开规范独立实现
（不复制 libmodbus，保持 MIT）。

### 新增

**PDU 层（纯函数，无 IO）**

- `encode_pdu` / `decode_pdu`：8 个功能码的编解码
  （01 读线圈 / 02 读离散输入 / 03 读保持寄存器 / 04 读输入寄存器 /
   05 写单线圈 / 06 写单寄存器 / 0F 写多线圈 / 10 写多寄存器）
- `crc16`：Modbus CRC（多项式 0xA001，初值 0xFFFF）
- `parse_read_response` / `parse_write_response`：响应解析 + 异常码提取
- 位打包按规范"每字节低位在前"

**RTU 传输**

- `RtuTransport`：串口 + 帧边界 + CRC 校验
- 接收分三阶段：等 2 字节（地址+功能码）→ 按功能码推总长 → 等够
- 超时的残留帧在下次请求前清掉（避免跟新响应混在一起）

**主站**

- `Master::create_rtu`：RTU 主站
- `read_coils` / `read_discrete_inputs` / `read_holding_registers` /
  `read_input_registers` / `write_single_coil` / `write_single_register` /
  `write_multiple_coils` / `write_multiple_registers`
- **同步 API**（阻塞到响应或超时），内部用条件变量把 netmech 的异步读转同步
- 传输失败重试（`Config::retries`），协议异常不重试

**RTU 从站**

- `Slave::create_rtu`：RTU 从站，`RegisterBank` 回调提供寄存器
- **silence 帧边界**：从站不知道请求有多长，只能靠"线路安静下来"切帧
  —— 这是 3.5 字符静默唯一的用武之地
- CRC 错 / 地址不匹配 → **不应答**（规范要求）
- 广播（地址 0）→ 执行但不应答
- 8 个功能码全部支持，地址越界自动回异常码 02

**TCP 传输**

- `TcpTransport`：MBAP（事务 ID / 协议 ID / 长度 / 单元 ID）
- 事务 ID 自增并校验（响应必须匹配，否则 `bad_transaction`）
- 单元 ID 校验（一个 TCP 连接可访问多个单元，靠它区分）
- 有长度字段 → **不需要**帧边界判断，也不需要静默

**TCP 从站（多客户端）**

- `Slave::create_tcp`：监听端口，每个客户端一个独立会话
- **所有会话共享同一个 `RegisterBank`**，应用需自行保证线程安全
- 每个会话独立累积字节、按 MBAP 长度切帧（可能一次收到多帧）

**结果类型**

- `Result` 区分"传输失败"（`ec`）和"从站拒绝"（`exception`）
- `retryable()` 给出重试建议

### 设计要点

1. **寄存器由应用提供**（`RegisterBank` 回调），不用静态数组 ——
   应用的数据不用同步两份，写操作能触发副作用。
2. **帧边界显式配置**：`silence`（规范做法，3.5 字符）或 `length`。
   Windows 的默认定时器分辨率是 15.6ms，4ms 静默判不准，建议用 length。
3. **许可证保持 MIT**：参照规范独立实现，不复制 libmodbus（LGPL）。

### 关于 3.5 字符静默

**只有 RS-485 半双工需要它**——同一对线不能同时收发，从站只能靠"线路安静"
判断帧结束。RS-232 是全双工、TCP 有 MBAP 长度字段，都不需要。

库**不按波特率自动算**这个值（算出来大概率不准：USB 转 485 有切换延迟、
不同设备容忍度不同）。由使用者按实测配置 `Config::silence_ms`，默认 5ms。

主站侧也不用它——主站知道自己在等什么，按功能码推长度读就够。

### 已知限制
- 从站的 silence 帧边界精度受平台定时器限制（Windows 默认 15.6ms 分辨率）
- 不支持"批量合并读"（应用层自己做）

### 验证

| 平台 | 结果 |
|---|---|
| Linux x86_64（GCC 15） | 34 用例 / 170 断言（PDU 18 + RTU 主站 5 + RTU 从站 6 + TCP 5） |
| Windows x64（MSVC 14.44） | 34 用例 / 111 断言（RTU 端到端在 Windows 上跳过 pty 部分） |
