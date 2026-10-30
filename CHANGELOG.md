# Changelog

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

**结果类型**

- `Result` 区分"传输失败"（`ec`）和"从站拒绝"（`exception`）
- `retryable()` 给出重试建议

### 设计要点

1. **寄存器由应用提供**（`RegisterBank` 回调），不用静态数组 ——
   应用的数据不用同步两份，写操作能触发副作用。
2. **帧边界显式配置**：`silence`（规范做法，3.5 字符）或 `length`。
   Windows 的默认定时器分辨率是 15.6ms，4ms 静默判不准，建议用 length。
3. **许可证保持 MIT**：参照规范独立实现，不复制 libmodbus（LGPL）。

### 已知限制

- TCP 传输与从站尚未实现（RTU 主站已完成）
- 从站的 silence 帧边界精度受平台定时器限制
- 不支持"批量合并读"（应用层自己做）

### 验证

| 平台 | 结果 |
|---|---|
| Linux x86_64（GCC 15） | 23 用例 / 93 断言（PDU 18 + RTU 端到端 5，用 pty 虚拟串口） |
| Windows x64（MSVC 14.44） | 23 用例 / 71 断言（RTU 端到端在 Windows 上跳过 pty 部分） |
