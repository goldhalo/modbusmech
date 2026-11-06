# 互测手册（modbusmech ↔ 第三方实现）

> 本文是**逐步可执行**的互测清单：先本库自测，再与 libmodbus、mthings 对打，最后是判定标准与常见坑。
> 示例程序见 `example/master_main.cpp`、`example/slave_main.cpp`（默认随构建产出，`-DMODBUSMECH_BUILD_EXAMPLES=OFF` 可关）。

## 0.1 已跑通记录（libmodbus 3.2.0，2026-11）

工具与编译方法见 [ref/README.md](../ref/README.md)。两个方向都过：

| 方向 | 方式 | 结果 |
|---|---|---|
| libmodbus 从站 ← 我们的 Master | 进程内 `ref/interop_test`（44 项检查） | **PASS**，连跑 3 次 0 失败 |
| 我们的从站 ← libmodbus 主站 | 同上 | **PASS** |
| libmodbus 从站 ← `modbusmech_master`（示例 CLI） | 独立进程 | 四类读写 + 125 寄存器读 + 越界 02 + 流水线轮询 20/20 全过 |
| `modbusmech_slave`（示例 CLI）← libmodbus 主站 | 独立进程 `ref/lb_master.c` | **PASS**（18 项检查） |

libmodbus 侧的 API 细节坑（`modbus_tcp_accept` 返回 fd 而非 0、`read_bits` 返回逐位字节、
`send_raw_request` 首字节是 slave id 等）记录在 ref/README.md 末尾，省得下次再踩。

---


## 0. 约定：寄存器初值（对表基准）

`modbusmech_slave` 的 `DemoBank` 初值是确定的，互测时按这张表比对：

| 区 | 地址 | 初值 |
|---|---|---|
| 保持寄存器 03 | `0x0000..` | `holding[i] = i` |
| 输入寄存器 04 | `0x0000..` | `input[i] = 1000 + i` |
| 线圈 01 | `0x0000..` | `coils[i] = (i % 4 == 0)` → `1 0 0 0 1 0 0 0 ...` |
| 离散输入 02 | `0x0000..` | `discrete[i] = (i % 5 == 0)` → `1 0 0 0 0 1 0 ...` |

默认 `--regs 1000`，越界（`addr + count > 1000`）回异常码 **02**。

```bash
modbusmech_slave tcp 1502                 # 单元 1
modbusmech_slave tcp 1502 --units 1,2     # 同时模拟两台
modbusmech_slave tcp 1502 --regs 100 --quiet
```

---

## 1. 自测（本库主站 ↔ 本库从站）——已跑通

```bash
# 终端 A
./build/bin/modbusmech_slave tcp 1502 --units 1,2 --regs 100

# 终端 B
./build/bin/modbusmech_master tcp 127.0.0.1 1502 rh 0 10        # 0..9
./build/bin/modbusmech_master tcp 127.0.0.1 1502 ri 0 5         # 1000..1004
./build/bin/modbusmech_master tcp 127.0.0.1 1502 rc 0 8         # 1 0 0 0 1 0 0 0
./build/bin/modbusmech_master tcp 127.0.0.1 1502 rd 0 10        # 1 0 0 0 0 1 0 0 0 0
./build/bin/modbusmech_master tcp 127.0.0.1 1502 wr 5 1234
./build/bin/modbusmech_master tcp 127.0.0.1 1502 rh 5 1         # 0x04D2
./build/bin/modbusmech_master tcp 127.0.0.1 1502 wrs 10 0x11 0x22 0x33
./build/bin/modbusmech_master tcp 127.0.0.1 1502 rh 10 3        # 0x0011 0x0022 0x0033
./build/bin/modbusmech_master tcp 127.0.0.1 1502 wc 1 1
./build/bin/modbusmech_master tcp 127.0.0.1 1502 rc 0 4         # 1 1 0 0
./build/bin/modbusmech_master tcp 127.0.0.1 1502 --unit 2 rh 0 3
./build/bin/modbusmech_master tcp 127.0.0.1 1502 --async rh 0 4  # 异步（回调+超时）
./build/bin/modbusmech_master tcp 127.0.0.1 1502 --inflight 2 poll 0 4 5 20
./build/bin/modbusmech_master tcp 127.0.0.1 1502 rh 95 10       # 期望 exception: illegal data address
./build/bin/modbusmech_master tcp 127.0.0.1 1502 --unit 5 rh 0 2 # 期望 timeout（addresses 过滤）
```

---

## 2. 与 libmodbus 互测

libmodbus 是 C 实现（LGPL），自带一对测试程序 `unit-test-client` / `unit-test-server`，
在 `tests/` 目录下编译后可用。

### 2.1 libmodbus 当**从站**，modbusmech 当主站

```c
/* libmodbus 从站（TCP 1502）—— 约 30 行 */
#include <modbus.h>
int main(void) {
    modbus_t* ctx = modbus_new_tcp("0.0.0.0", 1502);
    modbus_mapping_t* map = modbus_mapping_new(100, 100, 100, 100); /* coil, discrete, holding, input */
    for (int i = 0; i < 100; ++i) map->tab_registers[i] = (uint16_t)i;
    int s = modbus_tcp_listen(ctx, 5);
    modbus_tcp_accept(ctx, &s);
    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    for (;;) {
        int rc = modbus_receive(ctx, query);
        if (rc > 0) modbus_reply(ctx, query, rc, map);
    }
}
```

```bash
./modbusmech_master tcp 127.0.0.1 1502 --unit 1 rh 0 10   # 期望 0..9
```

要点：
- libmodbus 的 `tab_registers` 就是**保持寄存器**（地址 0 基），第 i 个 = i，正好对上我们的基准表；
- libmodbus 默认单元号 1；我们的 `--unit` 必须与之一致；
- 它**不校验** MBAP 的 unit（直接回显），所以对齐靠 `--unit`。

### 2.2 modbusmech 当从站，libmodbus 当主站

```c
#include <modbus.h>
#include <stdio.h>
int main(void) {
    modbus_t* ctx = modbus_new_tcp("127.0.0.1", 1502);
    modbus_connect(ctx);
    uint16_t regs[10];
    int n = modbus_read_registers(ctx, 0, 10, regs);   /* 期望 0..9 */
    printf("read %d: %u %u %u\n", n, regs[0], regs[1], regs[9]);
    modbus_close(ctx); modbus_free(ctx);
    return 0;
}
```

或用 libmodbus 自带的 `unit-test-client` 打我们的从站（它会做写入+回读校验，覆盖面更大）：

```bash
./unit-test-client -m tcp -a 127.0.0.1 -p 1502 -t 1
```

⚠ `unit-test-client` 会在它认为的地址段里写值，我们的 DemoBank 是可写的，通常能过；
但它会校验"读回等于写入值"，若它用了 1 基地址假设，会在首地址偏移上暴露差异——这正好是
我们要确认的**地址口径**（Modbus 规范是 0 基；很多工具界面用 1 基显示）。

### 2.3 RTU 互测（虚拟串口）

Linux：
```bash
socat -d -d pty,raw,echo=0,link=/tmp/ttyM pty,raw,echo=0,link=/tmp/ttyS
# 终端 A（从站）
./modbusmech_slave rtu /tmp/ttyS 9600 --parity E --boundary length
# 终端 B（主站）
./modbusmech_master rtu /tmp/ttyM 9600 --parity E rh 0 10
```
Windows：用 com0com 建一对虚拟串口（COM5 ↔ COM6），从站用 COM6、主站用 COM5。
注意两边**波特率/校验位/停止位必须一致**（8E1 是最常见的现场配置）。

---

## 3. 与 mthings 互测

mthings 是 Windows 图形化 Modbus 调试工具，可当主站也可以当从站。

### 3.1 mthings 当主站，modbusmech 当从站

1. 启动从站：`modbusmech_slave.exe tcp 1502 --units 1,2 --regs 100`
2. mthings 新建 TCP 连接：IP `127.0.0.1`，端口 `1502`（**不是** 502，避免和真实设备冲突），
   从站地址 `1`；
3. 按第 0 节的表建点表：
   - 保持寄存器 `40001` 起（界面上通常是 1 基显示，对应线缆上地址 0）；
   - 输入寄存器 30001 起；
   - 线圈 00001 起；离散输入 10001 起；
4. 读一遍，核对初值表；
5. 写 `40006`（地址 5）= 1234，再用命令行 `modbusmech_master ... rh 5 1` 回读，确认写穿透到从站。

### 3.2 mthings 当从站，modbusmech 当主站

1. mthings 建立 TCP 从站（监听端口，如 1502），配置寄存器区初值（保持寄存器 0..9 = 0..9）；
2. `./build/bin/modbusmech_master tcp 127.0.0.1 1502 rh 0 10` → 期望 0..9；
3. `wr 3 999` → mthings 界面上地址 3 应变 999。

---

## 4. 判定标准（每条互测都要过）

| # | 检查项 | 期望 |
|---|---|---|
| 1 | 四类读（01/02/03/04） | 值与第 0 节初值表**逐项一致** |
| 2 | 四类写（05/06/0F/10） | 写后回读一致；跨工具回读一致 |
| 3 | 越界读/写 | 从站回异常码 **02**（`illegal data address`） |
| 4 | 不支持的 function code | 从站回异常码 **01**（`illegal function`） |
| 5 | 未知 unit（配置了 `--units`） | 静默丢弃，主站超时 |
| 6 | 报文长度边界 | 读 125 个寄存器 / 写 123 个寄存器（规范上限）正常 |
| 7 | 连续轮询 | `poll 0 10 100 50` 无失败；观察 `timeouts` 计数为 0 |
| 8 | TCP 流水线（可选） | `--inflight 4` 下并发轮询无错配（值不错位） |
| 9 | 断链恢复（可选） | 从站重启后：`--auto-reconnect` 的主站下一次请求能自动连上 |
| 10 | RTU 帧边界（RTU） | 9600/8E1 下连续 100 次读写无 CRC 错 |

---

## 5. 常见坑

1. **端口 502 需要 root/管理员**（Linux 下 <1024 端口），本地互测统一用 1502 更省事。
2. **窗口防火墙**：Windows 上从站监听后，其它机器连不进来先看防火墙。
3. **地址口径**：Modbus 报文是 **0 基**；很多工具界面按 1 基显示（40001 = 地址 0）。对表时先统一口径。
4. **RTU 参数**：波特率、数据位、校验位、停止位**必须完全一致**；Modbus 规范默认 8E1，很多设备是 8N1。
5. **libmodbus 的 error recovery**：只开 `MODBUS_ERROR_RECOVERY_PROTOCOL`，**不要**开 `_LINK`（会在 send 失败时无限重试卡死 worker 线程）——这也是 modbusmech 侧"连接策略"的由来。
6. **mthings 的从站默认只回一个 unit**：多 unit 场景要确认它是否支持按 unit 分派。
7. **同一连接并发**：libmodbus 是单事务模型，对打时不要并发；modbusmech 的 `--inflight > 1` 要单独和"支持流水线的对端"测。
