# ref/ —— 与 libmodbus 的互通对照测试

这里放的是**验证工具**，不是 modbusmech 的一部分。

## 为什么单独放

libmodbus 是 **LGPL-2.1+**，modbusmech 是 **MIT**。把它的源码提交进本仓库
会让整个仓库变成 LGPL 衍生作品。所以：

- `ref/src/`（libmodbus 源码）、`ref/obj/`、`ref/libmodbus.a` **不进仓库**，见 `.gitignore`
- 本目录只提交**我们自己写的**测试程序

## 文件

| 文件 | 作用 |
|---|---|
| `interop_test.cpp` | 进程内双向对照：libmodbus 从站 ← 我们的 Master；我们的从站 ← libmodbus 主站。**44 项检查** |
| `lb_slave.c` | 独立的 libmodbus TCP 从站（给 `modbusmech_master` 示例当对端） |
| `lb_master.c` | 独立的 libmodbus TCP 主站（给 `modbusmech_slave` 示例当对端） |

## 准备 libmodbus

```bash
# 1. 取源码（任意 3.x）
git clone https://github.com/stephane/libmodbus.git /tmp/libmodbus
cp -r /tmp/libmodbus/src ref/src

# 2. 手写 config.h（没有 autotools 也能编）
cat > ref/src/config.h <<'CONFIG'
#define HAVE_ARPA_INET_H 1
#define HAVE_ERRNO_H 1
#define HAVE_FCNTL_H 1
#define HAVE_NETDB_H 1
#define HAVE_NETINET_IN_H 1
#define HAVE_NETINET_IP_H 1
#define HAVE_NETINET_TCP_H 1
#define HAVE_SYS_IOCTL_H 1
#define HAVE_SYS_SOCKET_H 1
#define HAVE_SYS_TIME_H 1
#define HAVE_TERMIOS_H 1
#define HAVE_UNISTD_H 1
#define HAVE_DECL_TIOCSRS485 1
#define HAVE_BUILTIN_BSWAP16 1
#define HAVE_BUILTIN_BSWAP32 1
#define HAVE_BUILTIN_BSWAP64 1
CONFIG

# 3. 编译静态库
mkdir -p /tmp/lbo
gcc -c -O2 -fPIC -I ref/src ref/src/modbus.c      -o /tmp/lbo/modbus.o
gcc -c -O2 -fPIC -I ref/src ref/src/modbus-data.c -o /tmp/lbo/modbus-data.o
gcc -c -O2 -fPIC -I ref/src ref/src/modbus-rtu.c  -o /tmp/lbo/modbus-rtu.o
gcc -c -O2 -fPIC -D_GNU_SOURCE -DIPTOS_LOWDELAY=0x10 \
        -I ref/src ref/src/modbus-tcp.c -o /tmp/lbo/modbus-tcp.o
ar rcs /tmp/lbo/libmodbus.a /tmp/lbo/*.o
```

## 跑测试

### 1) 进程内双向对照（推荐先跑这个）

```bash
g++ -std=c++17 -O1 -g -Iinclude -Isrc -Iref/src \
    -I../netmech/include -I../netmech/thirdparty/asio/include \
    ref/interop_test.cpp src/*.cpp /tmp/lbo/libmodbus.a \
    build/lib/libnetmech.a -lpthread -lm -o /tmp/interop_test
/tmp/interop_test        # 退出码 0 = 44 项全过
```

覆盖：四类读（01/02/03/04）、四类写（05/06/0F/10）+ 回读、规范上限
（读 125 个寄存器）、异常码 02、未知功能码 01、多 unit。

### 2) 示例程序 ↔ libmodbus 独立进程

```bash
# 编译两个对端
gcc -O2 -I ref/src ref/lb_slave.c  /tmp/lbo/libmodbus.a -lpthread -lm -o /tmp/lb_slave
gcc -O2 -I ref/src ref/lb_master.c /tmp/lbo/libmodbus.a -lpthread -lm -o /tmp/lb_master

# A. libmodbus 从站 ← 我们的主站示例
/tmp/lb_slave 1502 &
build/bin/modbusmech_master tcp 127.0.0.1 1502 rh 0 10
build/bin/modbusmech_master tcp 127.0.0.1 1502 wr 5 1234
build/bin/modbusmech_master tcp 127.0.0.1 1502 --inflight 4 poll 0 8 20 10
kill %1

# B. 我们的从站示例 ← libmodbus 主站
build/bin/modbusmech_slave tcp 1503 --regs 200 --quiet &
/tmp/lb_master 1503
kill %1
```

## 踩过的坑（都是 libmodbus 3.2 的 API 细节，写下来省下一次）

1. **`modbus_tcp_accept()` 成功时返回已接受的 socket fd（≥0），失败返回 -1**
   —— 不是"成功返回 0"。按 `!= 0` 判失败会不停重进 accept、**永远不读数据**，
   现象是客户端全部超时（我们的 `lb_slave.c` 就踩过这一脚）。
2. **`modbus_read_bits()` / `modbus_read_input_bits()` 返回"逐位字节"**（每元素 0/1），
   不是 Modbus 报文里的打包位图。别按 `0b0110` 去断言。
3. **`modbus_send_raw_request()` 的第一个字节是 slave id**，其后才是 PDU
   （功能码 + 数据）；`modbus_receive_confirmation()` 只返回**原始报文字节**，
   不做异常码解码（异常解码在 `check_confirmation` 里，raw 路径不经过它）。
4. **别开 `MODBUS_ERROR_RECOVERY_LINK`**：send 失败时 libmodbus 会无限重试，
   worker 线程会永久卡住（我们在 `ModbusClient` 里也踩过）。
5. 独立进程跑从站时，用 `kill -9` 收尾：`SIGTERM` 的默认 handler 带 `SA_RESTART`，
   会把阻塞在 `accept()` 的进程"重启"回去，`wait` 会一直等（本仓库的测试脚本
   就因此卡过一次）。

## 为什么这是"对照"而不是"复制"

`interop_test.cpp` / `lb_*.c` 只调用 libmodbus 的**公开 API**，不包含、不修改它的
任何源码。这是**使用一个库做测试**，不是**从它派生代码**——两者在许可证上是
不同的事。modbusmech 自己的实现是参照公开规范独立写的，见 `NOTICE`。
