# ref/ —— 与 libmodbus 的互通对照测试

这里放的是**验证工具**，不是 modbusmech 的一部分。

## 为什么单独放

libmodbus 是 **LGPL-2.1+**，modbusmech 是 **MIT**。把它的源码提交进本仓库
会让整个仓库变成 LGPL 衍生作品。所以：

- `ref/src/`（libmodbus 源码）**不进仓库**，见 `.gitignore`
- 本目录只提交 `interop_test.cpp`（我们自己写的测试程序）

## 怎么跑

```bash
# 1. 取 libmodbus 源码（任意版本，下面以 3.1.10 为例）
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

# 3. 编译 libmodbus 静态库
mkdir -p ref/obj
gcc -c -O2 -fPIC -I ref/src ref/src/modbus.c      -o ref/obj/modbus.o
gcc -c -O2 -fPIC -I ref/src ref/src/modbus-data.c -o ref/obj/modbus-data.o
gcc -c -O2 -fPIC -I ref/src ref/src/modbus-rtu.c  -o ref/obj/modbus-rtu.o
gcc -c -O2 -fPIC -D_GNU_SOURCE -DIPTOS_LOWDELAY=0x10 \
        -I ref/src ref/src/modbus-tcp.c -o ref/obj/modbus-tcp.o
ar rcs ref/libmodbus.a ref/obj/*.o

# 4. 编译并跑互通测试
g++ -std=c++17 -O2 -Iinclude -Isrc -Iref/src \
    -I../netmech/include -I../netmech/thirdparty/asio/include \
    ref/interop_test.cpp src/*.cpp ref/libmodbus.a \
    build/lib/libnetmech.a -lpthread -o ref/interop_test
./ref/interop_test
```

## 测什么

两个方向都测：

| 方向 | 内容 |
|---|---|
| **libmodbus 当从站 ← 我们的 Master** | 读保持/输入寄存器、写单/多寄存器 |
| **我们的从站 → libmodbus 当主站** | 读保持/输入寄存器、写单寄存器、越界返回异常码 |

两个方向都通过，才说明"我们的协议实现和成熟实现一致"。

## 为什么这是"对照"而不是"复制"

`interop_test.cpp` 只调用 libmodbus 的**公开 API**（`modbus_new_tcp` /
`modbus_read_registers` / `modbus_reply` 等），不包含、不修改它的任何源码。

这是**使用一个库做测试**，不是**从它派生代码**——两者在许可证上是不同的事。
modbusmech 自己的实现是参照公开规范独立写的，见 `NOTICE`。
