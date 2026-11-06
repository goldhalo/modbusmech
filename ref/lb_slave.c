/* =============================================================================
 *  lb_slave.c —— libmodbus 3.2.0 的 TCP 从站（互测用，不是 modbusmech 的一部分）
 * =============================================================================
 *  用途：给 modbusmech_master 示例当对端。
 *
 *  初值（与 docs/interop.md 的基准表一致）：
 *      holding[i]   = i
 *      input[i]     = 1000 + i
 *      coils[i]     = (i % 4 == 0)
 *      discrete[i]  = (i % 5 == 0)
 *  区大小 200（够读 125 个寄存器的规范上限）。
 *
 *  编译：
 *      gcc -O2 -I ref/src ref/lb_slave.c ref/libmodbus.a -lpthread -lm -o /tmp/lb_slave
 *  运行：
 *      /tmp/lb_slave 1502
 * ========================================================================== */
#include <modbus.h>

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

int main(int argc, char** argv)
{
    const int port = argc > 1 ? atoi(argv[1]) : 1502;

    modbus_t* ctx = modbus_new_tcp("0.0.0.0", port);
    if (ctx == NULL) {
        fprintf(stderr, "modbus_new_tcp failed: %s\n", modbus_strerror(errno));
        return 1;
    }

    /* LB_DEBUG=1 打印收发帧（互测排障用） */
    if (getenv("LB_DEBUG") != NULL) {
        modbus_set_debug(ctx, 1);
    }

    modbus_mapping_t* mb = modbus_mapping_new(200, 200, 200, 200);
    if (mb == NULL) {
        fprintf(stderr, "modbus_mapping_new failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return 1;
    }
    for (int i = 0; i < 200; ++i) {
        mb->tab_registers[i]       = (uint16_t)i;
        mb->tab_input_registers[i] = (uint16_t)(1000 + i);
        mb->tab_bits[i]            = (i % 4 == 0) ? 1 : 0;
        mb->tab_input_bits[i]      = (i % 5 == 0) ? 1 : 0;
    }

    int s = modbus_tcp_listen(ctx, 1);
    if (s < 0) {
        fprintf(stderr, "modbus_tcp_listen failed: %s\n", modbus_strerror(errno));
        modbus_mapping_free(mb);
        modbus_free(ctx);
        return 1;
    }

    signal(SIGINT, on_sigint);
    signal(SIGTERM, on_sigint);

    printf("libmodbus TCP slave listening on 0.0.0.0:%d\n", port);
    fflush(stdout);

    uint8_t query[MODBUS_TCP_MAX_ADU_LENGTH];
    while (!g_stop) {
        /* ★ libmodbus 3.2 的 modbus_tcp_accept 成功时返回**已接受的 socket fd**
           （>= 0），失败返回 -1 —— 不是"成功返回 0"。按 != 0 判失败会不停重进
           accept、永远不读数据（表现为客户端全部超时）。 */
        if (modbus_tcp_accept(ctx, &s) < 0) {
            continue;   /* 出错/被信号打断：继续等下一个连接 */
        }
        for (;;) {
            const int rc = modbus_receive(ctx, query);
            if (rc > 0) {
                modbus_reply(ctx, query, rc, mb);
            } else if (rc == -1) {
                break;   /* 连接断开或空闲超时：回到 accept */
            }
        }
    }

    close(s);
    modbus_mapping_free(mb);
    modbus_free(ctx);
    return 0;
}
