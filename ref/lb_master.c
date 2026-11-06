/* =============================================================================
 *  lb_master.c —— libmodbus 3.2.0 的 TCP 主站（互测用，不是 modbusmech 的一部分）
 * =============================================================================
 *  用途：给 modbusmech_slave 示例当对端。
 *
 *  对端从站的初值必须是 docs/interop.md 的基准表：
 *      holding[i] = i、input[i] = 1000+i、coils[i] = i%4==0、discrete[i] = i%5==0
 *  所以 `modbusmech_slave tcp <port> --regs 200` 可以直接对上。
 *
 *  编译：
 *      gcc -O2 -I ref/src ref/lb_master.c ref/libmodbus.a -lpthread -lm -o /tmp/lb_master
 *  运行：
 *      /tmp/lb_master 1503
 *  退出码 0 = 全过。
 * ========================================================================== */
#include <modbus.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail = 0;

static void check(int cond, const char* what)
{
    printf("  [%s] %s\n", cond ? " ok " : "FAIL", what);
    if (!cond) {
        ++g_fail;
    }
}

int main(int argc, char** argv)
{
    const int port = argc > 1 ? atoi(argv[1]) : 1503;

    modbus_t* ctx = modbus_new_tcp("127.0.0.1", port);
    if (ctx == NULL) {
        fprintf(stderr, "modbus_new_tcp failed: %s\n", modbus_strerror(errno));
        return 2;
    }
    modbus_set_response_timeout(ctx, 2, 0);

    if (modbus_connect(ctx) != 0) {
        fprintf(stderr, "connect failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return 2;
    }
    modbus_set_slave(ctx, 1);

    printf("--- libmodbus 主站 -> modbusmech 从站 ---\n");

    /* 03 读保持寄存器：0..9 */
    {
        uint16_t regs[10] = {0};
        const int rc = modbus_read_registers(ctx, 0, 10, regs);
        check(rc == 10, "读保持寄存器 0..9");
        if (rc == 10) {
            int ok = 1;
            for (int i = 0; i < 10; ++i) {
                if (regs[i] != (uint16_t)i) {
                    ok = 0;
                }
            }
            check(ok, "值正确（holding[i] == i）");
        }
    }

    /* 04 读输入寄存器：1000..1004 */
    {
        uint16_t regs[5] = {0};
        const int rc = modbus_read_input_registers(ctx, 0, 5, regs);
        check(rc == 5, "读输入寄存器 0..4");
        check(rc == 5 && regs[0] == 1000 && regs[4] == 1004, "值正确（1000..1004）");
    }

    /* 01 读线圈：1 0 0 0 1 0 0 0（libmodbus 逐位返回） */
    {
        uint8_t bits[8] = {0};
        const int rc = modbus_read_bits(ctx, 0, 8, bits);
        check(rc == 8, "读线圈 0..7");
        check(rc == 8 && bits[0] == 1 && bits[1] == 0 && bits[4] == 1 && bits[5] == 0,
              "值正确（bit0/bit4 为 1）");
    }

    /* 02 读离散输入：1 0 0 0 0 1 ... */
    {
        uint8_t bits[10] = {0};
        const int rc = modbus_read_input_bits(ctx, 0, 10, bits);
        check(rc == 10, "读离散输入 0..9");
        check(rc == 10 && bits[0] == 1 && bits[5] == 1 && bits[1] == 0,
              "值正确（bit0/bit5 为 1）");
    }

    /* 06 写单寄存器 + 回读 */
    {
        check(modbus_write_register(ctx, 5, 1234) == 1, "写单寄存器 5 = 1234");
        uint16_t v = 0;
        check(modbus_read_registers(ctx, 5, 1, &v) == 1 && v == 1234,
              "写后回读一致");
    }

    /* 10 写多寄存器 + 回读 */
    {
        const uint16_t vals[3] = {0x1111, 0x2222, 0x3333};
        check(modbus_write_registers(ctx, 10, 3, vals) == 3, "写多寄存器 10..12");
        uint16_t back[3] = {0};
        check(modbus_read_registers(ctx, 10, 3, back) == 3 && back[0] == 0x1111
                  && back[1] == 0x2222 && back[2] == 0x3333,
              "写多寄存器后回读一致");
    }

    /* 05 写单线圈 + 回读 */
    {
        check(modbus_write_bit(ctx, 1, 1) == 1, "写单线圈 1 = 1");
        uint8_t b = 0;
        check(modbus_read_bits(ctx, 1, 1, &b) == 1 && b == 1, "写单线圈后回读一致");
    }

    /* 0F 写多线圈 + 回读 */
    {
        const uint8_t vals[4] = {0, 1, 1, 0};
        check(modbus_write_bits(ctx, 0, 4, vals) == 4, "写多线圈 0..3");
        uint8_t back[4] = {0};
        if (modbus_read_bits(ctx, 0, 4, back) == 4) {
            check(back[0] == 0 && back[1] == 1 && back[2] == 1 && back[3] == 0,
                  "写多线圈后回读一致（0,1,1,0）");
        }
    }

    /* 规范上限：读 125 个寄存器 */
    {
        uint16_t regs[125] = {0};
        check(modbus_read_registers(ctx, 0, 125, regs) == 125,
              "读 125 个寄存器（规范上限）");
    }

    /* 越界 → 异常码 02 */
    {
        uint16_t regs[1] = {0};
        const int rc = modbus_read_registers(ctx, 5000, 1, regs);
        check(rc == -1, "越界读被拒绝（返回 -1）");
        check(errno == EMBXILADD, "异常码是 illegal data address（02）");
    }

    modbus_close(ctx);
    modbus_free(ctx);

    printf("\n%s\n", g_fail == 0 ? "PASS" : "FAIL");
    return g_fail == 0 ? 0 : 1;
}
