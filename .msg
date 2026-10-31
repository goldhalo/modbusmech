test: 加与 libmodbus 的双向互通对照

两个方向都验，才算"协议实现和成熟实现一致"：

  1. libmodbus 当从站 ← 我们的 Master
     读保持寄存器 / 读输入寄存器 / 写单寄存器 / 写多寄存器

  2. 我们的从站 → libmodbus 当主站
     读保持寄存器 / 读输入寄存器 / 写单寄存器
     越界读返回 -1 且 errno == EMBXILADD（异常码 02）

实测全部通过。

许可证处理：
  libmodbus 是 LGPL-2.1+，modbusmech 是 MIT。所以
  - ref/src/（libmodbus 源码）不进仓库，已 gitignore
  - 只提交 ref/interop_test.cpp（调它的公开 API，不包含其源码）
  - ref/README.md 写清怎么搭这个对照测试

  这是"用一个库做测试"，不是"从它派生代码"，两者许可上不同。
