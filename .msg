feat: Modbus TCP 主站与从站（多客户端）

MBAP 有长度字段和事务 ID，所以比 RTU 简单：不用静默切帧，靠 length 字段
循环切就行；事务 ID 保证并发下请求/响应能配对。

- TcpTransport：组/解 MBAP，事务 ID 自增并校验，单元 ID 校验
- Master::create_tcp：主站侧客户端
- Slave::create_tcp：从站侧监听，每个客户端一个独立会话，共享 RegisterBank

调试过程中抓到两个 netmech 用法上的坑（都写进注释了）：

1. Linkable 必须存住。Mechanism 只持有 weak_ptr<Linkable>，create 里的
   局部变量一出作用域，新连接就没人接，netmech 会按"无订阅者"立即关闭。
   现象是客户端连上后立刻收到 EOF。

2. ConnHandler::on_open(conn) 要用传进来的 conn，不能依赖外部先设好的
   成员。on_linkup 只负责"返回 handler"，紧接着就调 on_open(conn)，此时
   外部成员可能还是 null，导致 read() 失败、连接被关。

另外修了 TcpTransport 的 on_close 递归（原来调 owner->close()，而 close()
里又 close_all()，会连带关掉刚建好的连接）。

验证：Linux 34 用例 170 断言，Windows 34 用例 111 断言。
