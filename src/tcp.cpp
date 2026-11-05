// =============================================================================
//  tcp.cpp —— Modbus TCP 传输实现
// =============================================================================
#include "tcp.hpp"

#include <atomic>
#include <netmech/net.hpp>

#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <memory>
#include <vector>
#include <mutex>
#include <cstddef>
#include <cstdint>
#include <string>
#include <system_error>
#include <utility>

namespace modbusmech {
namespace detail {

namespace {

void put_u16(std::vector<std::uint8_t>& v, std::uint16_t x)
{
    v.push_back(static_cast<std::uint8_t>(x >> 8));
    v.push_back(static_cast<std::uint8_t>(x & 0xFF));
}

std::uint16_t get_u16(const std::uint8_t* p)
{
    return static_cast<std::uint16_t>((p[0] << 8) | p[1]);
}

} // namespace

// =============================================================================
//  Handler：netmech 的 Conn 事件
// =============================================================================

class TcpTransport::Handler final : public netmech::ConnHandler {
public:
    explicit Handler(std::shared_ptr<TcpTransport> owner) : m_owner(std::move(owner)) {}

    bool on_open(std::shared_ptr<netmech::Conn> conn) override
    {
        // ★ 只用传进来的 conn，**不要**写 owner->m_conn。
        //
        //   这里之前是 `owner->m_conn = conn`：on_open 跑在 io 线程上，而
        //   TcpTransport::create() 在**调用线程**上也会写 m_conn（等 linkup
        //   之后赋值）—— 两个线程无同步地写同一个 shared_ptr，TSan 直接报在
        //   create() 那一行上。
        //   现在 m_conn 只在 create() 里赋值一次（此时 io 线程还没开始用它），
        //   on_open/on_read 都走参数或 m_conn 的只读路径。
        return conn && conn->read(netmech::Buf{nullptr, m_chunk.data(), m_chunk.size()});
    }

    void on_read(std::error_code ec, netmech::Buf buf, std::size_t n) override
    {
        auto owner = m_owner.lock();
        if (!owner) {
            return;
        }
        if (ec) {
            owner->close();
            return;
        }
        if (n > 0) {
            owner->on_bytes(static_cast<const std::uint8_t*>(buf.data), n);
        }
        start_read();
    }

    void on_write(std::error_code ec, netmech::ConstBuf, std::size_t) override
    {
        if (ec) {
            if (auto owner = m_owner.lock()) {
                owner->close();
            }
        }
    }

    void on_close() override
    {
        // 连接已经关了，这里只标记 + 唤醒等待者。
        // 早先的实现调 owner->close()，而 close() 里又 m_mech->close_all()，
        // 会把这个刚建好的连接也关掉（还连带递归）。
        if (auto owner = m_owner.lock()) {
            owner->mark_closed();
        }
    }

    bool start_read()
    {
        auto owner = m_owner.lock();
        if (!owner || !owner->m_conn) {
            return false;
        }
        return owner->m_conn->read(netmech::Buf{nullptr, m_chunk.data(), m_chunk.size()});
    }

    std::array<std::uint8_t, 1024> m_chunk{};

private:
    std::weak_ptr<TcpTransport> m_owner;
};

// =============================================================================
//  Link：connect 的回调
// =============================================================================

class TcpTransport::Link final : public netmech::Linkable {
public:
    std::shared_ptr<netmech::ConnHandler> handler;

    /// connect 的结果（**受 mtx 保护**，不能是裸成员）。
    ///
    /// 以前这里是一个 `std::shared_ptr<Conn> conn_holder` + 两个原子标志：
    /// 原子标志自己没事，但 `conn_holder` 由 io 线程写、调用线程读，
    /// 是无同步的 shared_ptr 读写（TSan 直接报在 create() 的两行上）。
    /// 更根本的问题是**轮询**（500 × 10ms）：既慢又不精确。
    /// 现在改成 mutex + condition_variable，谁先到谁通知。
    std::mutex              mtx;
    std::condition_variable cv;
    bool                    done = false;
    std::shared_ptr<netmech::Conn> conn;
    std::error_code         err;

    std::shared_ptr<netmech::ConnHandler> on_linkup(netmech::Conn& c) override
    {
        {
            std::lock_guard<std::mutex> lk(mtx);
            conn = c.shared_from_this();
            done = true;
        }
        cv.notify_all();
        return handler;
    }

    void on_linkfail(const netmech::Address&, std::error_code e) override
    {
        {
            std::lock_guard<std::mutex> lk(mtx);
            err  = e;
            done = true;
        }
        cv.notify_all();
    }
};

// =============================================================================
//  TcpTransport
// =============================================================================

TcpTransport::TcpTransport() = default;

TcpTransport::~TcpTransport()
{
    close();
}

std::shared_ptr<TcpTransport> TcpTransport::create(const std::string& host,
                                                   std::uint16_t port,
                                                   std::error_code& ec)
{
    ec.clear();

    auto self = std::shared_ptr<TcpTransport>(new TcpTransport());

    self->m_mech = netmech::Mechanism::create();
    if (!self->m_mech) {
        ec = make_error_code(ModbusErrc::io_error);
        return nullptr;
    }

    auto handler = std::make_shared<Handler>(self);

    auto link = std::make_shared<Link>();
    link->handler = handler;
    self->m_link  = link;   // 必须存住，否则 weak_ptr 失效

    self->m_mech->set_linkable(link);
    const netmech::Address addr{host, port};
    ec = self->m_mech->connect(addr, link);
    if (ec) {
        return nullptr;
    }

    // connect 是异步的：等 on_linkup / on_linkfail 通知。
    //
    // netmech 的 connect() 自带超时（默认 3000ms，到点走 on_linkfail），
    // 这里给一个稍宽的上限纯粹是兜底 —— 正常情况下 cv 会被立刻唤醒，
    // 不再像以前那样每 10ms 醒一次、最多空转 5 秒。
    std::shared_ptr<netmech::Conn> connected;
    {
        std::unique_lock<std::mutex> lk(link->mtx);
        if (!link->cv.wait_for(lk, std::chrono::milliseconds(5000),
                               [&] { return link->done; })) {
            ec = make_error_code(ModbusErrc::timeout);
            return nullptr;
        }
        if (link->err) {
            ec = link->err;
            return nullptr;
        }
        connected = link->conn;
    }
    if (!connected) {
        ec = make_error_code(ModbusErrc::timeout);
        return nullptr;
    }

    self->m_conn = std::move(connected);
    return self;
}

bool TcpTransport::is_open() const noexcept
{
    return m_conn && m_conn->is_open();
}

void TcpTransport::on_bytes(const std::uint8_t* data, std::size_t n)
{
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        m_rx.insert(m_rx.end(), data, data + n);
    }
    m_cv.notify_all();
}

void TcpTransport::flush_input()
{
    std::lock_guard<std::mutex> lk(m_mtx);
    m_rx.clear();
}

std::error_code TcpTransport::send(span<const std::uint8_t> pdu, std::uint8_t unit)
{
    if (!is_open()) {
        return make_error_code(ModbusErrc::not_connected);
    }
    if (pdu.size() > Limits::max_pdu) {
        return make_error_code(ModbusErrc::frame_too_long);
    }

    // 组 MBAP：tid | 0 | len | unit | pdu
    const std::uint16_t tid = m_next_tid++;
    m_pending_tid = tid;

    std::vector<std::uint8_t> adu;
    adu.reserve(kMbapLen + pdu.size());
    put_u16(adu, tid);
    put_u16(adu, 0);                                          // 协议 ID
    put_u16(adu, static_cast<std::uint16_t>(1 + pdu.size())); // 单元 ID + PDU
    adu.push_back(unit);                                      // ← 用调用方传的 unit
    adu.insert(adu.end(), pdu.begin(), pdu.end());

    auto holder = std::make_shared<std::vector<std::uint8_t>>(std::move(adu));
    if (!m_conn->write(netmech::ConstBuf{holder, holder->data(), holder->size()})) {
        return make_error_code(ModbusErrc::io_error);
    }
    return {};
}

std::error_code TcpTransport::receive(std::vector<std::uint8_t>& out,
                                      std::chrono::milliseconds timeout,
                                      std::uint8_t unit)
{
    out.clear();

    const auto deadline = std::chrono::steady_clock::now() + timeout;

    // ---- 等 MBAP 头（7 字节）----
    {
        std::unique_lock<std::mutex> lk(m_mtx);
        if (!m_cv.wait_until(lk, deadline, [&] {
                return m_closed || m_rx.size() >= kMbapLen;
            })) {
            return make_error_code(ModbusErrc::timeout);
        }
        if (m_closed) {
            return make_error_code(ModbusErrc::not_connected);
        }
    }

    // ---- 解析 MBAP ----
    std::uint16_t tid = 0;
    std::uint16_t proto = 0;
    std::uint16_t len = 0;
    std::uint8_t  u = 0;
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        tid   = get_u16(m_rx.data());
        proto = get_u16(m_rx.data() + 2);
        len   = get_u16(m_rx.data() + 4);
        u     = m_rx[6];
    }
    // ★ 失败路径必须把坏帧从缓冲里**吃掉**。
    //   以前只 return 不清缓冲：下一帧会从残帧的错误偏移处开始切，
    //   于是"一帧坏、后面全乱"（bad_transaction / bad_unit_id 会一直报下去）。
    if (proto != 0) {
        // 协议 ID 不对 ⇒ len 也不可信，只吃掉 MBAP 头（7 字节已确认在缓冲里）
        std::lock_guard<std::mutex> lk(m_mtx);
        if (m_rx.size() >= kMbapLen) {
            m_rx.erase(m_rx.begin(), m_rx.begin() + static_cast<std::ptrdiff_t>(kMbapLen));
        }
        return make_error_code(ModbusErrc::bad_protocol_id);
    }
    if (len < 2 || len > Limits::max_tcp_adu) {
        // 长度非法 ⇒ 帧边界彻底不可信，只能整段丢弃
        std::lock_guard<std::mutex> lk(m_mtx);
        m_rx.clear();
        return make_error_code(ModbusErrc::bad_length);
    }

    const std::size_t total = 6 + len;   // MBAP 的前 6 字节 + 长度字段说明的部分

    if (tid != m_pending_tid) {
        // 事务号对不上：这条响应不是我们这次请求的（迟到的上一条）。
        // 长度字段可信，按整帧吃掉。
        std::lock_guard<std::mutex> lk(m_mtx);
        if (m_rx.size() >= total) {
            m_rx.erase(m_rx.begin(), m_rx.begin() + static_cast<std::ptrdiff_t>(total));
        } else {
            m_rx.clear();   // 还没收全，边界不可用；丢掉重来
        }
        return make_error_code(ModbusErrc::bad_transaction);
    }
    // 单元 ID 要跟请求一致（一个 TCP 连接可以访问多个单元，靠它区分）
    if (u != unit) {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (m_rx.size() >= total) {
            m_rx.erase(m_rx.begin(), m_rx.begin() + static_cast<std::ptrdiff_t>(total));
        } else {
            m_rx.clear();
        }
        return make_error_code(ModbusErrc::bad_unit_id);
    }

    // ---- 等够整帧 ----
    {
        std::unique_lock<std::mutex> lk(m_mtx);
        if (!m_cv.wait_until(lk, deadline, [&] {
                return m_closed || m_rx.size() >= total;
            })) {
            return make_error_code(ModbusErrc::timeout);
        }
        if (m_closed) {
            return make_error_code(ModbusErrc::not_connected);
        }
        // PDU = MBAP(7) 之后的内容
        out.assign(m_rx.begin() + kMbapLen,
                   m_rx.begin() + static_cast<std::ptrdiff_t>(total));
        m_rx.erase(m_rx.begin(), m_rx.begin() + static_cast<std::ptrdiff_t>(total));
    }

    return {};
}

void TcpTransport::close() noexcept
{
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (m_closed) {
            return;
        }
        m_closed = true;
    }
    m_cv.notify_all();
    if (m_conn) {
        m_conn->close();
    }
    if (m_mech) {
        m_mech->close_all();
    }
}

} // namespace detail
} // namespace modbusmech
