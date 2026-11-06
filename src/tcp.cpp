// =============================================================================
//  tcp.cpp —— Modbus TCP 传输实现
// =============================================================================
//  解析统一在 on_bytes（io 线程）做，按 tid 分发给各自的等待者 —— 这样同一连接
//  上可以同时有多个事务在途（流水线），也允许多个线程共用同一个 Master。
//  见 tcp.hpp 开头的说明。
// =============================================================================
#include "tcp.hpp"

#include <algorithm>
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
    return static_cast<std::uint16_t>((static_cast<std::uint16_t>(p[0]) << 8) | p[1]);
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
        // ★ 只在这里（io 线程）保存连接，后续 start_read 都用**自己这份**。
        //
        //   owner->m_conn 是给调用线程（send/is_open/close/重连）用的。以前
        //   start_read 读的是 owner->m_conn，而 create()/重连在调用线程写它 ——
        //   无同步的 shared_ptr 读写是数据竞争（TSan 抓得到）。
        m_conn = std::move(conn);
        return m_conn && m_conn->read(netmech::Buf{nullptr, m_chunk.data(), m_chunk.size()});
    }

    void on_read(std::error_code ec, netmech::Buf buf, std::size_t n) override
    {
        auto owner = m_owner.lock();
        if (!owner) {
            return;
        }
        if (ec) {
            // 读出错 = 这条连接废了。只标记（允许后续按需重连），不 close()。
            owner->mark_dropped();
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
                owner->mark_dropped();
            }
        }
    }

    void on_close() override
    {
        // 连接已经关了：标记 + 唤醒所有在途等待者（它们只会拿到 not_connected）。
        if (auto owner = m_owner.lock()) {
            owner->mark_dropped();
        }
    }

    bool start_read()
    {
        return m_conn && m_conn->read(netmech::Buf{nullptr, m_chunk.data(), m_chunk.size()});
    }

    std::array<std::uint8_t, 1024> m_chunk{};

private:
    std::shared_ptr<netmech::Conn> m_conn;   ///< 本连接（io 线程内读写）
    std::weak_ptr<TcpTransport>    m_owner;
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
    std::mutex                     mtx;
    std::condition_variable        cv;
    bool                           done = false;
    std::shared_ptr<netmech::Conn> conn;
    std::error_code                err;

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
    self->m_host = host;
    self->m_port = port;
    if (!self->connect_once(ec)) {
        return nullptr;
    }
    return self;
}

bool TcpTransport::connect_once(std::error_code& ec)
{
    // 新连接：清掉上一条连接留下的状态（重连时尤其重要 —— 否则旧的半帧
    // 会被当成新连接的第一帧）。
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        m_rx.clear();
        m_alive = false;
    }

    if (!m_mech) {
        m_mech = netmech::Mechanism::create();
        if (!m_mech) {
            ec = make_error_code(ModbusErrc::io_error);
            return false;
        }
    }

    // 每次连接用一套新的 Handler/Link：旧的 Conn 可能还持有旧的 handler，
    // 复用它会让两代连接共享同一个读缓冲。
    auto handler  = std::make_shared<Handler>(shared_from_this());
    auto link     = std::make_shared<Link>();
    link->handler = handler;
    m_link        = link;   // 必须存住，否则 Mechanism 的 weak_ptr 失效

    m_mech->set_linkable(link);
    const netmech::Address addr{m_host, m_port};
    ec = m_mech->connect(addr, link,
                         static_cast<std::uint32_t>(m_connect_timeout.count()));
    if (ec) {
        return false;
    }

    // connect 是异步的：等 on_linkup / on_linkfail 通知。
    // 等一个"连接超时 + 余量"，比 netmech 自己的超时稍宽，纯粹是兜底。
    std::shared_ptr<netmech::Conn> connected;
    {
        std::unique_lock<std::mutex> lk(link->mtx);
        if (!link->cv.wait_for(lk, m_connect_timeout + std::chrono::milliseconds(2000),
                               [&] { return link->done; })) {
            ec = make_error_code(ModbusErrc::timeout);   // 建连超时：直接失败
            return false;
        }
        if (link->err) {
            ec = link->err;
            return false;
        }
        connected = link->conn;
    }
    if (!connected) {
        ec = make_error_code(ModbusErrc::timeout);
        return false;
    }

    // close() 可能和上面的等待并发：用户已经显式关了就别再装上新连接。
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (m_closed) {
            connected->close();
            ec = make_error_code(ModbusErrc::not_connected);
            return false;
        }
    }

    m_conn = std::move(connected);
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        m_alive = true;
    }
    return true;
}

bool TcpTransport::ensure_connected(std::error_code& ec)
{
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (m_closed) {
            ec = make_error_code(ModbusErrc::not_connected);
            return false;
        }
    }
    if (is_open()) {
        return true;
    }
    if (!m_auto_reconnect) {
        ec = make_error_code(ModbusErrc::not_connected);
        return false;
    }
    // 按需建连：建连**失败/超时就直接返回失败**（由调用方决定要不要再试），
    // 不在这里做重试循环 —— 短连接设备更需要的是"快速失败"，而不是拖着。
    if (!connect_once(ec)) {
        return false;
    }
    m_reconnects.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool TcpTransport::is_open() const noexcept
{
    return m_conn && m_conn->is_open();
}

void TcpTransport::on_bytes(const std::uint8_t* data, std::size_t n)
{
    struct Frame {
        std::uint16_t             tid = 0;
        std::uint8_t              unit = 0;
        std::vector<std::uint8_t> pdu;
    };
    std::vector<Frame> frames;

    {
        std::lock_guard<std::mutex> lk(m_mtx);
        m_rx.insert(m_rx.end(), data, data + n);

        for (;;) {
            if (m_rx.size() < kMbapLen) {
                break;
            }
            const std::uint16_t tid   = get_u16(m_rx.data());
            const std::uint16_t proto = get_u16(m_rx.data() + 2);
            const std::uint16_t len   = get_u16(m_rx.data() + 4);
            const std::uint8_t  unit  = m_rx[6];

            if (proto != 0) {
                // 协议 ID 不对：不是 Modbus 帧，吃掉 MBAP 头继续找下一帧
                m_rx.erase(m_rx.begin(), m_rx.begin() + kMbapLen);
                continue;
            }
            if (len < 2 || static_cast<std::size_t>(len) + 6 > Limits::max_tcp_adu) {
                // 长度字段不可信：帧边界彻底乱了，只能整段丢弃
                m_rx.clear();
                break;
            }
            const std::size_t total = 6 + len;
            if (m_rx.size() < total) {
                break;   // 还没收够
            }

            Frame f;
            f.tid  = tid;
            f.unit = unit;
            f.pdu.assign(m_rx.begin() + kMbapLen, m_rx.begin() + total);
            frames.push_back(std::move(f));
            m_rx.erase(m_rx.begin(), m_rx.begin() + total);
        }
    }

    for (Frame& f : frames) {
        dispatch_frame(f.tid, f.unit, std::move(f.pdu));
    }
}

void TcpTransport::dispatch_frame(std::uint16_t tid, std::uint8_t unit,
                                  std::vector<std::uint8_t>&& pdu)
{
    std::shared_ptr<Pending> p;
    {
        std::lock_guard<std::mutex> lk(m_pending_mtx);
        auto it = m_pending.find(tid);
        if (it == m_pending.end()) {
            return;   // 迟到的响应（该事务已超时/已放弃）→ 丢弃
        }
        p = it->second;
        m_pending.erase(it);
    }
    {
        std::lock_guard<std::mutex> lk(p->mtx);
        if (p->unit != unit) {
            p->ec = make_error_code(ModbusErrc::bad_unit_id);
        } else {
            p->pdu = std::move(pdu);
        }
        p->done = true;
    }
    p->cv.notify_all();
}

void TcpTransport::fail_all(const std::error_code& ec) noexcept
{
    std::unordered_map<std::uint16_t, std::shared_ptr<Pending>> victims;
    {
        std::lock_guard<std::mutex> lk(m_pending_mtx);
        victims.swap(m_pending);
    }
    for (auto& kv : victims) {
        const std::shared_ptr<Pending>& p = kv.second;
        {
            std::lock_guard<std::mutex> lk(p->mtx);
            p->ec   = ec;
            p->done = true;
        }
        p->cv.notify_all();
    }
}

std::error_code TcpTransport::send(span<const std::uint8_t> pdu, std::uint8_t unit,
                                   std::uint16_t& tid)
{
    tid = 0;
    std::error_code ec;
    if (!ensure_connected(ec)) {
        return ec;
    }
    if (pdu.size() > Limits::max_pdu) {
        return make_error_code(ModbusErrc::frame_too_long);
    }

    auto p = std::make_shared<Pending>();
    {
        std::lock_guard<std::mutex> lk(m_pending_mtx);
        if (m_pending.size() >= m_max_in_flight) {
            // 在途数到上限：让调用方自己决定是排队还是放弃（不在这里阻塞，
            // 否则一个慢从站会把整条连接上的调用线程全堵住）。
            return make_error_code(ModbusErrc::busy);
        }
        // 分配一个当前没在用的 tid（回绕时跳过在途的）
        for (std::size_t i = 0; i < 65536; ++i) {
            if (m_pending.find(m_next_tid) == m_pending.end()) {
                break;
            }
            ++m_next_tid;
        }
        p->tid  = m_next_tid++;
        p->unit = unit;
        m_pending.emplace(p->tid, p);
    }
    tid = p->tid;

    // 组 MBAP：tid | 0 | len | unit | pdu
    std::vector<std::uint8_t> adu;
    adu.reserve(kMbapLen + pdu.size());
    put_u16(adu, p->tid);
    put_u16(adu, 0);                                          // 协议 ID
    put_u16(adu, static_cast<std::uint16_t>(1 + pdu.size())); // 单元 ID + PDU
    adu.push_back(unit);                                      // ← 调用方传的 unit
    adu.insert(adu.end(), pdu.begin(), pdu.end());

    auto holder = std::make_shared<std::vector<std::uint8_t>>(std::move(adu));
    const std::shared_ptr<netmech::Conn> conn = m_conn;   // 本线程内的快照
    if (!conn || !conn->write(netmech::ConstBuf{holder, holder->data(), holder->size()})) {
        std::lock_guard<std::mutex> lk(m_pending_mtx);
        m_pending.erase(tid);
        return make_error_code(ModbusErrc::io_error);
    }
    return {};
}

std::error_code TcpTransport::receive(std::vector<std::uint8_t>& out,
                                      std::chrono::milliseconds timeout,
                                      std::uint8_t /*unit*/, std::uint16_t tid)
{
    out.clear();

    std::shared_ptr<Pending> p;
    {
        std::lock_guard<std::mutex> lk(m_pending_mtx);
        auto it = m_pending.find(tid);
        if (it == m_pending.end()) {
            return make_error_code(ModbusErrc::not_connected);
        }
        p = it->second;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::error_code result;
    bool            completed = false;
    {
        std::unique_lock<std::mutex> lk(p->mtx);
        completed = p->cv.wait_until(lk, deadline, [&] { return p->done; });
        if (completed) {
            result = p->ec;
            if (!result) {
                out = std::move(p->pdu);
            }
        }
    }
    if (!completed) {
        // 超时：把这一笔从在途表摘掉。之后到达的响应会因"查不到 tid"被丢弃。
        std::lock_guard<std::mutex> lk(m_pending_mtx);
        m_pending.erase(tid);
        return make_error_code(ModbusErrc::timeout);
    }
    return result;
}

void TcpTransport::close() noexcept
{
    {
        std::lock_guard<std::mutex> lk(m_mtx);
        if (m_closed) {
            return;
        }
        m_closed = true;   // 用户显式关闭：此后不再自动重连
        m_alive  = false;
    }
    fail_all(make_error_code(ModbusErrc::not_connected));
    if (m_conn) {
        m_conn->close();
    }
    if (m_mech) {
        m_mech->close_all();
    }
}

} // namespace detail
} // namespace modbusmech
