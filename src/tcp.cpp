// =============================================================================
//  tcp.cpp —— Modbus TCP 传输实现
// =============================================================================
#include "tcp.hpp"

#include <netmech/net.hpp>

#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

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
        // ★ 这里必须用传进来的 conn —— on_linkup 只负责"返回 handler"，
        //   库紧接着调 on_open(conn)，此时 TcpTransport::m_conn 可能还没设。
        //   早先的实现里 start_read() 检查 owner->m_conn（null）就返回 false，
        //   结果 netmech 直接把刚建好的连接关掉了。
        if (auto owner = m_owner.lock()) {
            owner->m_conn = conn;
        }
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

    std::atomic<netmech::Conn*> conn{nullptr};
    std::atomic<int>            err_code{0};
    std::string                 err_msg;

    std::shared_ptr<netmech::ConnHandler> on_linkup(netmech::Conn& c) override
    {
        conn.store(c.shared_from_this().get(), std::memory_order_release);
        // 注意：这里只存裸指针用于"是否有结果"的判断，
        // 真正的 shared_ptr 在下面拿到后由 TcpTransport 持有。
        conn_holder = c.shared_from_this();
        return handler;
    }

    void on_linkfail(const netmech::Address&, std::error_code e) override
    {
        err_code.store(e.value(), std::memory_order_release);
        err_msg = e.message();
    }

    std::shared_ptr<netmech::Conn> conn_holder;
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

    // connect 是异步的：等 on_linkup（conn 非空）或 on_linkfail
    for (int i = 0; i < 500; ++i) {
        if (link->conn.load(std::memory_order_acquire) != nullptr
            || link->err_code.load(std::memory_order_acquire) != 0) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (link->err_code.load(std::memory_order_acquire) != 0) {
        ec = std::make_error_code(static_cast<std::errc>(link->err_code.load()));
        return nullptr;
    }
    if (!link->conn_holder) {
        ec = make_error_code(ModbusErrc::timeout);
        return nullptr;
    }

    self->m_conn = link->conn_holder;
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

std::error_code TcpTransport::send(span<const std::uint8_t> frame)
{
    if (!is_open()) {
        return make_error_code(ModbusErrc::not_connected);
    }
    if (frame.size() > Limits::max_pdu) {
        return make_error_code(ModbusErrc::frame_too_long);
    }

    // 组 MBAP：tid | 0 | len | unit | pdu
    const std::uint16_t tid = m_next_tid++;
    m_pending_tid = tid;

    std::vector<std::uint8_t> adu;
    adu.reserve(kMbapLen + frame.size());
    put_u16(adu, tid);
    put_u16(adu, 0);                                        // 协议 ID
    put_u16(adu, static_cast<std::uint16_t>(1 + frame.size()));   // 单元 ID + PDU
    adu.push_back(m_unit);
    adu.insert(adu.end(), frame.begin(), frame.end());

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
    if (proto != 0) {
        return make_error_code(ModbusErrc::bad_protocol_id);
    }
    if (len < 2 || len > Limits::max_tcp_adu) {
        return make_error_code(ModbusErrc::bad_length);
    }
    if (tid != m_pending_tid) {
        return make_error_code(ModbusErrc::bad_transaction);
    }
    // 单元 ID 要跟请求一致（一个 TCP 连接可以访问多个单元，靠它区分）
    if (u != unit) {
        return make_error_code(ModbusErrc::bad_unit_id);
    }

    const std::size_t total = 6 + len;   // MBAP 的前 6 字节 + 长度字段说明的部分

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
