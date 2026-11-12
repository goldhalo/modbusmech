// =============================================================================
//  async_master.cpp —— 异步主站实现
// =============================================================================
//  结构：
//     调用线程 ─ submit ─▶ 任务队列 ─▶ N 条工作线程（跑阻塞版 Master）
//                                        │
//                                        ▼
//                                 完成队列 ─▶ 1 条派发线程 ─▶ 用户回调
//
//  · 超时：由阻塞版 Master 自己负责（到点返回 timeout），所以回调一定会触发，
//    调用方不需要再做超时兜底。
//  · 回调串行：全部在派发线程上执行，调用方不需要给回调加锁。
//  · 流水线：工作线程数 = 传输层 max_in_flight，同一 TCP 连接上多个事务在途。
// =============================================================================
#include <modbusmech/async_master.hpp>

#include <modbusmech/master.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace modbusmech {

// =============================================================================
//  Impl
// =============================================================================

class AsyncMaster::Impl : public std::enable_shared_from_this<AsyncMaster::Impl> {
public:
    Config                  cfg;
    std::shared_ptr<Master> master;

    // ---- 任务队列（工作线程消费）----
    std::mutex                       q_mtx;
    std::condition_variable          q_cv;
    std::deque<std::function<void()>> tasks;
    /// 关闭标志。原子：is_running() 在任意线程无锁读（旧实现是普通 bool，
    /// 与 close() 的写构成数据竞争）。check-and-set 仍在 q_mtx 下，
    /// 与 submit 的入队互斥，保证"已排队的任务必定被排空"。
    std::atomic<bool>                closing{false};

    // ---- 完成队列（派发线程消费）----
    std::mutex                       c_mtx;
    std::condition_variable          c_cv;
    std::deque<std::function<void()>> completions;
    bool                             disp_stop = false;

    std::vector<std::thread> workers;
    std::thread              dispatcher;

    /// 已提交但回调还没跑完的请求数。
    std::atomic<std::size_t> pending{0};

    void start()
    {
        // ★ 强自持：线程捕获 shared_ptr<Impl>，保证 Impl 活到线程退出。
        //   这是"回调里调 close()"能安全 detach 自身的前提（见 close()）。
        auto self = shared_from_this();
        const std::size_t n = cfg.max_workers == 0 ? 1u : cfg.max_workers;
        workers.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            workers.emplace_back([self] { self->worker_loop(); });
        }
        dispatcher = std::thread([self] { self->dispatch_loop(); });
    }

    /// 把"回调"投递到派发线程。@return false 表示派发线程已停（调用方自行执行）。
    bool post(std::function<void()> fn)
    {
        {
            std::lock_guard<std::mutex> lk(c_mtx);
            if (disp_stop) {
                return false;
            }
            completions.push_back(std::move(fn));
        }
        c_cv.notify_one();
        return true;
    }

    /// 提交任务。@return false 表示已关闭（调用方需要按失败回调）。
    bool submit(std::function<void()> task)
    {
        {
            std::lock_guard<std::mutex> lk(q_mtx);
            if (closing.load(std::memory_order_acquire)) {
                return false;
            }
            tasks.push_back(std::move(task));
            pending.fetch_add(1, std::memory_order_relaxed);
        }
        q_cv.notify_one();
        return true;
    }

    void close() noexcept
    {
        {
            std::lock_guard<std::mutex> lk(q_mtx);
            if (closing.load(std::memory_order_acquire)) {
                return;
            }
            closing.store(true, std::memory_order_release);   // 不再接收新任务；已排队的继续跑完（回调仍会触发）
        }
        q_cv.notify_all();

        // 先关连接：在途请求会立刻以 not_connected 返回，队列里剩下的也一样，
        // 这样排空很快，不会让 close() 等满 N 个 timeout。
        if (master) {
            master->close();
        }

        const auto self = std::this_thread::get_id();
        for (std::thread& t : workers) {
            if (t.joinable() && t.get_id() != self) {
                t.join();
            }
        }

        {
            std::lock_guard<std::mutex> lk(c_mtx);
            disp_stop = true;
        }
        c_cv.notify_all();
        if (dispatcher.joinable() && dispatcher.get_id() != self) {
            dispatcher.join();
        }

        // ★ 自线程调用（用户在回调里调 close()）：不能 join 自己，但也
        //   绝不能留下 joinable 的线程对象 —— ~Impl 销毁 joinable 的
        //   std::thread 是 std::terminate（旧实现的 bug）。detach 自身：
        //   线程对象变 non-joinable，真实线程由 start() 的强自持
        //   shared_ptr<Impl> 保证活到循环自然结束，Impl 在最后一个
        //   退出的线程上销毁。外部调用方（~AsyncMaster）此时提前返回
        //   也安全。
        for (std::thread& t : workers) {
            if (t.joinable() && t.get_id() == self) {
                t.detach();
            }
        }
        if (dispatcher.joinable() && dispatcher.get_id() == self) {
            dispatcher.detach();
        }
    }

    /// 通用的"提交一次请求"：把阻塞调用放进工作线程，回调用派发线程执行。
    ///
    /// @param run   在工作线程执行的阻塞调用（返回 Result）
    /// @param done  完成回调（在派发线程执行，恰好一次）
    template <typename RunFn, typename DoneFn>
    void submit_request(RunFn&& run, DoneFn&& done)
    {
        // ★ done 包进 shared_ptr 由 task 与 reject 共享：submit 失败（closing）
        //   走 reject、成功走 task —— 两条路只走一条，但两个 lambda 都会被
        //   构造，done 绝不能被 move 进其中任何一个（旧实现双重移动，
        //   reject 捕获的 done 已被移空 → bad_function_call / 空指针，
        //   回调静默丢失或抛到调用线程）。
        auto shared_done = std::make_shared<std::decay_t<DoneFn>>(std::forward<DoneFn>(done));
        auto task = [this, run = std::forward<RunFn>(run), shared_done]() mutable {
            const Result r = run();
            // 完成动作：先减计数再回调。做成 std::function 是为了能"尝试投递、
            // 投递失败就地执行"——两条路只走一条，保证回调恰好一次。
            std::function<void()> finish = [this, r, shared_done]() mutable {
                pending.fetch_sub(1, std::memory_order_relaxed);
                (*shared_done)(r);
            };
            const std::function<void()> copy = finish;   // post 按值入队
            if (!post(copy)) {
                finish();   // 派发线程已停：就地回调
            }
        };
        if (!submit(std::move(task))) {
            // 已提交失败（正在关闭）：立刻按"未连接"回调。
            // 注意：submit 没成功就不会 pending++，这里也不做 pending--。
            std::function<void()> reject = [shared_done]() mutable {
                (*shared_done)(Result{make_error_code(ModbusErrc::not_connected),
                                      Exception::none});
            };
            const std::function<void()> copy = reject;
            if (!post(copy)) {
                reject();
            }
        }
    }

private:
    void worker_loop()
    {
        for (;;) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lk(q_mtx);
                q_cv.wait(lk, [&] { return !tasks.empty() || closing; });
                if (tasks.empty()) {
                    return;   // closing 且队列已空
                }
                task = std::move(tasks.front());
                tasks.pop_front();
            }
            try {
                task();
            } catch (...) {
                // task 内部已经把异常收敛成失败回调；这里只做兜底，绝不外抛。
            }
        }
    }

    void dispatch_loop()
    {
        for (;;) {
            std::function<void()> fn;
            {
                std::unique_lock<std::mutex> lk(c_mtx);
                c_cv.wait(lk, [&] { return !completions.empty() || disp_stop; });
                if (completions.empty()) {
                    return;   // 已停且没有待派发的
                }
                fn = std::move(completions.front());
                completions.pop_front();
            }
            try {
                fn();
            } catch (...) {
                // 用户回调抛异常：吞掉，不能让派发线程退出
            }
        }
    }
};

// =============================================================================
//  AsyncMaster 门面
// =============================================================================

AsyncMaster::AsyncMaster() = default;

AsyncMaster::~AsyncMaster()
{
    if (m_impl) {
        m_impl->close();
    }
}

std::shared_ptr<AsyncMaster> AsyncMaster::create_tcp(const std::string& host,
                                                     std::uint16_t port,
                                                     const Config& cfg,
                                                     std::error_code& ec,
                                                     std::shared_ptr<netmech::Runtime> runtime)
{
    ec.clear();

    Master::Config mc;
    mc.unit             = cfg.unit;
    mc.timeout          = cfg.timeout;
    mc.retries          = cfg.retries;
    mc.retry_delay      = cfg.retry_delay;
    mc.auto_reconnect   = cfg.auto_reconnect;
    mc.connect_timeout  = cfg.connect_timeout;
    mc.max_in_flight    = cfg.max_workers == 0 ? 1u : cfg.max_workers;

    auto master = Master::create_tcp(host, port, mc, ec, std::move(runtime));
    if (!master) {
        return nullptr;
    }

    auto self = std::shared_ptr<AsyncMaster>(new AsyncMaster());
    self->m_impl = std::make_shared<Impl>();
    self->m_impl->cfg    = cfg;
    self->m_impl->master = master;
    self->m_impl->start();
    return self;
}

// =============================================================================
//  提交辅助
// =============================================================================



// =============================================================================
//  读
// =============================================================================

void AsyncMaster::read_coils(std::uint8_t unit, std::uint16_t address,
                             std::uint16_t count, ReadBitsFn cb)
{
    if (!m_impl || !cb) {
        return;
    }
    auto impl = m_impl;
    auto bits = std::make_shared<std::vector<bool>>();
    impl->submit_request(
        [impl, unit, address, count, bits]() {
            return impl->master->read_coils(unit, address, count, *bits);
        },
        [cb = std::move(cb), bits](const Result& r) mutable {
            cb(r, std::move(*bits));
        });
}

void AsyncMaster::read_discrete_inputs(std::uint8_t unit, std::uint16_t address,
                                       std::uint16_t count, ReadBitsFn cb)
{
    if (!m_impl || !cb) {
        return;
    }
    auto impl = m_impl;
    auto bits = std::make_shared<std::vector<bool>>();
    impl->submit_request(
        [impl, unit, address, count, bits]() {
            return impl->master->read_discrete_inputs(unit, address, count, *bits);
        },
        [cb = std::move(cb), bits](const Result& r) mutable {
            cb(r, std::move(*bits));
        });
}

void AsyncMaster::read_holding_registers(std::uint8_t unit, std::uint16_t address,
                                         std::uint16_t count, ReadRegsFn cb)
{
    if (!m_impl || !cb) {
        return;
    }
    auto impl = m_impl;
    auto regs = std::make_shared<std::vector<std::uint16_t>>();
    impl->submit_request(
        [impl, unit, address, count, regs]() {
            return impl->master->read_holding_registers(unit, address, count, *regs);
        },
        [cb = std::move(cb), regs](const Result& r) mutable {
            cb(r, std::move(*regs));
        });
}

void AsyncMaster::read_input_registers(std::uint8_t unit, std::uint16_t address,
                                       std::uint16_t count, ReadRegsFn cb)
{
    if (!m_impl || !cb) {
        return;
    }
    auto impl = m_impl;
    auto regs = std::make_shared<std::vector<std::uint16_t>>();
    impl->submit_request(
        [impl, unit, address, count, regs]() {
            return impl->master->read_input_registers(unit, address, count, *regs);
        },
        [cb = std::move(cb), regs](const Result& r) mutable {
            cb(r, std::move(*regs));
        });
}

// =============================================================================
//  写
// =============================================================================

void AsyncMaster::write_single_coil(std::uint8_t unit, std::uint16_t address,
                                    bool value, WriteFn cb)
{
    if (!m_impl || !cb) {
        return;
    }
    auto impl = m_impl;
    impl->submit_request(
        [impl, unit, address, value]() {
            return impl->master->write_single_coil(unit, address, value);
        },
        [cb = std::move(cb)](const Result& r) mutable { cb(r); });
}

void AsyncMaster::write_single_register(std::uint8_t unit, std::uint16_t address,
                                        std::uint16_t value, WriteFn cb)
{
    if (!m_impl || !cb) {
        return;
    }
    auto impl = m_impl;
    impl->submit_request(
        [impl, unit, address, value]() {
            return impl->master->write_single_register(unit, address, value);
        },
        [cb = std::move(cb)](const Result& r) mutable { cb(r); });
}

void AsyncMaster::write_multiple_coils(std::uint8_t unit, std::uint16_t address,
                                       const std::vector<bool>& values, WriteFn cb)
{
    if (!m_impl || !cb) {
        return;
    }
    auto impl = m_impl;
    auto copy = std::make_shared<std::vector<bool>>(values);
    impl->submit_request(
        [impl, unit, address, copy]() {
            return impl->master->write_multiple_coils(unit, address, *copy);
        },
        [cb = std::move(cb)](const Result& r) mutable { cb(r); });
}

void AsyncMaster::write_multiple_registers(
    std::uint8_t unit, std::uint16_t address,
    const std::vector<std::uint16_t>& values, WriteFn cb)
{
    if (!m_impl || !cb) {
        return;
    }
    auto impl = m_impl;
    auto copy = std::make_shared<std::vector<std::uint16_t>>(values);
    impl->submit_request(
        [impl, unit, address, copy]() {
            return impl->master->write_multiple_registers(unit, address, *copy);
        },
        [cb = std::move(cb)](const Result& r) mutable { cb(r); });
}

// =============================================================================
//  状态
// =============================================================================

std::size_t AsyncMaster::pending_count() const noexcept
{
    return m_impl ? m_impl->pending.load(std::memory_order_relaxed) : 0;
}

bool AsyncMaster::is_running() const noexcept
{
    return m_impl && !m_impl->closing.load(std::memory_order_acquire);
}

void AsyncMaster::close() noexcept
{
    if (m_impl) {
        m_impl->close();
    }
}

} // namespace modbusmech
