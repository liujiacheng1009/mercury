#pragma once

#include <atomic>
#include <condition_variable>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

class Logger;

using LoggerPtr = std::shared_ptr<Logger>;

/**
 * @brief BufferQueue
 *
 * @tparam T
 */
template <typename T>
class BufferQueue
{
public:
    explicit BufferQueue()
    {
    }

    virtual ~BufferQueue()
    {
    }

    /**
     * @brief push a new item to the queue, movable only
     *
     * @param item
     */
    void enqueue(T &&item)
    {
        {
            std::unique_lock<std::mutex> lock(m_queue_mutex);
            m_queue.push_back(std::move(item));
        }
        m_push_cv.notify_one();
    }

    /**
     * @brief try to dequeue item. if no item found, wait upto timeout and try again
     *
     * @param popped_item
     * @param wait_duration
     * @return true if succeeded dequeue item
     */
    bool dequeue(T &popped_item, std::chrono::milliseconds wait_duration)
    {
        {
            std::unique_lock<std::mutex> lock(m_queue_mutex);
            if (!m_push_cv.wait_for(lock, wait_duration, [this]
                                    { return !this->m_queue.empty(); }))
            {
                return false;
            }
            popped_item = std::move(m_queue.front());
            m_queue.pop_front();
        }
        m_pop_cv.notify_one();
        return true;
    }

    /**
     * @brief get queue item size
     *
     * @return size_t
     */
    size_t size()
    {
        std::unique_lock<std::mutex> lock(m_queue_mutex);
        return m_queue.size();
    }

private:
    std::mutex m_queue_mutex;
    std::condition_variable m_push_cv;
    std::condition_variable m_pop_cv;
    std::list<T> m_queue;
};

enum class async_log_msg_type
{
    log,
    flush,
    sync_to_disk,
    terminate
};

using AsyncLogMsgItem = std::shared_ptr<std::string>;

/**
 * @brief Async msg to move to/from a queue.
 * Movable only, should never be copied
 *
 */
class AsyncLogMsg
{
public:
    AsyncLogMsg() = default;
    ~AsyncLogMsg() = default;

    // should only be moved in or out of the queue..
    AsyncLogMsg(const AsyncLogMsg &) = delete;
    AsyncLogMsg(AsyncLogMsg &&) = default;
    AsyncLogMsg &operator=(AsyncLogMsg &&) = default;

    /**
     * @brief Construct a new AsyncLogMsg from log_msg with given type
     *
     * @param m
     * @param the_type
     */
    AsyncLogMsg(const AsyncLogMsgItem &msg, async_log_msg_type the_type)
        : m_log_msg_buffer{msg}, m_msg_type{the_type}
    {
    }

    /**
     * @brief Construct a new AsyncLogMsg with given type and empty msg
     *
     * @param the_type
     */
    AsyncLogMsg(async_log_msg_type the_type)
        : m_msg_type{the_type}
    {
    }

    AsyncLogMsgItem m_log_msg_buffer;
    async_log_msg_type m_msg_type{async_log_msg_type::log};
};

using LogBufferQueue = BufferQueue<AsyncLogMsg>;

/**
 * @brief ASync LogThread
 *
 */
class LogThread
{
public:
    LogThread() = default;
    virtual ~LogThread();
    LogThread(const LogThread &) = delete;
    LogThread(LogThread &&) = default;
    LogThread &operator=(LogThread &&) = default;

    /**
     * @brief start worker thread
     *
     * @return true if successed
     */
    bool start();

    /**
     * @brief stop the worker thread
     *
     */
    void stop();

    /**
     * @brief Set the wait timeout in milliseconds
     *
     * @param timeout_ms
     * @return int64_t
     */
    int64_t set_wait_timeout_ms(int64_t timeout_ms);

    /**
     * @brief Get the wait timeout in milliseconds
     *
     * @return int64_t
     */
    int64_t get_wait_timeout_ms();

    /**
     * @brief add new logger to thread
     *
     * @param logger
     * @return true if successed
     */
    bool add_logger(const LoggerPtr &logger);

    /**
     * @brief post new log message
     *
     * @param new_msg
     */
    void post_async_msg(AsyncLogMsg &&new_msg);

    /**
     * @brief get logger's count
     *
     * @return size_t
     */
    size_t logger_size();

    /**
     * @brief get queue's item count
     *
     * @return size_t
     */
    size_t queue_size();

private:
    /**
     * @brief main worker thread loop
     *
     */
    void worker_loop();

    /**
     * @brief process messages
     *
     * @return true
     * @return false
     */
    bool process_next_msg();

    std::thread m_work_thread;
    LogBufferQueue m_log_queue;
    std::vector<LoggerPtr> m_loggers;
    std::atomic_bool m_task_stop{true};
    int64_t m_wait_timeout_ms = 1000;
};
