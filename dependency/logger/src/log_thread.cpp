#include <sys/prctl.h>

#include "log_thread.h"
#include "logging.h"

LogThread::~LogThread()
{
    while (!m_task_stop.load() && queue_size())
    {
        usleep(100000);
    }

    stop();

    if (m_work_thread.joinable())
    {
        m_work_thread.join();
    }

    for (auto &logger : m_loggers)
    {
        logger->flush();
        logger->sync_to_disk();
    }
}

bool LogThread::start()
{
    m_task_stop.store(false); 
    m_work_thread = std::thread(&LogThread::worker_loop, this); 
    return true;
}

void LogThread::stop()
{
    m_task_stop.store(true);
    AsyncLogMsg new_log(async_log_msg_type::terminate);
    post_async_msg(std::move(new_log));
}

int64_t LogThread::set_wait_timeout_ms(int64_t timeout_ms)
{
    m_wait_timeout_ms = timeout_ms;
    return m_wait_timeout_ms;
}

int64_t LogThread::get_wait_timeout_ms()
{
    return m_wait_timeout_ms;
}

bool LogThread::add_logger(const LoggerPtr &logger)
{
    m_loggers.push_back(logger);
    return true;
}

size_t LogThread::logger_size()
{
    return m_loggers.size();
}

size_t LogThread::queue_size()
{
    return m_log_queue.size();
}

void LogThread::post_async_msg(AsyncLogMsg &&new_msg)
{
    m_log_queue.enqueue(std::move(new_msg));
}

void LogThread::worker_loop()
{
    prctl(PR_SET_NAME, "logger");

    sigset_t mask;
    sigfillset(&mask);

    int ret = pthread_sigmask(SIG_BLOCK, &mask, NULL);

    if (ret != 0)
    {
        printf("pthread_sigmask error, ret=%d\n", ret);
        return;
    }

    while (!m_task_stop.load())
    {
        process_next_msg();
    }
}

bool LogThread::process_next_msg()
{
    AsyncLogMsg log_msg;
    bool dequeued = m_log_queue.dequeue(log_msg, std::chrono::milliseconds(m_wait_timeout_ms));
    if (!dequeued)
    {
        for (auto &logger : m_loggers)
        {
            logger->flush();
            logger->sync_to_disk();
        }
        return true;
    }

    switch (log_msg.m_msg_type)
    {
    case async_log_msg_type::log:
    {
        for (auto &logger : m_loggers)
        {
            logger->write(log_msg.m_log_msg_buffer->data(), log_msg.m_log_msg_buffer->size());
        }
        return true;
    }
    case async_log_msg_type::flush:
    {
        for (auto &logger : m_loggers)
        {
            logger->flush();
        }
        return true;
    }
    case async_log_msg_type::sync_to_disk:
    {
        for (auto &logger : m_loggers)
        {
            logger->sync_to_disk();
        }
        return true;
    }
    case async_log_msg_type::terminate:
    {
        for (auto &logger : m_loggers)
        {
            logger->sync_to_disk();
        }
        m_task_stop.store(true);
        return false;
    }
    default:
        break;
    }

    return true;
}
