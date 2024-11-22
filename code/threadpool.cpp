#include "threadpool.h"

ThreadPool::ThreadPool()
    : _shutdownFlag(0)
{
    _threadCount = std::thread::hardware_concurrency();
    _threads.resize(_threadCount);
    for (int i = 0; i < _threadCount; ++i)
    {
        _threads[i] = std::thread(&ThreadPool::_threadpoolWorkerFunction, this);
    }
}

ThreadPool::~ThreadPool()
{
    if (_threadCount != 0)
    {
        Join(SHUTDOWN_IMMEDIATE);
    }
}

void ThreadPool::Join(int flag)
{
    _mutex.lock();

    _shutdownFlag = flag;
    _condition.notify_all();

    _mutex.unlock();

    for (std::thread& t : _threads)
    {
        t.join();
    }

    _threads.clear();
    _threadCount = 0;
}

void ThreadPool::EnqueueJob(ThreadPoolJob f, void* argument)
{
    _mutex.lock();

    _queue.push({ f, argument });
    _condition.notify_one();

    _mutex.unlock();
}

size_t ThreadPool::GetThreadCount() const 
{
    return _threads.size(); 
}

void ThreadPool::_threadpoolWorkerFunction()
{
    ThreadPool* tp = this;

    while (true)
    {
        std::unique_lock<std::mutex> ul(tp->_mutex);

        tp->_condition.wait
        (
            ul,
            [tp]
            {
                return (tp->_queue.size() > 0 || tp->_shutdownFlag != 0);
            }
            );

        if (tp->_shutdownFlag == SHUTDOWN_IMMEDIATE ||
            (tp->_shutdownFlag == SHUTDOWN_GRACEFULLY && tp->_queue.size() == 0))
        {
            break;
        }

        QueueElement qe = tp->_queue.front();
        tp->_queue.pop();

        ul.unlock();

        qe.function(qe.argument);
    }
}