#ifndef __THREAD_POOL_H__
#define __THREAD_POOL_H__

#include <vector>
#include <condition_variable>
#include <queue>

typedef void(*ThreadPoolJob)(void*);

class ThreadPool
{
public:

    enum
    {
        SHUTDOWN_IMMEDIATE = (1 << 0),
        SHUTDOWN_GRACEFULLY = (1 << 1)
    };

    ThreadPool();
    ~ThreadPool();

    void Join(int flag);
    void EnqueueJob(ThreadPoolJob f, void* argument);
    size_t GetThreadCount() const;

private:

    struct QueueElement
    {
        ThreadPoolJob function;
        void* argument;
    };

    int _shutdownFlag;

    int _threadCount;
    std::vector<std::thread> _threads;
    std::queue<QueueElement> _queue;

    std::mutex _mutex;
    std::condition_variable _condition;

    void _threadpoolWorkerFunction();
};

#endif