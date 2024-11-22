#include "allocator.h"

#include <assert.h>

constexpr uint64_t PoolAllocator::_poolSizes[(int)PoolSize::SizeCount];

PoolAllocator::PoolAllocator() {}
PoolAllocator::~PoolAllocator()
{
    for (int i = 0; i < (int)PoolSize::SizeCount; ++i)
    {
        for (MemoryResource mr : _freedBufferAllocations[i])
        {
            free(mr.memory.p);
        }
    }

    for (std::pair<PointerKey, MemoryResource> elem : _usedBufferAllocations)
    {
        free(elem.first.p);
    }
}

void* PoolAllocator::Alloc(uint64_t size)
{
    const int poolSize = (int)_getPoolSizeForAlloc(size);
    if (poolSize != (int)PoolSize::SizeCount)
        size = _poolSizes[(int)poolSize];

    for (int idx = 0; idx < _freedBufferAllocations[poolSize].size(); ++idx)
    {
        MemoryResource mr = _freedBufferAllocations[poolSize][idx];
        if (idx != _freedBufferAllocations[poolSize].size() - 1)
        {
            _freedBufferAllocations[poolSize][idx] = _freedBufferAllocations[poolSize].back();
        }

        _freedBufferAllocations[poolSize].pop_back();
        _usedBufferAllocations.insert(std::pair<PointerKey, MemoryResource>(mr.memory, mr));

        return mr.memory.p;
    }

    MemoryResource newMemory;
    newMemory.memory.p = (void*)malloc(size);
    newMemory.poolSize = poolSize;

    _usedBufferAllocations.insert(std::pair<PointerKey, MemoryResource>(newMemory.memory, newMemory));

    return newMemory.memory.p;
}

void* PoolAllocator::Realloc(void* mem, uint64_t newSize)
{
    void* newMemory = NULL;

    if (mem == NULL)
    {
        newMemory = Alloc(newSize);
    }
    else
    {
        PointerKey pk;
        pk.p = mem;

        std::unordered_map<PointerKey, MemoryResource>::iterator it = _usedBufferAllocations.find(pk);
        assert(it != _usedBufferAllocations.end());

        MemoryResource mr = it->second;
        if (newSize <= _poolSizes[mr.poolSize])
        {
            // we can return it again
            newMemory = mem;
        }
        else
        {
            // we need to get a new pool memory, copy the original memory into the new memory,
            // and put the original memory into a bucket
            newMemory = Alloc(newSize);
            memcpy(newMemory, mr.memory.p, _poolSizes[mr.poolSize]);

            _freedBufferAllocations[mr.poolSize].push_back(mr);

            _usedBufferAllocations.erase(it);
        }
    }

    return newMemory;
}

void PoolAllocator::Free(void* mem)
{
    PointerKey pk;
    pk.p = mem;

    std::unordered_map<PointerKey, MemoryResource>::iterator it = _usedBufferAllocations.find(pk);
    assert(it != _usedBufferAllocations.end());

    MemoryResource mr = it->second;
    _freedBufferAllocations[mr.poolSize].push_back(mr);

    _usedBufferAllocations.erase(it);
}

bool PoolAllocator::CanAllocate(uint64_t size) const
{
    constexpr uint64_t maxSize = _poolSizes[(int)(PoolSize::SizeCount)-1];
    return size <= maxSize;
}

bool PoolAllocator::IsAllocated(void* mem)
{
    PointerKey pk;
    pk.p = mem;

    std::unordered_map<PointerKey, MemoryResource>::iterator it = _usedBufferAllocations.find(pk);

    return it != _usedBufferAllocations.end();
}

uint64_t PoolAllocator::GetMemSize(void* mem)
{
    PointerKey pk;
    pk.p = mem;

    std::unordered_map<PointerKey, MemoryResource>::iterator it = _usedBufferAllocations.find(pk);

    return _poolSizes[it->second.poolSize];
}

PoolAllocator::PoolSize PoolAllocator::_getPoolSizeForAlloc(uint64_t size)
{
    PoolSize poolSize = PoolSize::SizeCount;
    for (int i = 0; i < (int)PoolSize::SizeCount; ++i)
    {
        if (_poolSizes[i] >= size)
        {
            poolSize = (PoolSize)i;
            break;
        }
    }

    assert(poolSize != PoolSize::SizeCount);

    return poolSize;
}
