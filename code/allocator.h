#ifndef __ALLOCATOR_H__
#define __ALLOCATOR_H__

#include <stdint.h>
#include <vector>
#include <unordered_map>

struct PointerKey
{
    void* p;

    bool operator==(const PointerKey& o) const
    {
        return p == o.p;
    }
};

struct StringKey
{
    StringKey(const char* s) : str(s) {}

    const char* str;

    bool operator==(const StringKey& o) const
    {
        return strcmp(str, o.str) == 0;
    }
};

namespace std
{
    // From UE4
    inline uint32_t HashCombine(uint32_t A, uint32_t C)
    {
        uint32_t B = 0x9e3779b9;
        A += B;

        A -= B; A -= C; A ^= (C >> 13);
        B -= C; B -= A; B ^= (A << 8);
        C -= A; C -= B; C ^= (B >> 13);
        A -= B; A -= C; A ^= (C >> 12);
        B -= C; B -= A; B ^= (A << 16);
        C -= A; C -= B; C ^= (B >> 5);
        A -= B; A -= C; A ^= (C >> 3);
        B -= C; B -= A; B ^= (A << 10);
        C -= A; C -= B; C ^= (B >> 15);

        return C;
    }

    template<>
    struct hash<PointerKey>
    {
        size_t operator()(const PointerKey key) const
        {
            uint64_t ptrInt = reinterpret_cast<uint64_t>(key.p) >> 4;
            return HashCombine((uint32_t)ptrInt, 0);
        }
    };

    template<>
    struct hash<StringKey>
    {
        size_t operator()(StringKey key) const
        {
            size_t hash = 5381;
            int c;

            while (c = *(key.str)++)
            {
                hash = ((hash << 5) + hash) + c;
            }

            return hash;
        }
    };
}

class PoolAllocator
{
public:

    PoolAllocator();
    ~PoolAllocator();

    void* Alloc(uint64_t size);
    void* Realloc(void* mem, uint64_t newSize);
    void Free(void* mem);

    bool CanAllocate(uint64_t size) const;
    bool IsAllocated(void* mem);
    uint64_t GetMemSize(void* mem);
    constexpr uint64_t GetMaxAllocSize() const
    {
        return _poolSizes[(int)(PoolSize::SizeCount)-1];
    }

private:

    struct MemoryResource
    {
        PointerKey memory;
        int poolSize;
    };

    enum class PoolSize
    {
        E32,
        E64,
        E128,
        E256,
        E512,
        E1024,
        E2K,
        E8K,
        E16K,
        E64K,
        E256K,
        E512K,
        E1024K,
        E4096K,
        SizeCount
    };

    constexpr static uint64_t _poolSizes[(int)PoolSize::SizeCount] =
    {
        32,
        64,
        128,
        256,
        512,
        1024,
        2 * 1024,
        8 * 1024,
        16 * 1024,
        64 * 1024,
        256 * 1024,
        512 * 1024,
        1024 * 1024,
        4096 * 1024
    };

    PoolSize _getPoolSizeForAlloc(uint64_t size);

    std::vector<MemoryResource> _freedBufferAllocations[(int)PoolSize::SizeCount];
    std::unordered_map<PointerKey, MemoryResource> _usedBufferAllocations;
};

template <class T>
struct StdPoolAllocator
{
public:
    typedef T value_type;

    StdPoolAllocator()
        : real(NULL)
    {}

    ~StdPoolAllocator()
    {
        real = NULL;
    }

    StdPoolAllocator(PoolAllocator* pa)
        : real(pa)
    {}

    StdPoolAllocator(const StdPoolAllocator& o)
        : real(o.real)
    {}

    StdPoolAllocator(StdPoolAllocator&& o)
        : real(o.real)
    {}

    template<class U>
    StdPoolAllocator(const StdPoolAllocator<U>& o)
        : real(o.real)
    {}

    template<class U>
    StdPoolAllocator(StdPoolAllocator<U>&& o)
        : real(o.real)
    {}

    StdPoolAllocator& operator=(const StdPoolAllocator& o)
    {
        real = o.real;
        return *this;
    }
    
    template<class U>
    StdPoolAllocator& operator=(const StdPoolAllocator<U>& o)
    {
        real = o.real;
        return *this;
    }

    template<class U> bool operator==(const StdPoolAllocator<U>&) const noexcept
    {
        return true;
    }
    template<class U> bool operator!=(const StdPoolAllocator<U>&) const noexcept
    {
        return false;
    }

    T* allocate(const size_t n) const
    {
        if (n == 0)
        {
            return nullptr;
        }
        if (n > static_cast<size_t>(-1) / sizeof(T))
        {
            throw std::bad_array_new_length();
        }

        if (real)
        {
            void* const pv = real->Alloc(n * sizeof(T));
            if (!pv) { throw std::bad_alloc(); }
            return static_cast<T*>(pv);
        }
        else
        {
            void* const pv = malloc(n * sizeof(T));
            if (!pv) { throw std::bad_alloc(); }
            return static_cast<T*>(pv);
        }
    }

    void deallocate(T* const p, size_t) const noexcept
    {
        if (real)
        {
            real->Free(p);
        }
        else
        {
            free(p);
        }
    }

    PoolAllocator* real;
};

#endif