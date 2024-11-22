#ifndef __COMMON_H__
#define __COMMON_H__

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <vector>
#include <set>
#include <unordered_set>

#define PID 3.14159265359
#define PIF 3.14159265359f
#define GET_RADIAND(degree) (degree) * PID / 180.0
#define GET_RADIANF(degree) (degree) * PIF / 180.f

#ifndef ALLOCA
#if defined(_MSC_VER)
#include <malloc.h>
#define ALLOCA _alloca
#elif defined(__GNUC__) || defined(__clang__)
#include <alloca.h>
#define ALLOCA alloca
#endif
#endif

#include "allocator.h"

#if _WIN32 || _WIN64
// win32 specific function. You don't need to use these functions on another platform
void str_widen(const char* str, int strLenWithNULL, wchar_t* buffer, int bufferByteSize);
void str_narrow(const wchar_t* str, int wcsLenWithNULL, char* buffer, int bufferByteSize);
#endif

FILE* file_open(const char* utf8Path, const char* mode);
bool file_read_until_total_size(FILE* fp, int64_t total_size, void* buffer);
void file_open_fill_buffer(const char* path, std::vector<char>& buffer);
bool file_is_exist(const char* utf8_path);
void file_remove(const char* utf8_path);
bool directory_is_exist(const char* utf8_path);
void directory_create(const char* utf8_path);

template<class T> 
class TVector : public std::vector<T, StdPoolAllocator<T>>
{
public:
    TVector()
        : std::vector<T, StdPoolAllocator<T>>(StdPoolAllocator<T>())
    {}
    TVector(PoolAllocator* allocator) 
        : std::vector<T, StdPoolAllocator<T>>(StdPoolAllocator<T>(allocator))
    {}
    TVector(PoolAllocator& allocator)
        : std::vector<T, StdPoolAllocator<T>>(StdPoolAllocator<T>(&allocator))
    {}


    TVector<T>& operator=(const std::vector<T>& other)
    {
        size_t n = other.size();

        TVector::resize(n);

        for (size_t i = 0; i < n; ++i)
        {
            TVector::operator[](i) = other[i];
        }

        return *this;
    }
};

template<class K, class D, class HASH = std::hash<K>, class PRED = std::equal_to<K>>
class TUnorderedMap : public std::unordered_map<K, D, HASH, PRED, StdPoolAllocator<std::pair<const K, D>>>
{
public:
    TUnorderedMap()
        : std::unordered_map<K, D, HASH, PRED, StdPoolAllocator<std::pair<const K, D>>>(StdPoolAllocator<std::pair<const K, D>>())
    {}
    TUnorderedMap(PoolAllocator* allocator)
        : std::unordered_map<K, D, HASH, PRED, StdPoolAllocator<std::pair<const K, D>>>(StdPoolAllocator<std::pair<const K, D>>(allocator))
    {}
    TUnorderedMap(PoolAllocator& allocator)
        : std::unordered_map<K, D, HASH, PRED, StdPoolAllocator<std::pair<const K, D>>>(StdPoolAllocator<std::pair<const K, D>>(&allocator))
    {}
};

template<class K, class CMP = std::less<K>>
class TSet : public std::set<K, CMP, StdPoolAllocator<K>>
{
public:
    TSet()
        : std::set<K, CMP, StdPoolAllocator<K>>(StdPoolAllocator<K>())
    {}
    TSet(PoolAllocator* allocator)
        : std::set<K, CMP, StdPoolAllocator<K>>(StdPoolAllocator<K>(allocator))
    {}
    TSet(PoolAllocator& allocator)
        : std::set<K, CMP, StdPoolAllocator<K>>(StdPoolAllocator<K>(&allocator))
    {}
};

template<class K, class HASH = std::hash<K>, class KeyEQ = std::equal_to<K>>
class TUnorderedSet : public std::unordered_set<K, HASH, KeyEQ, StdPoolAllocator<K>>
{
public:
    TUnorderedSet()
        : std::unordered_set<K, HASH, KeyEQ, StdPoolAllocator<K>>(StdPoolAllocator<K>())
    {}
    TUnorderedSet(PoolAllocator* allocator)
        : std::unordered_set<K, HASH, KeyEQ, StdPoolAllocator<K>>(StdPoolAllocator<K>(allocator))
    {}
    TUnorderedSet(PoolAllocator& allocator)
        : std::unordered_set<K, HASH, KeyEQ, StdPoolAllocator<K>>(StdPoolAllocator<K>(&allocator))
    {}
};


#endif