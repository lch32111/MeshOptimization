#ifndef __STRING_H__
#define __STRING_H__

class PoolAllocator;

// string only with pool allocator
// This does not support copy/move operation
class PoolString
{
public:
    PoolString() = delete;
    PoolString(const PoolString&) = delete;
    PoolString(PoolString&&) = delete;
    PoolString& operator=(const PoolString&) = delete;
    PoolString& operator=(PoolString&&) = delete;

    PoolString(PoolAllocator* allocator);
    PoolString(PoolAllocator& allocator);
    PoolString(PoolAllocator& allocator, int strCount);
    ~PoolString();

    void Append(const char* str);
    void AppendSize(const char* str, int size);
    void AppendChar(char ch, int count);
    void AppendSPrintf(const char* fmt, ...);

    void Assign(const char* str);
    void AssignSize(const char* str, int size);
    void AssignChar(char ch, int count);
    void AssignSPrintf(const char* fmt, ...);

    void Reserve(int size);
    void Resize(int size);

    void Clear();

    int Size() const;

    char& operator[](int index);
    char operator[](int index) const;

    char* data();

private:
    char* _buffer;
    int _bufferCapacity;
    int _strCount;

    PoolAllocator& _allocator;
};

#endif