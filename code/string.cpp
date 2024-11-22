#include "string.h"

#include <stdarg.h>
#include <assert.h>

#include "allocator.h"

PoolString::PoolString(PoolAllocator* allocator)
    : _allocator(*allocator)
    , _buffer(nullptr)
    , _bufferCapacity(0)
    , _strCount(0)
{}

PoolString::PoolString(PoolAllocator& allocator)
    : _allocator(allocator)
    , _buffer(nullptr)
    , _bufferCapacity(0)
    , _strCount(0)
{}

PoolString::PoolString(PoolAllocator& allocator, int strCount)
    : _allocator(allocator)
    , _buffer(nullptr)
    , _bufferCapacity(0)
    , _strCount(0)
{
    if (strCount > 0)
    {
        _strCount = strCount;

        _bufferCapacity = 2;
        while (_bufferCapacity <= strCount)
        {
            _bufferCapacity <<= 1;
        }

        _buffer = (char*)_allocator.Alloc(_bufferCapacity);
        _buffer[strCount] = '\0';
    }
}

PoolString::~PoolString()
{
    if (_buffer != nullptr)
    {
        _allocator.Free(_buffer);
    }
}

void PoolString::Append(const char* str)
{
    int strLen = (int)strlen(str);
    if (_strCount + strLen >= _bufferCapacity)
    {
        Reserve(_strCount + strLen + 1);
    }

    memcpy(_buffer + _strCount, str, strLen);
    _strCount += strLen;
    _buffer[_strCount] = '\0';
}

void PoolString::AppendSize(const char* str, int size)
{
    if (size <= 0)
        return;

    if (_strCount + size >= _bufferCapacity)
    {
        Reserve(_strCount + size + 1);
    }

    memcpy(_buffer + _strCount, str, size);
    _strCount += size;
    _buffer[_strCount] = '\0';
}

void PoolString::AppendChar(char ch, int count)
{
    if (count <= 0)
        return;

    if (_strCount + count >= _bufferCapacity)
    {
        Reserve(_strCount + count + 1);
    }

    memset(_buffer + _strCount, ch, count);
    _strCount += count;
    _buffer[_strCount] = '\0';
}

void PoolString::AppendSPrintf(const char* fmt, ...)
{
    va_list args, writeArgs;
    va_start(args, fmt);
    va_copy(writeArgs, args);

    int lenWithNULL = vsnprintf(NULL, 0, fmt, args) + 1;
    va_end(args);

    if (_strCount + lenWithNULL >= _bufferCapacity)
    {
        Reserve(_strCount + lenWithNULL);
    }

    vsnprintf(_buffer + _strCount, lenWithNULL, fmt, writeArgs);
    va_end(writeArgs);

    _strCount += lenWithNULL - 1;
}

void PoolString::Assign(const char* str)
{
    int strLen = (int)strlen(str);
    if (strLen >= _bufferCapacity)
    {
        Reserve(strLen + 1);
    }

    memcpy(_buffer, str, strLen);
    _strCount = strLen;
    _buffer[_strCount] = '\0';
}

void PoolString::AssignSize(const char* str, int size)
{
    if (size <= 0)
        return;

    if (size >= _bufferCapacity)
    {
        Reserve(size + 1);
    }

    memcpy(_buffer, str, size);
    _strCount = size;
    _buffer[_strCount] = '\0';
}

void PoolString::AssignChar(char ch, int count)
{
    if (count <= 0)
        return;

    if (count >= _bufferCapacity)
    {
        Reserve(count + 1);
    }

    memset(_buffer, ch, count);
    _strCount = count;
    _buffer[_strCount] = '\0';
}

void PoolString::AssignSPrintf(const char* fmt, ...)
{
    va_list args, writeArgs;
    va_start(args, fmt);
    va_copy(writeArgs, args);

    int lenWithNULL = vsnprintf(NULL, 0, fmt, args) + 1;
    va_end(args);
    if (lenWithNULL >= _bufferCapacity)
    {
        Reserve(lenWithNULL);
    }

    vsnprintf(_buffer, lenWithNULL, fmt, writeArgs);

    va_end(writeArgs);

    _strCount = lenWithNULL - 1;
}

void PoolString::Reserve(int size)
{
    if (_bufferCapacity < size)
    {
        _bufferCapacity = 2;
        while (_bufferCapacity < size)
        {
            _bufferCapacity <<= 1;
        }

        _buffer = (char*)_allocator.Realloc(_buffer, _bufferCapacity);
        _buffer[_strCount] = '\0';
    }
}

void PoolString::Resize(int size)
{
    if (size < 0)
        return;

    _strCount = size;
    Reserve(size + 1);
    _buffer[_strCount] = '\0';
}

void PoolString::Clear()
{
    _strCount = 0;

    if (_bufferCapacity > 0)
    {
        _buffer[_strCount] = '\0';
    }
}

int PoolString::Size() const
{
    return _strCount;
}

char& PoolString::operator[](int index)
{
    assert(index >= 0 && index < _strCount);

    return _buffer[index];
}

char PoolString::operator[](int index) const
{
    assert(index >= 0 && index < _strCount);

    return _buffer[index];
}

char* PoolString::data()
{
    return _buffer;
}