#ifndef __FILE_BUFFER_H__
#define __FILE_BUFFER_H__

#include <stdio.h>

struct FileBuffer
{
    FILE* f;
};

void filebuffer_open(FileBuffer* fb, const char* path, const char* mode);
void filebuffer_close(FileBuffer* fb);

void filebuffer_write_int(FileBuffer* fb, int v);
void filebuffer_write_int_str(FileBuffer* fb, int v);
void filebuffer_write_float(FileBuffer* fb, float v);
void filebuffer_write_float_str(FileBuffer* fb, float v);
void filebuffer_write_str(FileBuffer* fb, const char* str);

#endif