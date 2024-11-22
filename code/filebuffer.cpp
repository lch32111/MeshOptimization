#include "filebuffer.h"

#include "common.h"

void filebuffer_open(FileBuffer* fb, const char* path, const char* mode)
{
    fb->f = file_open(path, mode);
}

void filebuffer_close(FileBuffer* fb)
{
    fclose(fb->f);
}

void filebuffer_write_int(FileBuffer* fb, int v)
{
    fwrite(&v, sizeof(int), 1, fb->f);
}

void filebuffer_write_int_str(FileBuffer* fb, int v)
{
    char str[11];
    int size = sprintf(str, "%d", v);
    fwrite(str, size, 1, fb->f);
}

void filebuffer_write_float(FileBuffer* fb, float v)
{
    fwrite(&v, sizeof(float), 1, fb->f);
}

void filebuffer_write_float_str(FileBuffer* fb, float v)
{
    char str[64];
    int size = sprintf(str, "%f", v);
    fwrite(str, size, 1, fb->f);
}

void filebuffer_write_str(FileBuffer* fb, const char* str)
{
    size_t len = strlen(str);
    fwrite(str, len, 1, fb->f);
}
