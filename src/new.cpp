// Standard new pulls in lots of exception handling code. So we
// have our own version
#include <Core.h>
#include <malloc.h>
#include <syscalls.h>
#include <cstring>

void *operator new(size_t size) noexcept
{
    void *mem = malloc(size);
    if (mem != nullptr)
        memset(mem, 0, size);
    return mem;
}

void *operator new[](size_t size) noexcept
{
    void *mem = malloc(size);
    if (mem != nullptr)
        memset(mem, 0, size);
    return mem;
}

void* operator new(size_t size, std::align_val_t align) noexcept
{
    void *mem = malloc(size);
    if (mem != nullptr)
        memset(mem, 0, size);
    return mem;
}

void *operator new[](size_t size, std::align_val_t align) noexcept
{
    void *mem = malloc(size);
    if (mem != nullptr)
        memset(mem, 0, size);
    return mem;
}

void operator delete(void * ptr) noexcept
{
        free(ptr);
}

void operator delete(void *ptr , std::size_t) noexcept
{
        free(ptr);
}

void operator delete[](void * ptr) noexcept
{
        free(ptr);
}

void operator delete(void * ptr, std::align_val_t align) noexcept
{
        free(ptr);
}


void operator delete(void *ptr , std::size_t, std::align_val_t align) noexcept
{
        free(ptr);
}

void operator delete[](void * ptr, std::align_val_t align) noexcept
{
        free(ptr);
}
