
#ifndef __OPERATOR_NEW_H__
#define __OPERATOR_NEW_H__

#include <new>

enum class Heap {
    DEFAULT,
    DDR
};

void* operator new(std::size_t size, Heap heap);

void* operator new[](std::size_t size, Heap heap);

#endif
