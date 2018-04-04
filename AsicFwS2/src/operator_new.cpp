#include <cstdlib>
#include <cstdint>
#include "os.h"
#include "cortex_r4.h"
#include "operator_new.h"
#include "gen_heap.h"
#include "ddr_heap.h"

namespace {
    heap_attribute_t* heap_attribute[] = {
            &default_heap_attribute,
            &ddr_heap_attribute
    };
}

void* operator new(std::size_t size) {
    return operator new(size, Heap::DEFAULT);
}

void operator delete(void* p) {
    Heap heap = Heap::DEFAULT;
    if (reinterpret_cast<std::uint32_t>(p) >= DDR_BASE)
        heap = Heap::DDR;
    gen_vPortFree(heap_attribute[static_cast<std::size_t>(heap)], p);
}

void* operator new[](std::size_t size) {
    return operator new(size, Heap::DEFAULT);
}

void operator delete[](void* p) {
    operator delete(p);
}

void* operator new(std::size_t size, Heap heap) {
    return gen_pvPortMalloc(heap_attribute[static_cast<std::size_t>(heap)], size);
}

void* operator new[](std::size_t size, Heap heap) {
    return operator new(size, heap);
}
