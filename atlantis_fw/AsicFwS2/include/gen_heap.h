/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission
 * of The LightCo.
 *
 * @file    gen_heap.h
 * @author  The LightCo
 * @version V1.0.0
 * @date    September-22-2016
 * @brief   Implements generic heap.
 *
 ******************************************************************************/


#ifndef __GEN_HEAP_H__
#define __GEN_HEAP_H__

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus
extern "C" {
#endif


/* Define the linked list structure.  This is used to link free blocks in order
of their memory address. */
typedef struct heap_block_link_s
{
	struct heap_block_link_s *pxNextFreeBlock;	/*<< The next free block in the list. */
	size_t xBlockSize;						/*<< The size of the free block. */
} heap_BlockLink_t;


typedef struct heap_attribute_s
{
	/* Create a couple of list links to mark the start and end of the list. */
    heap_BlockLink_t xStart;
    heap_BlockLink_t *pxEnd;

	/* Keeps track of the number of free bytes remaining, but says nothing about
	fragmentation. */
	size_t xFreeBytesRemaining;
	size_t xMinimumEverFreeBytesRemaining;

	size_t xByteAlignment;
    size_t xByteAlignmentMask;
    size_t uxHeapStructSize;
} heap_attribute_t;


extern heap_attribute_t default_heap_attribute;

void gen_vPortDefineHeapRegions( heap_attribute_t * heap_attribute, const HeapRegion_t * const pxHeapRegions, size_t byteAlignment );
size_t gen_xPortGetMinimumEverFreeHeapSize( heap_attribute_t * heap_attribute );
size_t gen_xPortGetFreeHeapSize( heap_attribute_t * heap_attribute );
void gen_vPortFree1(heap_attribute_t * heap_attribute, void *pv, const char *func, int line);
void *gen_pvPortMalloc1(heap_attribute_t * heap_attribute, size_t xWantedSize, const char *func, int line);

#define gen_vPortFree(heap_attribute, pv) gen_vPortFree1(heap_attribute, pv, __FUNCTION__, __LINE__)
#define gen_pvPortMalloc(heap_attribute, xWantedSize) gen_pvPortMalloc1(heap_attribute, xWantedSize, __FUNCTION__, __LINE__)

#ifdef __cplusplus
}
#endif
#endif /* __GEN_HEAP_H__ */
