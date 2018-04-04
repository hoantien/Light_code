/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
 * A sample implementation of pvPortMalloc() that allows the heap to be defined
 * across multiple non-contigous blocks and combines (coalescences) adjacent
 * memory blocks as they are freed.
 *
 * See heap_1.c, heap_2.c, heap_3.c and heap_4.c for alternative
 * implementations, and the memory management pages of http://www.FreeRTOS.org
 * for more information.
 *
 * Usage notes:
 *
 * vPortDefineHeapRegions() ***must*** be called before pvPortMalloc().
 * pvPortMalloc() will be called if any task objects (tasks, queues, event
 * groups, etc.) are created, therefore vPortDefineHeapRegions() ***must*** be
 * called before any other objects are defined.
 *
 * vPortDefineHeapRegions() takes a single parameter.  The parameter is an array
 * of HeapRegion_t structures.  HeapRegion_t is defined in portable.h as
 *
 * typedef struct HeapRegion
 * {
 *	uint8_t *pucStartAddress; << Start address of a block of memory that will be part of the heap.
 *	size_t xSizeInBytes;	  << Size of the block of memory.
 * } HeapRegion_t;
 *
 * The array is terminated using a NULL zero sized region definition, and the
 * memory regions defined in the array ***must*** appear in address order from
 * low address to high address.  So the following is a valid example of how
 * to use the function.
 *
 * HeapRegion_t xHeapRegions[] =
 * {
 * 	{ ( uint8_t * ) 0x80000000UL, 0x10000 }, << Defines a block of 0x10000 bytes starting at address 0x80000000
 * 	{ ( uint8_t * ) 0x90000000UL, 0xa0000 }, << Defines a block of 0xa0000 bytes starting at address of 0x90000000
 * 	{ NULL, 0 }                << Terminates the array.
 * };
 *
 * vPortDefineHeapRegions( xHeapRegions ); << Pass the array into vPortDefineHeapRegions().
 *
 * Note 0x80000000 is the lower address so appears in the array first.
 *
 */
#include <stdlib.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "gen_heap.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

/* Assumes 8bit bytes! */
#define gen_heapBITS_PER_BYTE		( ( size_t ) 8 )


/*-----------------------------------------------------------*/

heap_attribute_t default_heap_attribute;

size_t xPortGetMinimumEverFreeHeapSize()
{
    return gen_xPortGetMinimumEverFreeHeapSize(&default_heap_attribute);
}

size_t xPortGetFreeHeapSize()
{
    return gen_xPortGetFreeHeapSize(&default_heap_attribute);
}

void vPortFree1(void *pv, const char *file, int line)
{
    gen_vPortFree1(&default_heap_attribute, pv, file, line);
}

void *pvPortMalloc1(size_t xWantedSize, const char *file, int line)
{
    return gen_pvPortMalloc1(&default_heap_attribute, xWantedSize, file, line);
}

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void prvInsertBlockIntoFreeList(heap_attribute_t * heap_attribute, heap_BlockLink_t *pxBlockToInsert );

/*-----------------------------------------------------------*/

/* Gets set to the top bit of an size_t type.  When this bit in the xBlockSize
member of an BlockLink_t structure is set then the block belongs to the
application.  When the bit is free the block is still part of the free heap
space. */
static const size_t xBlockAllocatedBit = ( ( size_t ) 1 ) << ( ( sizeof( size_t ) * gen_heapBITS_PER_BYTE ) - 1 );

/*-----------------------------------------------------------*/

void *gen_pvPortMalloc1(heap_attribute_t * heap_attribute, size_t xWantedSize, const char *func, int line)
{
	heap_BlockLink_t *pxBlock = NULL, *pxPreviousBlock, *pxNewBlockLink;
	void *pvReturn = NULL;

	/* The heap must be initialised before the first call to
	prvPortMalloc(). */
	configASSERT( heap_attribute->pxEnd );

	vTaskSuspendAll();
	{
		/* Check the requested block size is not so large that the top bit is
		set.  The top bit of the block size member of the BlockLink_t structure
		is used to determine who owns the block - the application or the
		kernel, so it must be free. */
		if( ( xWantedSize & xBlockAllocatedBit ) == 0 )
		{
			/* The wanted size is increased so it can contain a BlockLink_t
			structure in addition to the requested amount of bytes. */
			if( xWantedSize > 0 )
			{
				xWantedSize += heap_attribute->uxHeapStructSize;

				/* Ensure that blocks are always aligned to the required number
				of bytes. */
				if( ( xWantedSize & heap_attribute->xByteAlignmentMask ) != 0x00 )
				{
					/* Byte alignment required. */
					xWantedSize += ( heap_attribute->xByteAlignment - ( xWantedSize & heap_attribute->xByteAlignmentMask ) );
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}

			if( ( xWantedSize > 0 ) && ( xWantedSize <= heap_attribute->xFreeBytesRemaining ) )
			{
				/* Traverse the list from the start	(lowest address) block until
				one	of adequate size is found. */
				pxPreviousBlock = &heap_attribute->xStart;
				pxBlock = heap_attribute->xStart.pxNextFreeBlock;
				while( ( pxBlock->xBlockSize < xWantedSize ) && ( pxBlock->pxNextFreeBlock != NULL ) )
				{
					pxPreviousBlock = pxBlock;
					pxBlock = pxBlock->pxNextFreeBlock;
				}

				/* If the end marker was reached then a block of adequate size
				was	not found. */
				if( pxBlock != heap_attribute->pxEnd )
				{
					/* Return the memory space pointed to - jumping over the
					BlockLink_t structure at its start. */
					pvReturn = ( void * ) ( ( ( uint8_t * ) pxPreviousBlock->pxNextFreeBlock ) + heap_attribute->uxHeapStructSize );

					/* This block is being returned for use so must be taken out
					of the list of free blocks. */
					pxPreviousBlock->pxNextFreeBlock = pxBlock->pxNextFreeBlock;

					/* If the block is larger than required it can be split into
					two. */
					if( ( pxBlock->xBlockSize - xWantedSize ) > 2 * heap_attribute->uxHeapStructSize )
					{
						/* This block is to be split into two.  Create a new
						block following the number of bytes requested. The void
						cast is used to prevent byte alignment warnings from the
						compiler. */
						pxNewBlockLink = ( void * ) ( ( ( uint8_t * ) pxBlock ) + xWantedSize );

						/* Calculate the sizes of two blocks split from the
						single block. */
						pxNewBlockLink->xBlockSize = pxBlock->xBlockSize - xWantedSize;
						pxBlock->xBlockSize = xWantedSize;

						/* Insert the new block into the list of free blocks. */
						prvInsertBlockIntoFreeList(heap_attribute, ( pxNewBlockLink ) );
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					heap_attribute->xFreeBytesRemaining -= pxBlock->xBlockSize;

					if( heap_attribute->xFreeBytesRemaining < heap_attribute->xMinimumEverFreeBytesRemaining )
					{
						heap_attribute->xMinimumEverFreeBytesRemaining = heap_attribute->xFreeBytesRemaining;
					}
					else
					{
						mtCOVERAGE_TEST_MARKER();
					}

					/* The block is being returned - it is allocated and owned
					by the application and has no "next" block. */
					pxBlock->xBlockSize |= xBlockAllocatedBit;
					pxBlock->pxNextFreeBlock = NULL;
				}
				else
				{
					mtCOVERAGE_TEST_MARKER();
				}
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}

		traceMALLOC( pvReturn, xWantedSize );
	}
	( void ) xTaskResumeAll();

	#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			printf("\r\nError in gen_pvPortMalloc() when called from %s [%d]. "
			       "Wanted size: %u, first free block addr: %x, size: %u, total free size: %u\r\n",
			        func, line, (unsigned int)xWantedSize, (unsigned int)(heap_attribute->xStart.pxNextFreeBlock),
			        (unsigned int)(heap_attribute->xStart.pxNextFreeBlock->xBlockSize), (unsigned int)(heap_attribute->xFreeBytesRemaining));
			vApplicationMallocFailedHook();
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
	#endif

	return pvReturn;
}
/*-----------------------------------------------------------*/

void gen_vPortFree1(heap_attribute_t * heap_attribute, void *pv, const char *func, int line)
{
    uint8_t *puc = ( uint8_t * ) pv;
    heap_BlockLink_t *pxLink;

	if( pv != NULL )
	{
		/* The memory being freed will have an BlockLink_t structure immediately
		before it. */
		puc -= heap_attribute->uxHeapStructSize;

		/* This casting is to keep the compiler from issuing warnings. */
		pxLink = ( void * ) puc;

		if ((( pxLink->xBlockSize & xBlockAllocatedBit ) == 0 ) || (pxLink->pxNextFreeBlock != NULL))
		{
            printf("\r\nError in gen_vPortFree() when called from %s [%d]. p to free: %x, xBlockSize: %x, next_free_p: %x\r\n",
                    func, line, (int)pv, (int)pxLink->xBlockSize, (int)pxLink->pxNextFreeBlock);
		}
		configASSERT( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 );
		configASSERT( pxLink->pxNextFreeBlock == NULL );

		if( ( pxLink->xBlockSize & xBlockAllocatedBit ) != 0 )
		{
			if( pxLink->pxNextFreeBlock == NULL )
			{
				/* The block is being returned to the heap - it is no longer
				allocated. */
				pxLink->xBlockSize &= ~xBlockAllocatedBit;

				vTaskSuspendAll();
				{
					/* Add this block to the list of free blocks. */
					heap_attribute->xFreeBytesRemaining += pxLink->xBlockSize;
					traceFREE( pv, pxLink->xBlockSize );
					prvInsertBlockIntoFreeList(heap_attribute, ( ( heap_BlockLink_t * ) pxLink ) );
				}
				( void ) xTaskResumeAll();
			}
			else
			{
				mtCOVERAGE_TEST_MARKER();
			}
		}
		else
		{
			mtCOVERAGE_TEST_MARKER();
		}
	}
}
/*-----------------------------------------------------------*/

size_t gen_xPortGetFreeHeapSize( heap_attribute_t * heap_attribute )
{
	return heap_attribute->xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

size_t gen_xPortGetMinimumEverFreeHeapSize( heap_attribute_t * heap_attribute )
{
	return heap_attribute->xMinimumEverFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

static void prvInsertBlockIntoFreeList(heap_attribute_t * heap_attribute, heap_BlockLink_t *pxBlockToInsert )
{
	heap_BlockLink_t *pxIterator;
	uint8_t *puc;

	/* Iterate through the list until a block is found that has a higher address
	than the block being inserted. */
	for( pxIterator = &(heap_attribute->xStart); pxIterator->pxNextFreeBlock < pxBlockToInsert; pxIterator = pxIterator->pxNextFreeBlock )
	{
		/* Nothing to do here, just iterate to the right position. */
	}

	/* Do the block being inserted, and the block it is being inserted after
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxIterator;
	if( ( puc + pxIterator->xBlockSize ) == ( uint8_t * ) pxBlockToInsert )
	{
		pxIterator->xBlockSize += pxBlockToInsert->xBlockSize;
		pxBlockToInsert = pxIterator;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}

	/* Do the block being inserted, and the block it is being inserted before
	make a contiguous block of memory? */
	puc = ( uint8_t * ) pxBlockToInsert;
	if( ( puc + pxBlockToInsert->xBlockSize ) == ( uint8_t * ) pxIterator->pxNextFreeBlock )
	{
		if( pxIterator->pxNextFreeBlock != heap_attribute->pxEnd )
		{
			/* Form one big block from the two blocks. */
			pxBlockToInsert->xBlockSize += pxIterator->pxNextFreeBlock->xBlockSize;
			pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock->pxNextFreeBlock;
		}
		else
		{
			pxBlockToInsert->pxNextFreeBlock = heap_attribute->pxEnd;
		}
	}
	else
	{
		pxBlockToInsert->pxNextFreeBlock = pxIterator->pxNextFreeBlock;
	}

	/* If the block being inserted plugged a gab, so was merged with the block
	before and the block after, then it's pxNextFreeBlock pointer will have
	already been set, and should not be set here as that would make it point
	to itself. */
	if( pxIterator != pxBlockToInsert )
	{
		pxIterator->pxNextFreeBlock = pxBlockToInsert;
	}
	else
	{
		mtCOVERAGE_TEST_MARKER();
	}
}
/*-----------------------------------------------------------*/

void gen_vPortDefineHeapRegions(heap_attribute_t * heap_attribute, const HeapRegion_t * const pxHeapRegions, size_t byteAlignment )
{
	heap_BlockLink_t *pxFirstFreeBlockInRegion = NULL, *pxPreviousFreeBlock;
    uint8_t *pucAlignedHeap;
    size_t xTotalRegionSize, xTotalHeapSize = 0;
    BaseType_t xDefinedRegions = 0;
    uint32_t ulAddress;
    const HeapRegion_t *pxHeapRegion;

	/* Can only call once! */
	configASSERT( heap_attribute->pxEnd == NULL );

	configASSERT((byteAlignment % portBYTE_ALIGNMENT) == 0);
	configASSERT((byteAlignment & (byteAlignment - 1)) == 0); /* requested alignment must be power of 2 */
    /* The size of the structure placed at the beginning of each allocated memory block must by correctly byte aligned. */
	size_t byteAlignmentMask = byteAlignment - 1;
    heap_attribute->xByteAlignment = byteAlignment;
    heap_attribute->xByteAlignmentMask = byteAlignmentMask;
    heap_attribute->uxHeapStructSize = ( ( sizeof ( heap_BlockLink_t ) + ( byteAlignment - 1 ) ) & ~byteAlignmentMask );

    pxHeapRegion = &( pxHeapRegions[ xDefinedRegions ] );

	while( pxHeapRegion->xSizeInBytes > 0 )
	{
		xTotalRegionSize = pxHeapRegion->xSizeInBytes;

		/* Ensure the heap region starts on a correctly aligned boundary. */
		ulAddress = ( uint32_t ) pxHeapRegion->pucStartAddress;
		if( ( ulAddress & heap_attribute->xByteAlignmentMask ) != 0 )
		{
			ulAddress += ( heap_attribute->xByteAlignment - 1 );
			ulAddress &= ~heap_attribute->xByteAlignmentMask;

			/* Adjust the size for the bytes lost to alignment. */
			xTotalRegionSize -= ulAddress - ( uint32_t ) pxHeapRegion->pucStartAddress;
		}

		pucAlignedHeap = ( uint8_t * ) ulAddress;

		/* Set xStart if it has not already been set. */
		if( xDefinedRegions == 0 )
		{
			/* xStart is used to hold a pointer to the first item in the list of
			free blocks.  The void cast is used to prevent compiler warnings. */
			heap_attribute->xStart.pxNextFreeBlock = ( heap_BlockLink_t * ) pucAlignedHeap;
			heap_attribute->xStart.xBlockSize = ( size_t ) 0;
		}
		else
		{
			/* Should only get here if one region has already been added to the
			heap. */
			configASSERT( heap_attribute->pxEnd != NULL );

			/* Check blocks are passed in with increasing start addresses. */
			configASSERT( ulAddress > ( uint32_t ) heap_attribute->pxEnd );
		}

		/* Remember the location of the end marker in the previous region, if
		any. */
		pxPreviousFreeBlock = heap_attribute->pxEnd;

		/* pxEnd is used to mark the end of the list of free blocks and is
		inserted at the end of the region space. */
		ulAddress = ( ( uint32_t ) pucAlignedHeap ) + xTotalRegionSize;
		ulAddress -= heap_attribute->uxHeapStructSize;
		ulAddress &= ~heap_attribute->xByteAlignmentMask;
		heap_attribute->pxEnd = ( heap_BlockLink_t * ) ulAddress;
		heap_attribute->pxEnd->xBlockSize = 0;
		heap_attribute->pxEnd->pxNextFreeBlock = NULL;

		/* To start with there is a single free block in this region that is
		sized to take up the entire heap region minus the space taken by the
		free block structure. */
		pxFirstFreeBlockInRegion = ( heap_BlockLink_t * ) pucAlignedHeap;
		pxFirstFreeBlockInRegion->xBlockSize = ulAddress - ( uint32_t ) pxFirstFreeBlockInRegion;
		pxFirstFreeBlockInRegion->pxNextFreeBlock = heap_attribute->pxEnd;

		/* If this is not the first region that makes up the entire heap space
		then link the previous region to this region. */
		if( pxPreviousFreeBlock != NULL )
		{
			pxPreviousFreeBlock->pxNextFreeBlock = pxFirstFreeBlockInRegion;
		}

		xTotalHeapSize += pxFirstFreeBlockInRegion->xBlockSize;

		/* Move onto the next HeapRegion_t structure. */
		xDefinedRegions++;
		pxHeapRegion = &( pxHeapRegions[ xDefinedRegions ] );
	}

	heap_attribute->xMinimumEverFreeBytesRemaining = xTotalHeapSize;
	heap_attribute->xFreeBytesRemaining = xTotalHeapSize;

	/* Check something was actually defined before it is accessed. */
	configASSERT( xTotalHeapSize );
}
