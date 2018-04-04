/*
 * Interpolators.c
 *
 *  Created on: Jun 6, 2015
 *      Author: john
 */

#include <stdio.h>
#include "Interpolators.h"
#include "log.h"
#include "types.h"

static void PrintInterpTable(InterpTable *Table)
{
    int i;
    FloatPoint *t = (FloatPoint *)(Table->Table);

    log_debug("InterpTable:");
    log_debug("  DataType: %d", Table->DataType);
    log_debug("  Size:     %d", Table->Size);

    for (i = 0; i < Table->Size; i++)
    {
        Int16 integer1, fraction1, integer2, fraction2;

        integer1 = t[i].x;
        fraction1 = (t[i].x - integer1) * 1000;
        if(fraction1 < 0) fraction2 = -fraction1;

        integer2 = t[i].y;
        fraction2 = (t[i].y - integer2) * 1000;
        if(fraction2 < 0) fraction2 = -fraction2;

        log_debug("  Table[%d]: x = %d.%03d  y = %d.%03d",
                    i, integer1, fraction1, integer2, fraction2);
    }
}
int ValidateFloatTable(InterpTable *Table)
{
	int i;
	FloatPoint *t = NULL;

	if (Table == NULL) return 0;
	t = (FloatPoint*) Table->Table;

	for(i = 0; i < Table->Size-1; i++)
	{
		// Check for divide by zero in the slope calculation.
		if(t[i].x == t[i+1].x) return 0;

		// Ensure the table is sorted in x.
		if(t[i].x > t[i+1].x) return 0;
	}

	return 1;
}

int SetupInterpFloatTable(InterpTable *Table)
{
	int i, j, n;
	FloatPoint *t = NULL;
	float tmp;

	if (Table == NULL) return 0;
	t = (FloatPoint*) Table->Table;
	n = Table->Size;

	// Ensure the table is in order of increasing x.
	// Reverse the table if needed.
	if(t[0].x > t[n-1].x)
	{
		for(i = 0; i < n/2; i++)
		{
			// Swap table entries.
			j = n - i - 1;
			tmp = t[i].x; t[i].x = t[j].x; t[j].x = tmp;
			tmp = t[i].y; t[i].y = t[j].y; t[j].y = tmp;
		}
	}

	// Add any other initialization.

	PrintInterpTable(Table);

	return ValidateFloatTable(Table);
}

float InterpLinearFloat(InterpTable *Table, float x)
{
	FloatPoint *t = NULL;
	int n;
	int i;
	float y;
	float dx;

	if (!Table || Table->DataType != InterpType_Float)
	{
		// TODO: Assert!
		return -1;
	}

	n = Table->Size;
	t = (FloatPoint *) Table->Table;

	// Clamp to table bounds.
	if (x < t[0  ].x) return t[0  ].y;
	if (x > t[n-1].x) return t[n-1].y;

	// Locate the pair of elements between which to interpolate.
	for (i = 0; i < n-1; i++)
	{
		if ((x >= t[i].x) && (x <= t[i+1].x)) break;
	}

	// Linearly interpolate between t[i] and t[i+1].
	dx = t[i+1].x - t[i].x;
	if (dx == 0.f)
	{
		// TODO: ASSERT()!
		return -1.f;
	}
	y = t[i].y + (t[i+1].y - t[i].y) * (x - t[i].x) / dx;
	return y;
}
