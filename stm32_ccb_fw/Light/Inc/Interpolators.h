/*
 * Interpolators.h - Function interpolators
 *
 *  Created on: Jun 6, 2015
 *      Author: john
 */

#ifndef LIGHT_INC_INTERPOLATORS_H_
#define LIGHT_INC_INTERPOLATORS_H_

typedef enum
{
	InterpType_None = 0,
	InterpType_Float  = 1,
	InterpType_Max
} InterpType;

typedef struct FloatPointRec
{
	float x;
	float y;
} FloatPoint;

typedef struct InterpTableRec
{
	InterpType 	   DataType;
	unsigned int   Size; // Number of table elements
	void 		  *Table; // Cast as needed
} InterpTable;

// Diagnostic routine: Validate the table is properly formed.
int ValidateFloatTable(InterpTable *Table);

// Setup internal table data.
int SetupInterpFloatTable(InterpTable *Table);

// Linearly interpolate x within the table to compute y.
float InterpLinearFloat(InterpTable *Table, float x);

#endif /* LIGHT_INC_INTERPOLATORS_H_ */
