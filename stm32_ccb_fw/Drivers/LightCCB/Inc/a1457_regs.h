/**
  ******************************************************************************
  * \file    a1457_regs.h
  * \author  Light Co
  * \version V0.0.1
  * \date    04-Feb-2016
  * \brief   Header for A1457 register map
  ******************************************************************************
  */

#ifndef __A1457_REGS_H
#define __A1457_REGS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "platform.h"
#include "types.h"

#define A1457_FIELD_VAL(x, mask, offset) (((x) >> (offset)) & (mask))

#define		A1457_POSMSB		0x00
#define		A1457_POSLSB		0x01
#define		A1457_CONTROL1		0x02
#define		A1457_CONTROL2		0x03
#define		A1457_SLEEPCONT		0x04
#define		A1457_ADCVALUE1		0x05
#define		A1457_ADCVALUE2		0x06
#define		A1457_EEWRSTAT		0x07

// Volatile registers shadow EEPROM BLK1 to 3
#define		A1457_SHD_PREG		0x14
#define		A1457_SHD_NREG		0x15
#define		A1457_SHD_PIDCTL0		0x16
#define		A1457_SHD_PIDCTL1		0x17
#define		A1457_SHD_PIDCTL2		0x18
#define		A1457_SHD_PIDCTL3		0x19
#define		A1457_SHD_PIDCTL4		0x1A
#define		A1457_SHD_PIDCTL5		0x1B
#define		A1457_SHD_PIDCTL6		0x1C
#define		A1457_SHD_FACTRES1		0x1D
#define		A1457_SHD_FACTRES2		0x1E
#define		A1457_SHD_FACTRES3		0x1F

//EEPROM Block Registers

#define		A1457_EE_BLOCK1_PREG		0x94
#define		A1457_EE_BLOCK1_NREG		0x95
#define		A1457_EE_BLOCK1_CTRL3		0x96
#define		A1457_EE_BLOCK1_PIDCTL1		0x97

#define		A1457_EE_BLOCK2_PIDCTL2		0x98
#define		A1457_EE_BLOCK2_PIDCTL3		0x99
#define		A1457_EE_BLOCK2_PIDCTL4		0x9A
#define		A1457_EE_BLOCK2_PIDCTL5		0x9B

#define		A1457_EE_BLOCK3_PIDCTL6			0x9C
#define		A1457_EE_BLOCK3_FACTRES1		0x9D
#define		A1457_EE_BLOCK3_FACTRES2		0x9E
#define		A1457_EE_BLOCK3_FACTRES3		0x9F

// Scratch register Light specific
#define		A1457_EE_HSTOP_MIN_LSB		0xA0
#define		A1457_EE_HSTOP_MIN_MSB		0xA1
#define		A1457_EE_HSTOP_MAX_LSB		0xA2
#define		A1457_EE_HSTOP_MAX_MSB		0xA3
#define		A1457_EE_KHSTOP_MIN_LSB		0xA4
#define		A1457_EE_KHSTOP_MIN_MSB		0xA5
#define		A1457_EE_KHSTOP_MAX_LSB		0xA6
#define		A1457_EE_KHSTOP_MAX_MSB		0xA7
#define		A1457_EE_HOME_LSB			0xA8
#define		A1457_EE_HOME_MSB			0xA9
#define		A1457_EE_KAA_LSB			0xAA
#define		A1457_EE_KAA_MSB			0xAB
#define		A1457_EE_AA_LSB				0xAC
#define		A1457_EE_AA_MSB				0xAD
#define		A1457_EE_GAIN_LSB			0xAE
#define		A1457_EE_GAIN_MSB			0xAF

#ifdef __cplusplus
}
#endif

#endif /* __A1457_REGS_H */
