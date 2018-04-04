/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without prior permission of
 * The LightCo.
 *
 * @file	hal_ddr.c
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Jun-20-2016
 * @brief
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "hal_ddr.h"
#include "std_type.h"
#include "cortex_r4.h"
#include "stdio.h"

#define GUC_SETTINGS
// #define DDR_800
#define _DEBUG

/* Privated define------------------------------------------------------------*/
#ifdef GUC_SETTINGS
#define RIDR		0x00
#define PIR			0x01*0x4
#define PGCR		0x02*0x4
#define PGSR		0x03*0x4
#define DLLGCR		0x04*0x4
#define ACDLLCR		0x05*0x4
#define PTR0		0x06*0x4
#define PTR1		0x07*0x4
#define PTR2		0x08*0x4
#define ACIOCR		0x09*0x4
#define DXCCR		0x0A*0x4
#define DSGCR		0x0B*0x4
#define DCR			0x0C*0x4
#define DTPR0		0x0D*0x4
#define DTPR1		0x0E*0x4
#define DTPR2		0x0F*0x4
#define MR0			0x10*0x4
#define MR1			0x11*0x4
#define MR2			0x12*0x4
#define MR3			0x13*0x4
#define ODTCR		0x14*0x4
#define DTAR		0x15*0x4
#define DTDR0		0x16*0x4
#define DTDR1		0x17*0x4
#define DCUAR		0x30*0x4
#define DCUDR		0x31*0x4
#define DCURR		0x32*0x4
#define DCULR		0x33*0x4
#define DCUGCR		0x34*0x4
#define DCUTPR		0x35*0x4
#define DCUSR0		0x36*0x4
#define DCUSR1		0x37*0x4
#define BISTRR		0x40*0x4
#define BISTMSKR0	0x41*0x4
#define BISTMSKR1	0x42*0x4
#define BISTWCR		0x43*0x4
#define BISTLSR		0x44*0x4
#define BISTAR0		0x45*0x4
#define BISTAR1		0x46*0x4
#define BISTAR2		0x47*0x4
#define BISTUDPR	0x48*0x4
#define BISTGSR		0x49*0x4
#define BISTWER		0x4a*0x4
#define BISTBER0	0x4b*0x4
#define BISTBER1	0x4c*0x4
#define BISTBER2	0x4d*0x4
#define BISTWCSR	0x4e*0x4
#define BISTFWR0	0x4f*0x4
#define BISTFWR1	0x50*0x4
#define GPR0		0x5e*0x4
#define GPR1		0x5f*0x4
#define ZQ0CR0		0x60*0x4
#define ZQ0CR1		0x61*0x4
#define ZQ0SR0		0x62*0x4
#define ZQ0SR1		0x63*0x4
#define ZQ1CR0		0x64*0x4
#define ZQ1CR1		0x65*0x4
#define ZQ1SR0		0x66*0x4
#define ZQ1SR1		0x67*0x4
#define ZQ2CR0		0x68*0x4
#define ZQ2CR1		0x69*0x4
#define ZQ2SR0		0x6a*0x4
#define ZQ2SR1		0x6b*0x4
#define ZQ3CR0		0x6c*0x4
#define ZQ3CR1		0x6d*0x4
#define ZQ3SR0		0x6e*0x4
#define ZQ3SR1		0x6f*0x4
#define DX0GCR		0x70*0x4
#define DX0GSR0		0x71*0x4
#define DX0GSR1		0x72*0x4
#define DX0DLLCR	0x73*0x4
#define DX0DQTR		0x74*0x4
#define DX0DQSTR	0x75*0x4
#define DX1GCR		0x80*0x4
#define DX1GSR0		0x81*0x4
#define Dx1GSR1		0x82*0x4
#define DX1DLLCR	0x83*0x4
#define DX1DQTR		0x84*0x4
#define DX1DQSTR	0x85*0x4
#define DX2GCR		0x90*0x4
#define DX2GSR0		0x91*0x4
#define Dx2GSR1		0x92*0x4
#define DX2DLLCR	0x93*0x4
#define DX2DQTR		0x94*0x4
#define DX2DQSTR	0x95*0x4
#define DX3GCR		0xa0*0x4
#define DX3GSR0		0xa1*0x4
#define Dx3GSR1		0xa2*0x4
#define DX3DLLCR	0xa3*0x4
#define DX3DQTR		0xa4*0x4
#define DX3DQSTR	0xa5*0x4
#define DX4GCR		0xb0*0x4
#define DX4GSR0		0xb1*0x4
#define Dx4GSR1		0xb2*0x4
#define DX4DLLCR	0xb3*0x4
#define DX4DQTR		0xb4*0x4
#define DX4DQSTR	0xb5*0x4

#define DX5GCR		0xc0*0x4
#define DX5GSR0		0xc1*0x4
#define Dx5GSR1		0xc2*0x4
#define DX5DLLCR	0xc3*0x4
#define DX5DQTR		0xc4*0x4
#define DX5DQSTR	0xc5*0x4
#define DX6GCR		0xd0*0x4
#define DX6GSR0		0xd1*0x4
#define Dx6GSR1		0xd2*0x4
#define DX6DLLCR	0xd3*0x4
#define DX6DQTR		0xd4*0x4
#define DX6DQSTR	0xd5*0x4

#define DX7GCR		0xe0*0x4
#define DX7GSR0		0xe1*0x4
#define Dx7GSR1		0xe2*0x4
#define DX7DLLCR	0xe3*0x4
#define DX7DQTR		0xe4*0x4
#define DX7DQSTR	0xe5*0x4

#define DX8GCR		0xf0*0x4
#define DX8GSR0		0xf1*0x4
#define Dx8GSR1		0xf2*0x4
#define DX8DLLCR	0xf3*0x4
#define DX8DQTR		0xf4*0x4
#define DX8DQSTR	0xf5*0x4
#else
#define DCR 0x0C*0x4
#define MR0 0x10*0x4
#define MR1 0x11*0x4
#define MR2 0x12*0x4
#define MR3 0x13*0x4
#define DTPR0 0x0D*0x4
#define DTPR1 0x0E*0x4
#define DTPR2 0x0F*0x4
#define PIR 0x01*0x4
#define PGCR 0x02*0x4
#define PGSR 0x03*0x4
#define DXCCR 0x0A*0x4
#define DSGCR 0x0B*0x4
#define DX0GCR 0x70*0x4
#define DX1GCR 0x80*0x4
#define DX2GCR 0x90*0x4
#define DX3GCR 0xA0*0x4
#define DX0DQSTR 0x75*0x4
#define DX1DQSTR 0x85*0x4
#define DX2DQSTR 0x95*0x4
#define DX3DQSTR 0xa5*0x4
#define ACDLLCR 0x05*0x4
#define BISTRR		0x40*0x4
#define BISTMSKR0	0x41*0x4
#define BISTMSKR1	0x42*0x4
#define BISTWCR		0x43*0x4
#define BISTLSR		0x44*0x4
#define BISTAR0		0x45*0x4
#define BISTAR1		0x46*0x4
#define BISTAR2		0x47*0x4
#define BISTUDPR	0x48*0x4
#define BISTGSR		0x49*0x4
#define BISTWER		0x4A*0x4
#define BISTBER0	0x4B*0x4
#define BISTBER1	0x4C*0x4
#define BISTBER2	0x4D*0x4
#define BISTWCSR	0x4E*0x4
#define BISTFWR0	0x4F*0x4
#define BISTFWR1	0x50*0x4

#define BINST_NOP	0
#define BINST_RUN	1
#define BINST_STOP	2
#define BINST_RESET	3
#endif
/* Privated functions prototypes ---------------------------------------------*/
/* Privated variables --------------------------------------------------------*/
#ifdef GUC_SETTINGS
#define SETREG32(a, v)	writel(a,v)
#define GETREG32(a)		readl(a)
extern void udelay (unsigned int x);

static void prvRead4W(uint32_t addr, uint32_t *pData)
{
	asm volatile (
			"ldm %[addr], {r5-r8} \n\t"
			"stm %[data], {r5-r8} \n\t"
			:
			: [addr] "r" (addr), [data] "r" (pData)
			: "r5", "r6", "r7", "r8", "memory"
	);
}

static void prvWrite4W(uint32_t addr, const uint32_t *pData)
{
	asm volatile (
			"ldm %[data], {r5-r8} \n\t"
			"stm %[addr], {r5-r8} \n\t"
			:
			: [addr] "r" (addr), [data] "r" (pData)
			: "r5", "r6", "r7", "r8", "memory"
	);
}

static void prvITM_SW_Reset(unsigned int base)
{
	unsigned int v;
	SETREG32(base + PIR, 0x00000011);
  	do {
		v = GETREG32( base + PGSR ) & 1;
	} while(!v);
	udelay(15);
}

#ifdef _DEBUG
uint8_t ddr_rdata_for_training[2][4][6][4][4];
#endif

int prvSW_DQS_GW_Training(unsigned int base)
{
	int ret = 0;
	char res[4][6][4];
	unsigned int v;
	const uint32_t pattern[4] = {
		0x55555555,
		0xaaaaaaaa,
		0x33333333,
		0xcccccccc,
	};
	uint32_t rd_data[4] = {0};
	uint32_t test_addr;
#ifdef _DEBUG
	int idx=idx;
#endif
#ifdef CONFIG_CAL_PERF
	uint32_t ts, te, tav = 0;
	ts = timerN_get_count(FREERUN_TIMERn);
#endif

	if (base == DDRP0_BASE){
		test_addr = 0x04000000;
#ifdef _DEBUG
		idx = 0;
#endif
	}
	else {
		test_addr = 0x04000010;
#ifdef _DEBUG
		idx = 1;
#endif
	}	
	//Step 1
	v = GETREG32(base + PGCR);
	//Disable DFTCMP
	v &= ~(1 << 2);
	SETREG32(base + PGCR, v);

	//Step 3
	prvWrite4W(test_addr, pattern);

	//Disable PDDISDX
	v = GETREG32(base + PGCR);
	v &= ~(1 << 24);
	SETREG32(base + PGCR, v);

	//Step 5
	for (int i = 0; i < 4; i++) {
		v = GETREG32(base + DX0GCR + 0x40 * i);
		v &= ~0x1;
		SETREG32(base + DX0GCR + 0x40 * i, v);
	}

	for (int i = 0; i < 4; i++) {
		v = GETREG32(base + DX0GCR + 0x40 * i);
		v |= 0x1;
		SETREG32(base + DX0GCR + 0x40 * i, v);
		v = GETREG32(base + DX0DQSTR + 0x40 * i);
		v &= ~((7 << 0) | (3 << 12));
		for (int j = 0; j < 6; j ++) {
			for(int k = 0; k < 4; k++) {
				SETREG32(base + DX0DQSTR + 0x40 * i, v | j | (k << 12));
				prvRead4W(test_addr, rd_data);
				prvITM_SW_Reset(base);
#ifdef _DEBUG
				ddr_rdata_for_training[idx][i][j][k][0] = ((rd_data[0] >> (8 * i)) & 0xFF);
				ddr_rdata_for_training[idx][i][j][k][1] = ((rd_data[1] >> (8 * i)) & 0xFF);
				ddr_rdata_for_training[idx][i][j][k][2] = ((rd_data[2] >> (8 * i)) & 0xFF);
				ddr_rdata_for_training[idx][i][j][k][3] = ((rd_data[3] >> (8 * i)) & 0xFF);
#endif
				if (((rd_data[0] & (0xFF << (8 * i))) != (pattern[0] & (0xFF << (8 * i)))) ||
					((rd_data[1] & (0xFF << (8 * i))) != (pattern[1] & (0xFF << (8 * i)))) ||
					((rd_data[2] & (0xFF << (8 * i))) != (pattern[2] & (0xFF << (8 * i)))) ||
					((rd_data[3] & (0xFF << (8 * i))) != (pattern[3] & (0xFF << (8 * i))))) {
						res[i][j][k] = 0;
				}
				else {
						res[i][j][k] = 1;
				}
			}
		}
		v = GETREG32(base + DX0GCR + 0x40 * i);
		v &= ~0x1;
		SETREG32(base + DX0GCR + 0x40 * i, v);
	}
#ifdef _DEBUG
	int final_setting[4] = {-1, -1, -1, -1};
#endif
	for (int i = 0; i < 4; i++) {
		int stp, enp;
		int good_val, good_len;
		char *t_res;

		v = GETREG32(base + DX0GCR + 0x40 * i);
		v |= 0x1;
		SETREG32(base + DX0GCR + 0x40 * i, v);

		v = GETREG32(base + DX0DQSTR + 0x40 * i);
		v &= ~((7 << 0) | (3 << 12));
		enp = stp = -1;
		good_len = -1;
		t_res = (char *)res[i];
		for (int j = 0; j < 24; j++) {
			if (t_res[j]) {
				if (stp < 0) {
					enp = stp = j;
				}
				else {
					enp = j;
				}
			}
			else {
				if (stp >= 0) {
					if ((enp - stp) > good_len) {
						good_len = enp - stp;
						good_val = (stp + enp) >> 1;
					}
					enp = stp = -1;
				}
			}
		}
		if (good_len >= 0) {
			v |= ((good_val&0x3) << 12) + (good_val>>2);
#ifdef _DEBUG
			final_setting[i] = good_val;
#endif
			ret |= 1 << i;
		}
		SETREG32(base + DX0DQSTR + 0x40 * i, v);
	}
	//Enable PDDISDX
	v = GETREG32(base + PGCR);
	v |= (1 << 24);
	SETREG32(base + PGCR, v);

	//DLLSRST + DLLLOCK + ZCAL + ITMSRST
	SETREG32(base+PIR, 0x00000011);
	do
	{
		v = GETREG32(base + PGSR) & 0x1;
	} while(v != 0x1);
	//udelay(15);

#ifdef CONFIG_CAL_PERF
	te = timerN_get_count(FREERUN_TIMERn);
	if (te > ts) {
		tav += (0xFFFFFFFF - te) + ts;
	}
	else {
		tav += ts - te;
	}
	printf("%"PRIx32"\n", tav);
#endif

#ifdef _DEBUG
	if (ret == 0xF) {
		printf("DQS_GW:%#x\n", base);
		for (int i = 0; i < 4; i++) {
			printf("Byte %d:(GL=%d, GP=%d)\n", i, (final_setting[i]>>2), (final_setting[i]&0x3));
			for (int j = 0; j < 6; j++) {
				printf("\t Gating Latency %d:", j);
				for (int k = 0; k < 4; k++) {
					printf(" %c", (res[i][j][k])?'O':'X');
				}
				printf("\n");
			}
		}
	}
	else {
		printf("DQS_GW(%#x): Training Fail.(%x)\n", base, ret);
	}
#endif
	return (ret == 0xF)?0:-1;
}

int prvSW_DQS_Training(unsigned int base)
{
	int ret = 0;
	char res[4][2][8];
	unsigned int v;
	const uint32_t pattern[4] = {
		0x55555555,
		0xaaaaaaaa,
		0x33333333,
		0xcccccccc,
	};
	uint32_t rd_data[4] = {0};
	uint32_t test_addr;
#ifdef CONFIG_CAL_PERF
	uint32_t ts, te, tav = 0;
	ts = timerN_get_count(FREERUN_TIMERn);
#endif

	if (base == DDRP0_BASE){
		test_addr = 0x04000000;
	}
	else {
		test_addr = 0x04000010;
	}	

	//Step 3
	prvWrite4W(test_addr, pattern);

	for (int i = 0; i < 4; i++) {
		v = GETREG32(base + DX0DQSTR + 0x40 * i);
		v &= ~((7 << 20) | (7 << 23));
		for (int j = 0; j < 8; j ++) {
			SETREG32(base + DX0DQSTR + 0x40 * i, v | (j << 20) | (j << 23));
			prvRead4W(test_addr, rd_data);
			prvITM_SW_Reset(base);
			if (((rd_data[0] & (0xFF << (8 * i))) != (pattern[0] & (0xFF << (8 * i)))) ||
				((rd_data[2] & (0xFF << (8 * i))) != (pattern[2] & (0xFF << (8 * i))))) {
					res[i][0][j] = 0;
			}
			else {
					res[i][0][j] = 1;
			}
			if (((rd_data[1] & (0xFF << (8 * i))) != (pattern[1] & (0xFF << (8 * i)))) ||
				((rd_data[3] & (0xFF << (8 * i))) != (pattern[3] & (0xFF << (8 * i))))) {
					res[i][1][j] = 0;
			}
			else {
					res[i][1][j] = 1;
			}
		}
	}

#ifdef _DEBUG
	int final_setting[4][2];
#endif
	for (int i = 0; i < 4; i++) {
		int stp, enp, cnt;
		char *t_res;

		v = GETREG32(base + DX0DQSTR + 0x40 * i);
		v &= ~((7 << 20) | (7 << 23));
		cnt = 0;
		enp = stp = -1;
		t_res = (char *)res[i][0];
		do {
			if (t_res[cnt]) {
				if (stp < 0) {
					enp = stp = cnt;
				}
				else {
					enp = cnt;
				}
			}
			else {
				if (stp >= 0) {
					break;
				}
			}
		} while (++cnt < 8);
		if (stp >=0) {
			stp = (stp + enp) >> 1;
			v |= stp << 20;
#ifdef _DEBUG
			final_setting[i][0] = stp;
#endif
			ret |= 1 << (i * 2);
		}
		cnt = 0;
		enp = stp = -1;
		t_res = (char *)res[i][1];
		do {
			if (t_res[cnt]) {
				if (stp < 0) {
					enp = stp = cnt;
				}
				else {
					enp = cnt;
				}
			}
			else {
				if (stp >= 0) {
					break;
				}
			}
		} while (++cnt < 8);
		if (stp >=0) {
			stp = (stp + enp) >> 1;
			v |= stp << 23;
#ifdef _DEBUG
			final_setting[i][1] = stp;
#endif
			ret |= 2 << (i * 2);
		}

		SETREG32(base + DX0DQSTR + 0x40 * i, v);
	}
	prvITM_SW_Reset(base);

#ifdef CONFIG_CAL_PERF
	te = timerN_get_count(FREERUN_TIMERn);
	if (te > ts) {
		tav += (0xFFFFFFFF - te) + ts;
	}
	else {
		tav += ts - te;
	}
	printf("%"PRIx32"\n", tav);
#endif

#ifdef _DEBUG
	printf("DQS:%#x\n", base);
	for (int i = 0; i < 4; i++) {
		printf("Byte %d(DQS DL=%d, DQSN DL=%d):\n", i, final_setting[i][0], final_setting[i][1]);
		for (int j = 0; j < 2; j++) {
			printf("\t %s:", j?"DQSN":"DQS");
			for (int k = 0; k < 8; k++) {
				printf(" %c", (res[i][j][k])?'O':'X');
			}
			printf("\n");
		}
	}
#endif
	return (ret == 0xFF)?0:-1;
}

#ifdef DDR_800
void DDR_MCTL_init (void)
{
	SETREG32(DDRC_BASE+0x0304, 0x00000001);
	SETREG32(DDRC_BASE+0x0030, 0x00000001);
	// //GETREG32(DDRC_BASE+0x0004);
	// //SETREG32(DDRC_BASE+0x0000 ,0x03040000);
	// //SETREG32(DDRC_BASE+0x0000 ,0x03040008);
	// //SETREG32(DDRC_BASE+0x0000 ,0x03040008);
	// //SETREG32(DDRC_BASE+0x0000 ,0x03040008);
	// //SETREG32(DDRC_BASE+0x0000 ,0x03040008);
	// //SETREG32(DDRC_BASE+0x0000 ,0x03040008);
	SETREG32(DDRC_BASE+0x0000 ,0x03040008);
	// //SETREG32(DDRC_BASE+0x0010 ,0x00000030);
	// //SETREG32(DDRC_BASE+0x0010 ,0x00000030);
	// //SETREG32(DDRC_BASE+0x0010 ,0x00006030);
	SETREG32(DDRC_BASE+0x0010 ,0x00006030);
	SETREG32(DDRC_BASE+0x0014 ,0x0000f322);
	// //SETREG32(DDRC_BASE+0x0020 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x0020 ,0x00000000);
	SETREG32(DDRC_BASE+0x0020 ,0x00000000);
	SETREG32(DDRC_BASE+0x0024 ,0xf15f7793);
	// //SETREG32(DDRC_BASE+0x0030 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x0030 ,0x00000002);
	// //SETREG32(DDRC_BASE+0x0030 ,0x00000002);
	// //SETREG32(DDRC_BASE+0x0030 ,0x00000002);
	SETREG32(DDRC_BASE+0x0030 ,0x00000002);
	// //SETREG32(DDRC_BASE+0x0034 ,0x00402001);
	// //SETREG32(DDRC_BASE+0x0034 ,0x00400501);
	SETREG32(DDRC_BASE+0x0034 ,0x00170501);
	// //SETREG32(DDRC_BASE+0x0038 ,0x00000003);
	// //SETREG32(DDRC_BASE+0x0038 ,0x00000003);
	SETREG32(DDRC_BASE+0x0038 ,0x00e50003);
	// //SETREG32(DDRC_BASE+0x0050 ,0x00210000);
	// //SETREG32(DDRC_BASE+0x0050 ,0x00210050);
	// //SETREG32(DDRC_BASE+0x0050 ,0x00217050);
	SETREG32(DDRC_BASE+0x0050 ,0x00d17050);
	// //SETREG32(DDRC_BASE+0x0054 ,0x0000000c);
	SETREG32(DDRC_BASE+0x0054 ,0x000b000c);
	// //SETREG32(DDRC_BASE+0x0060 ,0x00000000);
	SETREG32(DDRC_BASE+0x0060 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x0064 ,0x0062002a);
	// //SETREG32(DDRC_BASE+0x0064 ,0x0062002a);
	SETREG32(DDRC_BASE+0x0064 ,0x002a002a);
	// //SETREG32(DDRC_BASE+0x00c0 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x00c0 ,0x00000000);
	SETREG32(DDRC_BASE+0x00c0 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x00d0 ,0x00020003);
	// //SETREG32(DDRC_BASE+0x00d0 ,0x00030003);
	SETREG32(DDRC_BASE+0x00d0 ,0x40030003);
	SETREG32(DDRC_BASE+0x00d4 ,0x00000007);
	// //SETREG32(DDRC_BASE+0x00d8 ,0x00000d06);
	SETREG32(DDRC_BASE+0x00d8 ,0x00000806);
	// //SETREG32(DDRC_BASE+0x00dc ,0x00000044);
	SETREG32(DDRC_BASE+0x00dc ,0x00830044);
	// //SETREG32(DDRC_BASE+0x00e0 ,0x00000000);
	SETREG32(DDRC_BASE+0x00e0 ,0x00020000);
	// //SETREG32(DDRC_BASE+0x00e4 ,0x00100003);
	SETREG32(DDRC_BASE+0x00e4 ,0x00080003);
	// //SETREG32(DDRC_BASE+0x00f0 ,0x00000000);
	SETREG32(DDRC_BASE+0x00f0 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x00f4 ,0x0000066e);
	// //SETREG32(DDRC_BASE+0x00f4 ,0x0000067e);
	SETREG32(DDRC_BASE+0x00f4 ,0x0000037e);
	// //SETREG32(DDRC_BASE+0x0100 ,0x0f101b08);
	// //SETREG32(DDRC_BASE+0x0100 ,0x0f100c08);
	// //SETREG32(DDRC_BASE+0x0100 ,0x0f0a0c08);
	SETREG32(DDRC_BASE+0x0100 ,0x070a0608);
	// //SETREG32(DDRC_BASE+0x0104 ,0x0008040d);
	// //SETREG32(DDRC_BASE+0x0104 ,0x0008020d);
	SETREG32(DDRC_BASE+0x0104 ,0x0002020d);
	// //SETREG32(DDRC_BASE+0x0108 ,0x03050606);
	// //SETREG32(DDRC_BASE+0x0108 ,0x03050606);
	// //SETREG32(DDRC_BASE+0x0108 ,0x03030606);
	//SETREG32(DDRC_BASE+0x0108 ,0x02030606);
	SETREG32(DDRC_BASE+0x0108 ,0x02030806);
	// //SETREG32(DDRC_BASE+0x010c ,0x00505000);
	SETREG32(DDRC_BASE+0x010c ,0x00505000);
	// //SETREG32(DDRC_BASE+0x0110 ,0x05040405);
	// //SETREG32(DDRC_BASE+0x0110 ,0x05040205);
	// //SETREG32(DDRC_BASE+0x0110 ,0x05020205);
	SETREG32(DDRC_BASE+0x0110 ,0x04020205);
	// //SETREG32(DDRC_BASE+0x0114 ,0x05050403);
	// //SETREG32(DDRC_BASE+0x0114 ,0x05050303);
	// //SETREG32(DDRC_BASE+0x0114 ,0x050d0303);
	SETREG32(DDRC_BASE+0x0114 ,0x0c0d0303);
	// //SETREG32(DDRC_BASE+0x0118 ,0x02020004);
	// //SETREG32(DDRC_BASE+0x0118 ,0x020f0004);
	SETREG32(DDRC_BASE+0x0118 ,0x020f0004);
	// //SETREG32(DDRC_BASE+0x011c ,0x00000209);
	SETREG32(DDRC_BASE+0x011c ,0x00000209);
	// // SETREG32(DDRC_BASE+0x0120 ,0x00004401);
	SETREG32(DDRC_BASE+0x0120 ,0x00002801);
	SETREG32(DDRC_BASE+0x0138 ,0x0000002c);
	// //SETREG32(DDRC_BASE+0x0180 ,0x02000012);
	// //SETREG32(DDRC_BASE+0x0180 ,0x00480012);
	// //SETREG32(DDRC_BASE+0x0180 ,0x00480012);
	// //SETREG32(DDRC_BASE+0x0180 ,0x40480012);
	SETREG32(DDRC_BASE+0x0180 ,0x40480012);
	// //SETREG32(DDRC_BASE+0x0184 ,0x02000070);
	SETREG32(DDRC_BASE+0x0184 ,0x00a00070);
	SETREG32(DDRC_BASE+0x0188 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x0190 ,0x07020001);
	// //SETREG32(DDRC_BASE+0x0190 ,0x07020101);
	// //SETREG32(DDRC_BASE+0x0190 ,0x07020101);
	// //SETREG32(DDRC_BASE+0x0190 ,0x07030101);
	// //SETREG32(DDRC_BASE+0x0190 ,0x07030101);
	SETREG32(DDRC_BASE+0x0190 ,0x02020001);
	// //SETREG32(DDRC_BASE+0x0194 ,0x00000402);
	// //SETREG32(DDRC_BASE+0x0194 ,0x00000202);
	SETREG32(DDRC_BASE+0x0194 ,0x00030202);
	// //SETREG32(DDRC_BASE+0x0198 ,0x07000000);
	// //SETREG32(DDRC_BASE+0x0198 ,0x07000020);
	// //SETREG32(DDRC_BASE+0x0198 ,0x07000020);
	// //SETREG32(DDRC_BASE+0x0198 ,0x0700b020);
	// //SETREG32(DDRC_BASE+0x0198 ,0x0701b020);
	// //SETREG32(DDRC_BASE+0x0198 ,0x0701b020);
	SETREG32(DDRC_BASE+0x0198 ,0x0701b020);
	// //SETREG32(DDRC_BASE+0x01a0 ,0x00400005);
	// //SETREG32(DDRC_BASE+0x01a0 ,0x00400005);
	// //SETREG32(DDRC_BASE+0x01a0 ,0x20400005);
	// //SETREG32(DDRC_BASE+0x01a0 ,0x60400005);
	SETREG32(DDRC_BASE+0x01a0 ,0x60400005);
	// //SETREG32(DDRC_BASE+0x01a4 ,0x0001007c);
	SETREG32(DDRC_BASE+0x01a4 ,0x001b007c);
	SETREG32(DDRC_BASE+0x01a8 ,0x80000000);
	// //SETREG32(DDRC_BASE+0x01b0 ,0x00000001);
	// //SETREG32(DDRC_BASE+0x01b0 ,0x00000001);
	// //SETREG32(DDRC_BASE+0x01b0 ,0x00000001);
	SETREG32(DDRC_BASE+0x01b0 ,0x00000001);
	SETREG32(DDRC_BASE+0x0200 ,0x00000010);
	// //SETREG32(DDRC_BASE+0x0204 ,0x00000017);
	// //SETREG32(DDRC_BASE+0x0204 ,0x00000017);
	SETREG32(DDRC_BASE+0x0204 ,0x00000017);
	// //SETREG32(DDRC_BASE+0x0208 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x0208 ,0x00000500);
	// //SETREG32(DDRC_BASE+0x0208 ,0x00060500);
	SETREG32(DDRC_BASE+0x0208 ,0x00060500);
	// //SETREG32(DDRC_BASE+0x020c ,0x00000000);
	// //SETREG32(DDRC_BASE+0x020c ,0x00000000);
	// //SETREG32(DDRC_BASE+0x020c ,0x00030000);
	SETREG32(DDRC_BASE+0x020c ,0x04030000);
	// //SETREG32(DDRC_BASE+0x0210 ,0x0000000f);
	SETREG32(DDRC_BASE+0x0210 ,0x00000f0f);
	// //SETREG32(DDRC_BASE+0x0214 ,0x0000000b);
	// //SETREG32(DDRC_BASE+0x0214 ,0x0000020b);
	// //SETREG32(DDRC_BASE+0x0214 ,0x000f020b);
	SETREG32(DDRC_BASE+0x0214 ,0x0a0f020b);
	// //SETREG32(DDRC_BASE+0x0218 ,0x0000000a);
	// //SETREG32(DDRC_BASE+0x0218 ,0x0000040a);
	// //SETREG32(DDRC_BASE+0x0218 ,0x0006040a);
	// //SETREG32(DDRC_BASE+0x0218 ,0x0f06040a);
	SETREG32(DDRC_BASE+0x0218 ,0x8f06040a);
	// //SETREG32(DDRC_BASE+0x0224 ,0x00000006);
	// //SETREG32(DDRC_BASE+0x0224 ,0x00000a06);
	// //SETREG32(DDRC_BASE+0x0224 ,0x00060a06);
	SETREG32(DDRC_BASE+0x0224 ,0x09060a06);
	// //SETREG32(DDRC_BASE+0x0228 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x0228 ,0x00000500);
	// //SETREG32(DDRC_BASE+0x0228 ,0x00010500);
	SETREG32(DDRC_BASE+0x0228 ,0x06010500);
	SETREG32(DDRC_BASE+0x022c ,0x00000008);
	//cs 0   : 0x200[4:0]    6	22 16
	//cs 1   : 0x200[12:8]   7	22 16
	//bank 0 : 0x204 [4:0]   2	23 17	25
	//bank 1 : 0x204 [12:8]  3	23 17	26
	//bank 2 : 0x204 [20:16] 4	23 17	27
	//col 2  : 0x208 [3:0]   2	0	2
	//col 3  : 0x208 [11:8]  3	0	3
	//col 4  : 0x208 [19:16] 4	0	4
	//col 5  : 0x208 [27:24] 5	0	5
	//col 6  : 0x20c [3:0]   6	0	6
	//col 7  : 0x20c [11:8]  7	0	7
	//col 8  : 0x20c [19:16] 8	0	8
	//col 9  : 0x20c [27:24] 9	0	9
	//col 10 : 0x210 [3:0]   10	f
	//col 11 : 0x210 [11:8]  11	f
	//row 0  : 0x214 [3:0]   6	4	10
	//row 1  : 0x214 [11:8]  7	4	11
	//row 2  : 0x214 [19:16] 8
	//row 11 : 0x214 [27:24] 17	4	21
	//row 12 : 0x218 [3:0]   18	4	22
	//row 13 : 0x218 [11:8]  19	4	23
	//row 14 : 0x218 [19:16] 20	4	24
	//row 15 : 0x218 [27:24] 21	f

	//row 2  : 0x224 [3:0]   8	4	12
	//row 3  : 0x224 [11:8]  9	4	13
	//row 4  : 0x224 [19:16] 10	4	14
	//row 5  : 0x224 [27:24] 11	4	15
	//row 6  : 0x228 [3:0]   12	4	16
	//row 7  : 0x228 [11:8]  13	4	17
	//row 8  : 0x228 [19:16] 14	4	18
	//row 9  : 0x228 [27:24] 15	4	19

	SETREG32(DDRC_BASE+0x0200, 0x00000016);//cs
	SETREG32(DDRC_BASE+0x0204, 0x00080808);//bank 2 1 0
	SETREG32(DDRC_BASE+0x0208, 0x00000000);//col 5 4 3 2
	SETREG32(DDRC_BASE+0x020c, 0x00000000);//col 9 8 7 6
	SETREG32(DDRC_BASE+0x0210, 0x00000f0f);//col 11 10
	SETREG32(DDRC_BASE+0x0214, 0x07070707);//row 11 10 1  0
	SETREG32(DDRC_BASE+0x0218, 0x0f070707);//row 15 14 13 12
	SETREG32(DDRC_BASE+0x0224, 0x07070707);//row 5  4  3  2
	SETREG32(DDRC_BASE+0x0228, 0x07070707);//row 9  8  7  6
	//======================================================================
	// //SETREG32(DDRC_BASE+0x0240 ,0x04000458);
	// //SETREG32(DDRC_BASE+0x0240 ,0x04000e58);
	// //SETREG32(DDRC_BASE+0x0240 ,0x041b0e58);
	SETREG32(DDRC_BASE+0x0240 ,0x041b0e58);
	// //SETREG32(DDRC_BASE+0x0244 ,0x00002210);
	// //SETREG32(DDRC_BASE+0x0244 ,0x00002230);
	// //SETREG32(DDRC_BASE+0x0244 ,0x00002130);
	SETREG32(DDRC_BASE+0x0244 ,0x00002130);
	// //SETREG32(DDRC_BASE+0x0250 ,0x00001c05);
	// //SETREG32(DDRC_BASE+0x0250 ,0x00001c05);
	// //SETREG32(DDRC_BASE+0x0250 ,0x00001c01);
	// //SETREG32(DDRC_BASE+0x0250 ,0x00001c01);
	// //SETREG32(DDRC_BASE+0x0250 ,0x005c1c01);
	SETREG32(DDRC_BASE+0x0250 ,0x795c1c01);
	SETREG32(DDRC_BASE+0x0254 ,0x0000001d);
	// //SETREG32(DDRC_BASE+0x025c ,0x0f00ac4f);
	SETREG32(DDRC_BASE+0x025c ,0x4400ac4f);
	// //SETREG32(DDRC_BASE+0x0264 ,0x0f00e165);
	SETREG32(DDRC_BASE+0x0264 ,0x3200e165);
	// //SETREG32(DDRC_BASE+0x026c ,0x0f0037e7);
	SETREG32(DDRC_BASE+0x026c ,0x7f0037e7);
	// //SETREG32(DDRC_BASE+0x0300 ,0x00000000);
	SETREG32(DDRC_BASE+0x0300 ,0x00000010);
	// //SETREG32(DDRC_BASE+0x0304 ,0x00000000);
	SETREG32(DDRC_BASE+0x0304 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x030c ,0x00000000);
	// //SETREG32(DDRC_BASE+0x030c ,0x00000000);
	// //SETREG32(DDRC_BASE+0x030c ,0x00000000);
	SETREG32(DDRC_BASE+0x030c ,0x00000000);
	SETREG32(DDRC_BASE+0x0320 ,0x00000001);
	// //SETREG32(DDRC_BASE+0x036c ,0x00110011);
	// //SETREG32(DDRC_BASE+0x036c ,0x00110011);
	// //SETREG32(DDRC_BASE+0x036c ,0x00110011);
	// //SETREG32(DDRC_BASE+0x036c ,0x00100011);
	// //SETREG32(DDRC_BASE+0x036c ,0x00100011);
	SETREG32(DDRC_BASE+0x036c ,0x00100011);
	// //SETREG32(DDRC_BASE+0x2020 ,0x00000000);
	// //SETREG32(DDRC_BASE+0x2020 ,0x00000000);
	SETREG32(DDRC_BASE+0x2020 ,0x00000000);
	SETREG32(DDRC_BASE+0x2024 ,0xf15f7793);
	// //SETREG32(DDRC_BASE+0x2050 ,0x00210000);
	// //SETREG32(DDRC_BASE+0x2050 ,0x00210050);
	// //SETREG32(DDRC_BASE+0x2050 ,0x00217050);
	SETREG32(DDRC_BASE+0x2050 ,0x00d17050);
	// //SETREG32(DDRC_BASE+0x2064 ,0x0062002a);
	// //SETREG32(DDRC_BASE+0x2064 ,0x0062002a);
	SETREG32(DDRC_BASE+0x2064 ,0x002a002a);
	// //SETREG32(DDRC_BASE+0x20dc ,0x00000044);
	SETREG32(DDRC_BASE+0x20dc ,0x00830044);
	// //SETREG32(DDRC_BASE+0x20e0 ,0x00000000);
	SETREG32(DDRC_BASE+0x20e0 ,0x00020000);
	// //SETREG32(DDRC_BASE+0x2100 ,0x0f101b08);
	// //SETREG32(DDRC_BASE+0x2100 ,0x0f100c08);
	// //SETREG32(DDRC_BASE+0x2100 ,0x0f0a0c08);
	SETREG32(DDRC_BASE+0x2100 ,0x070a0608);
	// //SETREG32(DDRC_BASE+0x2104 ,0x0008040d);
	// //SETREG32(DDRC_BASE+0x2104 ,0x0008020d);
	SETREG32(DDRC_BASE+0x2104 ,0x0002020d);
	// //SETREG32(DDRC_BASE+0x2108 ,0x03050606);
	// //SETREG32(DDRC_BASE+0x2108 ,0x03050606);
	// //SETREG32(DDRC_BASE+0x2108 ,0x03030606);
	SETREG32(DDRC_BASE+0x2108 ,0x02030606);
	// //SETREG32(DDRC_BASE+0x210c ,0x00505000);
	SETREG32(DDRC_BASE+0x210c ,0x00505000);
	// //SETREG32(DDRC_BASE+0x2110 ,0x05040405);
	// //SETREG32(DDRC_BASE+0x2110 ,0x05040205);
	// //SETREG32(DDRC_BASE+0x2110 ,0x05020205);
	SETREG32(DDRC_BASE+0x2110 ,0x04020205);
	// //SETREG32(DDRC_BASE+0x2114 ,0x05050403);
	// //SETREG32(DDRC_BASE+0x2114 ,0x05050303);
	// //SETREG32(DDRC_BASE+0x2114 ,0x050d0303);
	SETREG32(DDRC_BASE+0x2114 ,0x0c0d0303);
	// //SETREG32(DDRC_BASE+0x2118 ,0x02020004);
	// //SETREG32(DDRC_BASE+0x2118 ,0x020f0004);
	SETREG32(DDRC_BASE+0x2118 ,0x020f0004);
	// //SETREG32(DDRC_BASE+0x211c ,0x00000209);
	SETREG32(DDRC_BASE+0x211c ,0x00000209);
	// //SETREG32(DDRC_BASE+0x2120 ,0x00004401);
	SETREG32(DDRC_BASE+0x2120 ,0x00002801);
	SETREG32(DDRC_BASE+0x2138 ,0x0000002c);
	// //SETREG32(DDRC_BASE+0x2180 ,0x02000012);
	// //SETREG32(DDRC_BASE+0x2180 ,0x00480012);
	// //SETREG32(DDRC_BASE+0x2180 ,0x00480012);
	// //SETREG32(DDRC_BASE+0x2180 ,0x40480012);
	SETREG32(DDRC_BASE+0x2180 ,0x40480012);
	// //SETREG32(DDRC_BASE+0x2190 ,0x07020001);
	// //SETREG32(DDRC_BASE+0x2190 ,0x07020101);
	// //SETREG32(DDRC_BASE+0x2190 ,0x07020101);
	// //SETREG32(DDRC_BASE+0x2190 ,0x07030101);
	// //SETREG32(DDRC_BASE+0x2190 ,0x07030101);
	SETREG32(DDRC_BASE+0x2190 ,0x02030101);
	// //SETREG32(DDRC_BASE+0x2194 ,0x00000402);
	// //SETREG32(DDRC_BASE+0x2194 ,0x00000202);
	SETREG32(DDRC_BASE+0x2194 ,0x00030202);
	// //SETREG32(DDRC_BASE+0x2240 ,0x04000458);
	// //SETREG32(DDRC_BASE+0x2240 ,0x04000e58);
	// //SETREG32(DDRC_BASE+0x2240 ,0x041b0e58);
	SETREG32(DDRC_BASE+0x2240 ,0x041b0e58);
	SETREG32(DDRC_BASE+0x0400 ,0x00000010);
	SETREG32(DDRC_BASE+0x0404 ,0x00005086);
	SETREG32(DDRC_BASE+0x0408 ,0x00004336);
	SETREG32(DDRC_BASE+0x0490 ,0x00000001);
	SETREG32(DDRC_BASE+0x0494 ,0x0021000a);
	SETREG32(DDRC_BASE+0x0498 ,0x0390019d);
	SETREG32(DDRC_BASE+0x049c ,0x00000001);
	SETREG32(DDRC_BASE+0x04a0 ,0x000002a0);
	SETREG32(DDRC_BASE+0x04b4 ,0x000061de);
	SETREG32(DDRC_BASE+0x04b8 ,0x00004293);
	SETREG32(DDRC_BASE+0x0544 ,0x00210000);
	SETREG32(DDRC_BASE+0x0548 ,0x009b01b9);
	SETREG32(DDRC_BASE+0x054c ,0x0011000a);
	SETREG32(DDRC_BASE+0x0550 ,0x000004f7);
	SETREG32(DDRC_BASE+0x0564 ,0x0000008c);
	SETREG32(DDRC_BASE+0x0568 ,0x0000735e);
	SETREG32(DDRC_BASE+0x05f4 ,0x00210006);
	SETREG32(DDRC_BASE+0x05f8 ,0x05e003ae);
	SETREG32(DDRC_BASE+0x05fc ,0x00000006);
	SETREG32(DDRC_BASE+0x0600 ,0x000003a2);
	SETREG32(DDRC_BASE+0x0614 ,0x00004268);
	SETREG32(DDRC_BASE+0x0618 ,0x0000238b);
	SETREG32(DDRC_BASE+0x06a0 ,0x00000001);
	SETREG32(DDRC_BASE+0x06a4 ,0x00110001);
	SETREG32(DDRC_BASE+0x06a8 ,0x001c0026);
	SETREG32(DDRC_BASE+0x06ac ,0x00010003);
	SETREG32(DDRC_BASE+0x06b0 ,0x00000035);
	SETREG32(DDRC_BASE+0x06c4 ,0x000042bf);
	SETREG32(DDRC_BASE+0x06c8 ,0x0000225b);
	SETREG32(DDRC_BASE+0x0750 ,0x00000001);
	SETREG32(DDRC_BASE+0x0754 ,0x00210002);
	SETREG32(DDRC_BASE+0x0758 ,0x004b0576);
	SETREG32(DDRC_BASE+0x075c ,0x0000000a);
	SETREG32(DDRC_BASE+0x0760 ,0x00000570);
	SETREG32(DDRC_BASE+0x0490 ,0x00000001);
	SETREG32(DDRC_BASE+0x0540 ,0x00000001);
	SETREG32(DDRC_BASE+0x05f0 ,0x00000001);
	SETREG32(DDRC_BASE+0x06a0 ,0x00000001);
	SETREG32(DDRC_BASE+0x0750 ,0x00000001);
	GETREG32(DDRC_BASE+0x0060);
}

void DDR_MCTL_init_2 (void)
{
	SETREG32(DDRC_BASE+0x0304, 0x00000000);
	// //GETREG32(DDRC_BASE+0x0030);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000002);
	// //GETREG32(DDRC_BASE+0x0030);
	//Debug--------------------------------
	SETREG32(DDRC_BASE+0x0030, 0x00000000);
	//-----------------------------------
	SETREG32(DDRC_BASE+0x0320, 0x00000000);
	SETREG32(DDRC_BASE+0x01b0, 0x00000000);
}

void DDR_MCTL_init_3 (void)
{
	unsigned int get_value,pgsr_ok;

	////
	SETREG32(DDRC_BASE+0x01b0, 0x00000001);
	SETREG32(DDRC_BASE+0x0320, 0x00000001);
	do {
		get_value = GETREG32(DDRC_BASE + 0x0324 );
		pgsr_ok =  get_value   & 0x1;
	} while( pgsr_ok != 0x1 );

	do {
		get_value = GETREG32(DDRC_BASE + 0x0004 );
		pgsr_ok =  get_value   & 0x1;
	} while( pgsr_ok != 0x1 );

	// //SETREG32(DDRC_BASE+0x0030, 0x00000000);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000002);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000002);
	SETREG32(DDRC_BASE+0x0030, 0x00000000);
	GETREG32(DDRC_BASE+0x0308);
	GETREG32(DDRC_BASE+0x0308);
}

void DDRPHY_init0 (void)
{
	unsigned int get_value,pgsr_ok;

	SETREG32(DDRP0_BASE+DCR,0x0000000d);
	SETREG32(DDRP0_BASE+MR0,0x00000852);
	SETREG32(DDRP0_BASE+MR1,0x00000083);
	SETREG32(DDRP0_BASE+MR2,0x00000004);

	//SETREG32(DDRP0_BASE+MR3,0x00000002);
	//Disable DDR ODT
	SETREG32(DDRP0_BASE+MR3,0x00000000);

	SETREG32(DDRP0_BASE+DTPR0,0x46918892);
	SETREG32(DDRP0_BASE+DTPR1,0x19341088);
	SETREG32(DDRP0_BASE+DTPR2,0x0647a0c8);
	SETREG32(DDRP0_BASE+PGCR,0x018c2e02);
	SETREG32(DDRP0_BASE+DXCCR, 0x00000c40);

	//SETREG32(DDRP0_BASE+DSGCR ,0xfa00025f);
	//For HDR mode
	SETREG32(DDRP0_BASE+DSGCR ,0xfa001a5f);

	get_value = GETREG32(DDRP0_BASE + DX0GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP0_BASE + DX0GCR, get_value);
	get_value = GETREG32(DDRP0_BASE + DX1GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP0_BASE + DX1GCR, get_value);
	get_value = GETREG32(DDRP0_BASE + DX2GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP0_BASE + DX2GCR, get_value);
	get_value = GETREG32(DDRP0_BASE + DX3GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP0_BASE + DX3GCR,  get_value);

	SETREG32(DDRP0_BASE+DX0DQSTR ,0x3db0f000);
	SETREG32(DDRP0_BASE+DX1DQSTR ,0x3db0f000);
	SETREG32(DDRP0_BASE+DX2DQSTR ,0x3db0f000);
	SETREG32(DDRP0_BASE+DX3DQSTR ,0x3db0f000);
	//-> 1103.000 ns: [BENCH] Polling register at address 0x3 on bits [0:0] for value 1 ...

	do {
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value   & 0x1;
	} while( pgsr_ok != 0x1 );

	get_value = 0;
	pgsr_ok = 0;
#if 0
	//-> 1893.000 ns: [BENCH] PHY initialization done...
	//-> 1892.812 ns: [CFG] Data out: Q = 00000007
	//
	SETREG32(DDRP0_BASE+PIR, 0x000000c1);
	//SETREG32(DDRP0_BASE+PIR, 0x00040001);
	//-> 1910.000 ns: [BENCH] Polling register at address 0x3 on bits [3:0] for value 15 ...
	do {
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value   & 0x1f;
	} while( pgsr_ok != 0x1f );
#else
	SETREG32(DDRP0_BASE+PTR0, 0x003FFFFF);
	SETREG32(DDRP0_BASE+PTR1, 0x07FFFFFF);
	//DLLSRST + DLLLOCK + ZCAL + ITMSRST
	SETREG32(DDRP0_BASE+PIR, 0x0000001F);
	do {
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value & 0x7;
	} while( pgsr_ok != 0x7 );

	//DRAMINIT
	SETREG32(DDRP0_BASE+PIR, 0x00000041);
	do {
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value & 0x1;
	} while( !pgsr_ok );

#  if 0
	//QSTRN
	SETREG32(DDRP0_BASE+PIR, 0x00000081);
	do {
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value & 0x1;
	} while( !pgsr_ok );
	//RVTRN
	SETREG32(DDRP0_BASE+PIR, 0x00000101);
	do {
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value & 0x1;
	} while( !pgsr_ok );
#  endif
#endif
	SETREG32(DDRP0_BASE+PIR, 0x00000011);
	do {
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value   & 0x1;
	} while( !pgsr_ok );
	udelay(15);

	//-> 5849.000 ns: [BENCH] PUB initialization done...
	//-> 5849.062 ns: [CFG] Data out: Q = 0000001f
	//-> 5860.312 ns: [SYS] END OF INITIALIZATION
}

void DDRPHY_init1 (void)
{
	unsigned int get_value,pgsr_ok;

	SETREG32(DDRP1_BASE+DCR,0x0000000d);
	SETREG32(DDRP1_BASE+MR0,0x00000852);
	SETREG32(DDRP1_BASE+MR1,0x00000083);
	SETREG32(DDRP1_BASE+MR2,0x00000004);

	//SETREG32(DDRP1_BASE+MR3,0x00000002);
	//Disable DDR ODT
	SETREG32(DDRP1_BASE+MR3,0x00000000);

	SETREG32(DDRP1_BASE+DTPR0,0x46918892);
	SETREG32(DDRP1_BASE+DTPR1,0x19341088);
	SETREG32(DDRP1_BASE+DTPR2,0x0647a0c8);
	SETREG32(DDRP1_BASE+PGCR,0x018c2e02);
	SETREG32(DDRP1_BASE+DXCCR, 0x00000c40);

	//SETREG32(DDRP1_BASE+DSGCR ,0xfa00025f);
	//For HDR mode
	SETREG32(DDRP1_BASE+DSGCR ,0xfa001a5f);

	get_value = GETREG32(DDRP1_BASE + DX0GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP1_BASE + DX0GCR,  get_value);
	get_value = GETREG32(DDRP1_BASE + DX1GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP1_BASE + DX1GCR,  get_value);
	get_value = GETREG32(DDRP1_BASE + DX2GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP1_BASE + DX2GCR,  get_value);
	get_value = GETREG32(DDRP1_BASE + DX3GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP1_BASE + DX3GCR,  get_value);

	SETREG32(DDRP1_BASE+DX0DQSTR ,0x3db0f000);
	SETREG32(DDRP1_BASE+DX1DQSTR ,0x3db0f000);
	SETREG32(DDRP1_BASE+DX2DQSTR ,0x3db0f000);
	SETREG32(DDRP1_BASE+DX3DQSTR ,0x3db0f000);
	//-> 1103.000 ns: [BENCH] Polling register at address 0x3 on bits [0:0] for value 1 ...

	do {
		  get_value = GETREG32( DDRP1_BASE + PGSR );
		  pgsr_ok =  get_value   & 0x1;
	} while( pgsr_ok != 0x1 );

	 get_value = 0;
	 pgsr_ok = 0;

#if 0
	//-> 1893.000 ns: [BENCH] PHY initialization done...
	//-> 1892.812 ns: [CFG] Data out: Q = 00000007
	//
	SETREG32(DDRP1_BASE+PIR, 0x000000c1);
	//SETREG32(DDRP1_BASE+PIR, 0x00040001);
	//-> 1910.000 ns: [BENCH] Polling register at address 0x3 on bits [3:0] for value 15 ...
	do {
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value   & 0x1f;
	} while( pgsr_ok != 0x1f );
#else
	SETREG32(DDRP1_BASE+PTR0, 0x003FFFFF);
	SETREG32(DDRP1_BASE+PTR1, 0x07FFFFFF);
	//DLLSRST + DLLLOCK + ZCAL + ITMSRST
	SETREG32(DDRP1_BASE+PIR, 0x0000001F);
	do {
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value   & 0x7;
	} while( pgsr_ok != 0x7 );

	//DRAMINIT
	SETREG32(DDRP1_BASE+PIR, 0x00000041);
	do {
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value   & 0x1;
	} while( !pgsr_ok );

#  if 0
	//QSTRN
	SETREG32(DDRP1_BASE+PIR, 0x00000081);
	do {
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value   & 0x1;
	} while( !pgsr_ok );
	//RVTRN
	SETREG32(DDRP1_BASE+PIR, 0x00000101);
	do {
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value   & 0x1;
	} while( pgsr_ok != 0x1 );
#  endif
#endif
	SETREG32(DDRP1_BASE+PIR, 0x00000011);
	do {
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value & 0x1;
	} while( !pgsr_ok );
	udelay(15);
	//-> 5849.000 ns: [BENCH] PUB initialization done...
	//-> 5849.062 ns: [CFG] Data out: Q = 0000001f
	//-> 5860.312 ns: [SYS] END OF INITIALIZATION
}
#ifndef LPDDR_TRAINING
#define LPDDR_TRAINING 1
#endif
int DDR_Init (int bp0p1)
{
//DDR CTRL INIT
	SETREG32(SCU_BASE + 0x3c, 0x00007f00);
	SETREG32(SCU_BASE + 0x44, 0x000c0000);
	//deassert DDRPHY core reset and DDRPHY APB reset
	SETREG32(SCU_BASE + 0x44, 0x00000000);
	SETREG32(SCU_BASE + 0x3c, 0x00007d00);
	udelay(20);
	DDR_MCTL_init();
	SETREG32(SCU_BASE + 0x3c, 0x00000000);
	DDR_MCTL_init_2();

	if (bp0p1) {
		//DDR PHY INIT
		//SETREG32(SCU_BASE + 0x44, 0x00080000);
		//wait_cycles(10);
		//udelay(12);
		DDRPHY_init0();

		//SETREG32(SCU_BASE + 0x44, 0x00000000);
		//wait_cycles(10);
		//udelay(12);
		DDRPHY_init1();
	}
	else {
		//DDR PHY INIT
		//SETREG32(SCU_BASE + 0x44, 0x00040000);
		//wait_cycles(10);
		//udelay(12);
		DDRPHY_init1();

		//SETREG32(SCU_BASE + 0x44, 0x00000000);
		//wait_cycles(10);
		//udelay(12);
		DDRPHY_init0();
	}
	DDR_MCTL_init_3();

#if (LPDDR_TRAINING == 1)
	if (prvSW_DQS_GW_Training(DDRP0_BASE))
		return -1;
	if (prvSW_DQS_GW_Training(DDRP1_BASE))
		return -1;
	if (prvSW_DQS_Training(DDRP0_BASE))
		return -1;
	if (prvSW_DQS_Training(DDRP1_BASE))
		return -1;
#endif

	return 0;
}

#else

#define DDRPHYPIR(v)					\
({							\
	uint32_t val;					\
	SETREG32(DDRP0_BASE + PIR, (v));		\
	SETREG32(DDRP1_BASE + PIR, (v));		\
	do {						\
		val = GETREG32( DDRP0_BASE + PGSR );	\
	} while( !(val & 0x1) );			\
	do {						\
		val = GETREG32( DDRP1_BASE + PGSR );	\
	} while( !(val & 0x1) );			\
 })

#define DDRPHYMASKWRITE(r, v, m)			\
({							\
	uint32_t val;					\
	val = GETREG32(DDRP0_BASE + r);			\
	val &= ~(m);					\
	val |= (v);					\
	SETREG32(DDRP0_BASE + r, val);			\
	val = GETREG32(DDRP1_BASE + r);			\
	val &= ~(m);					\
	val |= (v);					\
	SETREG32(DDRP1_BASE + r, val);			\
 })
static uint32_t DXDQSTR[2][4] = {
	{0x3db00001, 0x3db00001, 0x3db00001, 0x3db00001},
	{0x3db00001, 0x3db00001, 0x3db00001, 0x3db00001},
};
static void DDR_Update_TrainingValue(void)
{
	for(int i = 0; i < 2; i++) {
		for(int j = 0; j < 4; j++) {
			DXDQSTR[i][j] = GETREG32(DDRP0_BASE + i * 0x1000 + DX0DQSTR + 0x40 * j);
		}
	}
}
int DDR_TEST(void)
{
	const uint32_t pattern[4] = {
		0x55555555,
		0xaaaaaaaa,
		0x22222222,
		0xcccccccc,
	};
	uint32_t rd_data[4];

	prvWrite4W(0x04000000, pattern);
	prvWrite4W(0x04000010, pattern);

	for (int i = 0; i < 2; i++) {
		prvRead4W(0x04000000 + 0x10 * i, rd_data);
		for (int j = 0; j < 4; j++) {
			if (rd_data[j] != pattern[j])
				return -1;
		}
	}
	return 0;
}

void DDR_MCTL_init (void)
{
	SETREG32(DDRC_BASE+0x0304, 0x00000001);
	SETREG32(DDRC_BASE+0x0030, 0x00000001);
	// //GETREG32(DDRC_BASE+0x0004);
	// //SETREG32(DDRC_BASE+0x0000, 0x03040000);
	// //SETREG32(DDRC_BASE+0x0000, 0x03040008);
	// //SETREG32(DDRC_BASE+0x0000, 0x03040008);
	// //SETREG32(DDRC_BASE+0x0000, 0x03040008);
	// //SETREG32(DDRC_BASE+0x0000, 0x03040008);
	// //SETREG32(DDRC_BASE+0x0000, 0x03040008);
	SETREG32(DDRC_BASE+0x0000, 0x03040008);
	// //SETREG32(DDRC_BASE+0x0010, 0x00000030);
	// //SETREG32(DDRC_BASE+0x0010, 0x00000030);
	// //SETREG32(DDRC_BASE+0x0010, 0x00003030);
	SETREG32(DDRC_BASE+0x0010, 0x00003030);
	SETREG32(DDRC_BASE+0x0014, 0x0000a630);
	// //SETREG32(DDRC_BASE+0x0020, 0x00000001);
	// //SETREG32(DDRC_BASE+0x0020, 0x00000001);
	SETREG32(DDRC_BASE+0x0020, 0x00000041);
	SETREG32(DDRC_BASE+0x0024, 0x49af1c53);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000000);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000000);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000000);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000008);
	SETREG32(DDRC_BASE+0x0030, 0x00000008);
	// //SETREG32(DDRC_BASE+0x0034, 0x00402000);
	// //SETREG32(DDRC_BASE+0x0034, 0x00404900);
	SETREG32(DDRC_BASE+0x0034, 0x000d4900);
	// //SETREG32(DDRC_BASE+0x0038, 0x00000002);
	// //SETREG32(DDRC_BASE+0x0038, 0x00000000);
	SETREG32(DDRC_BASE+0x0038, 0x00a70000);
	// //SETREG32(DDRC_BASE+0x0050, 0x00210000);
	// //SETREG32(DDRC_BASE+0x0050, 0x00210070);
	// //SETREG32(DDRC_BASE+0x0050, 0x0021f070);
	SETREG32(DDRC_BASE+0x0050, 0x0071f070);
	// //SETREG32(DDRC_BASE+0x0060, 0x00000000);
	SETREG32(DDRC_BASE+0x0060, 0x00000000);
	// //SETREG32(DDRC_BASE+0x0064, 0x00620038);
	// //SETREG32(DDRC_BASE+0x0064, 0x00620038);
	SETREG32(DDRC_BASE+0x0064, 0x00200023);
	//SETREG32(DDRC_BASE+0x0064, 0x0006000B);
	// //SETREG32(DDRC_BASE+0x00c0, 0x00000000);
	// //SETREG32(DDRC_BASE+0x00c0, 0x00000000);
	SETREG32(DDRC_BASE+0x00c0, 0x00000000);
	// //SETREG32(DDRC_BASE+0x00d0, 0x00020003);
	// //SETREG32(DDRC_BASE+0x00d0, 0x00030003);
	SETREG32(DDRC_BASE+0x00d0, 0x00030003);
	SETREG32(DDRC_BASE+0x00d4, 0x0000000f);
	// //SETREG32(DDRC_BASE+0x00d8, 0x00000d06);
	SETREG32(DDRC_BASE+0x00d8, 0x00000a06);
	// //SETREG32(DDRC_BASE+0x00dc, 0x00000046);
	SETREG32(DDRC_BASE+0x00dc, 0x00c30046);
	// //SETREG32(DDRC_BASE+0x00e0, 0x00000000);
	SETREG32(DDRC_BASE+0x00e0, 0x00020000);
	// //SETREG32(DDRC_BASE+0x00e4, 0x00100004);
	SETREG32(DDRC_BASE+0x00e4, 0x000a0004);
	// //SETREG32(DDRC_BASE+0x00f0, 0x00000000);
	SETREG32(DDRC_BASE+0x00f0, 0x00000000);
	// //SETREG32(DDRC_BASE+0x00f4, 0x0000066e);
	// //SETREG32(DDRC_BASE+0x00f4, 0x0000068e);
	SETREG32(DDRC_BASE+0x00f4, 0x0000098e);
	// //SETREG32(DDRC_BASE+0x0100, 0x0f101b0b);
	// //SETREG32(DDRC_BASE+0x0100, 0x0f10110b);
	// //SETREG32(DDRC_BASE+0x0100, 0x0f0e110b);
	SETREG32(DDRC_BASE+0x0100, 0x080e060b);
	// //SETREG32(DDRC_BASE+0x0104, 0x00080413);
	// //SETREG32(DDRC_BASE+0x0104, 0x00080213);
	SETREG32(DDRC_BASE+0x0104, 0x00020213);
	// //SETREG32(DDRC_BASE+0x0108, 0x03050607);
	// //SETREG32(DDRC_BASE+0x0108, 0x03050607);
	// //SETREG32(DDRC_BASE+0x0108, 0x03040607);
	SETREG32(DDRC_BASE+0x0108, 0x02040607);
	// //SETREG32(DDRC_BASE+0x010c, 0x00505000);
	SETREG32(DDRC_BASE+0x010c, 0x00505000);
	// //SETREG32(DDRC_BASE+0x0110, 0x05040407);
	// //SETREG32(DDRC_BASE+0x0110, 0x05040307);
	// //SETREG32(DDRC_BASE+0x0110, 0x05020307);
	SETREG32(DDRC_BASE+0x0110, 0x07020307);
	// //SETREG32(DDRC_BASE+0x0114, 0x05050404);
	// //SETREG32(DDRC_BASE+0x0114, 0x05050404);
	// //SETREG32(DDRC_BASE+0x0114, 0x05070404);
	SETREG32(DDRC_BASE+0x0114, 0x08070404);
	// //SETREG32(DDRC_BASE+0x0118, 0x02020006);
	// //SETREG32(DDRC_BASE+0x0118, 0x020a0006);
	SETREG32(DDRC_BASE+0x0118, 0x0a0a0006);
	// //SETREG32(DDRC_BASE+0x011c, 0x0000020e);
	SETREG32(DDRC_BASE+0x011c, 0x0000070e);
	// //SETREG32(DDRC_BASE+0x0120, 0x00004401);
	SETREG32(DDRC_BASE+0x0120, 0x00002901);
	SETREG32(DDRC_BASE+0x0138, 0x0000003b);
	// //SETREG32(DDRC_BASE+0x0180, 0x02000018);
	// //SETREG32(DDRC_BASE+0x0180, 0x00600018);
	// //SETREG32(DDRC_BASE+0x0180, 0x00600018);
	// //SETREG32(DDRC_BASE+0x0180, 0x40600018);
	SETREG32(DDRC_BASE+0x0180, 0x40600018);
	// //SETREG32(DDRC_BASE+0x0184, 0x02000070);
	SETREG32(DDRC_BASE+0x0184, 0x00e00070);
	SETREG32(DDRC_BASE+0x0188, 0x00000000);
	// //SETREG32(DDRC_BASE+0x0190, 0x07020002);
	// //SETREG32(DDRC_BASE+0x0190, 0x07020102);
	// //SETREG32(DDRC_BASE+0x0190, 0x07020102);
	// //SETREG32(DDRC_BASE+0x0190, 0x07040102);
	// //SETREG32(DDRC_BASE+0x0190, 0x07040102);
	//SETREG32(DDRC_BASE+0x0190, 0x02040102);
	//SETREG32(DDRC_BASE+0x0190, 0x02040002);
	SETREG32(DDRC_BASE+0x0190, 0x02030002);
	// //SETREG32(DDRC_BASE+0x0194, 0x00000402);
	// //SETREG32(DDRC_BASE+0x0194, 0x00000202);
	SETREG32(DDRC_BASE+0x0194, 0x00030202);
	// //SETREG32(DDRC_BASE+0x0198, 0x07000000);
	// //SETREG32(DDRC_BASE+0x0198, 0x07000001);
	// //SETREG32(DDRC_BASE+0x0198, 0x07000001);
	// //SETREG32(DDRC_BASE+0x0198, 0x07000001);
	// //SETREG32(DDRC_BASE+0x0198, 0x07009001);
	// //SETREG32(DDRC_BASE+0x0198, 0x07719001);
	SETREG32(DDRC_BASE+0x0198, 0x07719001);
	// //SETREG32(DDRC_BASE+0x01a0, 0x00400005);
	// //SETREG32(DDRC_BASE+0x01a0, 0x00400005);
	// //SETREG32(DDRC_BASE+0x01a0, 0x20400005);
	// //SETREG32(DDRC_BASE+0x01a0, 0x60400005);
	SETREG32(DDRC_BASE+0x01a0, 0x60400005);
	// //SETREG32(DDRC_BASE+0x01a4, 0x000100d8);
	SETREG32(DDRC_BASE+0x01a4, 0x003000d8);
	SETREG32(DDRC_BASE+0x01a8, 0x00000000);
	// //SETREG32(DDRC_BASE+0x01b0, 0x00000001);
	// //SETREG32(DDRC_BASE+0x01b0, 0x00000001);
	// //SETREG32(DDRC_BASE+0x01b0, 0x00000001);
	SETREG32(DDRC_BASE+0x01b0, 0x00000001);
	SETREG32(DDRC_BASE+0x0200, 0x00000000);
	// //SETREG32(DDRC_BASE+0x0204, 0x00000003);
	// //SETREG32(DDRC_BASE+0x0204, 0x00000b03);
	SETREG32(DDRC_BASE+0x0204, 0x00070b14);
	// //SETREG32(DDRC_BASE+0x0208, 0x00000000);
	// //SETREG32(DDRC_BASE+0x0208, 0x00000600);
	// //SETREG32(DDRC_BASE+0x0208, 0x00040600);
	SETREG32(DDRC_BASE+0x0208, 0x07040600);
	// //SETREG32(DDRC_BASE+0x020c, 0x00000001);
	// //SETREG32(DDRC_BASE+0x020c, 0x00000701);
	// //SETREG32(DDRC_BASE+0x020c, 0x00050701);
	SETREG32(DDRC_BASE+0x020c, 0x02050701);
	// //SETREG32(DDRC_BASE+0x0210, 0x0000000f);
	SETREG32(DDRC_BASE+0x0210, 0x00000f0f);
	// //SETREG32(DDRC_BASE+0x0214, 0x00000009);
	// //SETREG32(DDRC_BASE+0x0214, 0x00000309);
	// //SETREG32(DDRC_BASE+0x0214, 0x00080309);
	SETREG32(DDRC_BASE+0x0214, 0x0b080309);
	// //SETREG32(DDRC_BASE+0x0218, 0x00000009);
	// //SETREG32(DDRC_BASE+0x0218, 0x00000709);
	// //SETREG32(DDRC_BASE+0x0218, 0x00050709);
	// //SETREG32(DDRC_BASE+0x0218, 0x0f050709);
	SETREG32(DDRC_BASE+0x0218, 0x0f050709);
	// //SETREG32(DDRC_BASE+0x0224, 0x00000007);
	// //SETREG32(DDRC_BASE+0x0224, 0x00000a07);
	// //SETREG32(DDRC_BASE+0x0224, 0x00080a07);
	SETREG32(DDRC_BASE+0x0224, 0x00080a07);
	// //SETREG32(DDRC_BASE+0x0228, 0x00000004);
	// //SETREG32(DDRC_BASE+0x0228, 0x00000104);
	// //SETREG32(DDRC_BASE+0x0228, 0x00080104);
	SETREG32(DDRC_BASE+0x0228, 0x05080104);
	SETREG32(DDRC_BASE+0x022c, 0x00000000);

	//cs 0   : 0x200[4:0]    6 22 16
	//cs 1   : 0x200[12:8]   7 22 16
	//bank 0 : 0x204 [4:0]   2 23 17 25
	//bank 1 : 0x204 [12:8]  3 23 17 26
	//bank 2 : 0x204 [20:16] 4 23 17 27
	//col 2  : 0x208 [3:0]   2 0 2
	//col 3  : 0x208 [11:8]  3 0 3
	//col 4  : 0x208 [19:16] 4 0 4
	//col 5  : 0x208 [27:24] 5 0 5
	//col 6  : 0x20c [3:0]   6 0 6
	//col 7  : 0x20c [11:8]  7 0 7
	//col 8  : 0x20c [19:16] 8 0 8
	//col 9  : 0x20c [27:24] 9 0 9
	//col 10 : 0x210 [3:0]   10  f
	//col 11 : 0x210 [11:8]  11  f
	//row 0  : 0x214 [3:0]   6 4 10
	//row 1  : 0x214 [11:8]  7 4 11
	//row 2  : 0x214 [19:16] 8
	//row 11 : 0x214 [27:24] 17  4 21
	//row 12 : 0x218 [3:0]   18  4 22
	//row 13 : 0x218 [11:8]  19  4 23
	//row 14 : 0x218 [19:16] 20  4 24
	//row 15 : 0x218 [27:24] 21  f

	//row 2  : 0x224 [3:0]   8 4 12
	//row 3  : 0x224 [11:8]  9 4 13
	//row 4  : 0x224 [19:16] 10  4 14
	//row 5  : 0x224 [27:24] 11  4 15
	//row 6  : 0x228 [3:0]   12  4 16
	//row 7  : 0x228 [11:8]  13  4 17
	//row 8  : 0x228 [19:16] 14  4 18
	//row 9  : 0x228 [27:24] 15  4 19
#if 0
	SETREG32(DDRC_BASE+0x0200, 0x00000016);//cs
#  ifdef CONFIG_FPGA
	SETREG32(DDRC_BASE+0x0204, 0x00090909);//bank 2 1 0
#  else
	SETREG32(DDRC_BASE+0x0204, 0x00171717);//bank 2 1 0
#  endif
	SETREG32(DDRC_BASE+0x0208, 0x00000000);//col 5 4 3 2
	SETREG32(DDRC_BASE+0x020c, 0x00000000);//col 9 8 7 6
#  ifdef CONFIG_FPGA
	SETREG32(DDRC_BASE+0x0210, 0x00000f00);//col 11 10
	SETREG32(DDRC_BASE+0x0214, 0x08080808);//row 11 10 1  0
	SETREG32(DDRC_BASE+0x0218, 0x0f0f0808);//row 15 14 13 12
	//SETREG32(DDRC_BASE+0x0224, 0x04040404);//row 5  4  3  2
	//SETREG32(DDRC_BASE+0x0228, 0x04040404);//row 9  8  7  6
#  else
	SETREG32(DDRC_BASE+0x0210, 0x00000f0f);//col 11 10
	SETREG32(DDRC_BASE+0x0214, 0x04040404);//row 11 10 1  0
	SETREG32(DDRC_BASE+0x0218, 0x0f040404);//row 15 14 13 12
	SETREG32(DDRC_BASE+0x0224, 0x04040404);//row 5  4  3  2
	SETREG32(DDRC_BASE+0x0228, 0x04040404);//row 9  8  7  6
#  endif
#endif

#if 0
	SETREG32(DDRC_BASE+0x0200, 0x00000016);//cs
	SETREG32(DDRC_BASE+0x0204, 0x00080808);//bank 2 1 0
	SETREG32(DDRC_BASE+0x0208, 0x00000000);//col 5 4 3 2
	SETREG32(DDRC_BASE+0x020c, 0x00000000);//col 9 8 7 6
	SETREG32(DDRC_BASE+0x0210, 0x00000f0f);//col 11 10
	SETREG32(DDRC_BASE+0x0214, 0x07070707);//row 11 10 1  0
	SETREG32(DDRC_BASE+0x0218, 0x0f070707);//row 15 14 13 12
	SETREG32(DDRC_BASE+0x0224, 0x07070707);//row 5  4  3  2
	SETREG32(DDRC_BASE+0x0228, 0x07070707);//row 9  8  7  6
#endif

	printf("\r\n Setting DDR banking size to 16MB \r\n");
	SETREG32(DDRC_BASE+0x0200, 0x00000016);//cs
	SETREG32(DDRC_BASE+0x0204, 0x00131313);//bank 2 1 0
	SETREG32(DDRC_BASE+0x0208, 0x00000000);//col 5 4 3 2
	SETREG32(DDRC_BASE+0x020c, 0x00000000);//col 9 8 7 6
	SETREG32(DDRC_BASE+0x0210, 0x00000f0f);//col 11 10
	SETREG32(DDRC_BASE+0x0214, 0x07040404);//row 11 10 1  0
	SETREG32(DDRC_BASE+0x0218, 0x0f0f0707);//row 15 14 13 12
	SETREG32(DDRC_BASE+0x0224, 0x04040404);//row 5  4  3  2
	SETREG32(DDRC_BASE+0x0228, 0x04040404);//row 9  8  7  6

	// //SETREG32(DDRC_BASE+0x0240, 0x04000440);
	// //SETREG32(DDRC_BASE+0x0240, 0x04000c40);
	// //SETREG32(DDRC_BASE+0x0240, 0x04120c40);
	SETREG32(DDRC_BASE+0x0240, 0x0c120c40);
	// //SETREG32(DDRC_BASE+0x0244, 0x00002212);
	// //SETREG32(DDRC_BASE+0x0244, 0x00002232);
	// //SETREG32(DDRC_BASE+0x0244, 0x00002132);
	SETREG32(DDRC_BASE+0x0244, 0x00000132);
	// //SETREG32(DDRC_BASE+0x0250, 0x00000905);
	// //SETREG32(DDRC_BASE+0x0250, 0x00000905);
	// //SETREG32(DDRC_BASE+0x0250, 0x00000905);
	// //SETREG32(DDRC_BASE+0x0250, 0x00000905);
	// //SETREG32(DDRC_BASE+0x0250, 0x00f20905);
	SETREG32(DDRC_BASE+0x0250, 0x07f20905);
	SETREG32(DDRC_BASE+0x0254, 0x0000001e);
	// //SETREG32(DDRC_BASE+0x025c, 0x0f00d6a1);
	SETREG32(DDRC_BASE+0x025c, 0x1300d6a1);
	// //SETREG32(DDRC_BASE+0x0264, 0x0f00cfbe);
	SETREG32(DDRC_BASE+0x0264, 0xd000cfbe);
	// //SETREG32(DDRC_BASE+0x026c, 0x0f00e287);
	SETREG32(DDRC_BASE+0x026c, 0x1600e287);
	// //SETREG32(DDRC_BASE+0x0300, 0x00000001);
	SETREG32(DDRC_BASE+0x0300, 0x00000011);
	// //SETREG32(DDRC_BASE+0x0304, 0x00000000);
	SETREG32(DDRC_BASE+0x0304, 0x00000000);
	// //SETREG32(DDRC_BASE+0x030c, 0x00000000);
	// //SETREG32(DDRC_BASE+0x030c, 0x00000000);
	SETREG32(DDRC_BASE+0x030c, 0x00000000);
	SETREG32(DDRC_BASE+0x0320, 0x00000001);
	// //SETREG32(DDRC_BASE+0x036c, 0x00110010);
	// //SETREG32(DDRC_BASE+0x036c, 0x00110000);
	// //SETREG32(DDRC_BASE+0x036c, 0x00110000);
	// //SETREG32(DDRC_BASE+0x036c, 0x00100000);
	// //SETREG32(DDRC_BASE+0x036c, 0x00000000);
	SETREG32(DDRC_BASE+0x036c, 0x00000000);

	// //SETREG32(DDRC_BASE+0x2020, 0x00000001);
	// //SETREG32(DDRC_BASE+0x2020, 0x00000001);
	SETREG32(DDRC_BASE+0x2020, 0x00000041);
	SETREG32(DDRC_BASE+0x2024, 0x49af1c53);
	// //SETREG32(DDRC_BASE+0x2050, 0x00210070);
	// //SETREG32(DDRC_BASE+0x2050, 0x00210070);
	// //SETREG32(DDRC_BASE+0x2050, 0x0021f070);
	SETREG32(DDRC_BASE+0x2050, 0x0071f070);
	// //SETREG32(DDRC_BASE+0x2064, 0x00620038);
	// //SETREG32(DDRC_BASE+0x2064, 0x00628038);
	SETREG32(DDRC_BASE+0x2064, 0x00380038);
	// //SETREG32(DDRC_BASE+0x20dc, 0x00000046);
	SETREG32(DDRC_BASE+0x20dc, 0x00c30046);
	// //SETREG32(DDRC_BASE+0x20e0, 0x00000000);
	SETREG32(DDRC_BASE+0x20e0, 0x00020000);
	// //SETREG32(DDRC_BASE+0x2100, 0x0f101b0b);
	// //SETREG32(DDRC_BASE+0x2100, 0x0f10110b);
	// //SETREG32(DDRC_BASE+0x2100, 0x0f0e110b);
	SETREG32(DDRC_BASE+0x2100, 0x080e060b);
	// //SETREG32(DDRC_BASE+0x2104, 0x00080413);
	// //SETREG32(DDRC_BASE+0x2104, 0x00080213);
	SETREG32(DDRC_BASE+0x2104, 0x00020213);
	// //SETREG32(DDRC_BASE+0x2108, 0x03050607);
	// //SETREG32(DDRC_BASE+0x2108, 0x03050607);
	// //SETREG32(DDRC_BASE+0x2108, 0x03040607);
	SETREG32(DDRC_BASE+0x2108, 0x02040607);
	// //SETREG32(DDRC_BASE+0x210c, 0x00505000);
	SETREG32(DDRC_BASE+0x210c, 0x00505000);
	// //SETREG32(DDRC_BASE+0x2110, 0x05040407);
	// //SETREG32(DDRC_BASE+0x2110, 0x05040307);
	// //SETREG32(DDRC_BASE+0x2110, 0x05020307);
	SETREG32(DDRC_BASE+0x2110, 0x07020307);
	// //SETREG32(DDRC_BASE+0x2114, 0x05050404);
	// //SETREG32(DDRC_BASE+0x2114, 0x05050404);
	// //SETREG32(DDRC_BASE+0x2114, 0x05070404);
	SETREG32(DDRC_BASE+0x2114, 0x08070404);
	// //SETREG32(DDRC_BASE+0x2118, 0x02020006);
	// //SETREG32(DDRC_BASE+0x2118, 0x020a0006);
	SETREG32(DDRC_BASE+0x2118, 0x0a0a0006);
	// //SETREG32(DDRC_BASE+0x211c, 0x0000020e);
	SETREG32(DDRC_BASE+0x211c, 0x0000070e);
	// //SETREG32(DDRC_BASE+0x2120, 0x00004401);
	SETREG32(DDRC_BASE+0x2120, 0x00002901);
	SETREG32(DDRC_BASE+0x2138, 0x0000003b);
	// //SETREG32(DDRC_BASE+0x2180, 0x02000018);
	// //SETREG32(DDRC_BASE+0x2180, 0x00600018);
	// //SETREG32(DDRC_BASE+0x2180, 0x00600018);
	// //SETREG32(DDRC_BASE+0x2180, 0x40600018);
	SETREG32(DDRC_BASE+0x2180, 0x40600018);
	// //SETREG32(DDRC_BASE+0x2190, 0x07020002);
	// //SETREG32(DDRC_BASE+0x2190, 0x07020102);
	// //SETREG32(DDRC_BASE+0x2190, 0x07020102);
	// //SETREG32(DDRC_BASE+0x2190, 0x07040102);
	// //SETREG32(DDRC_BASE+0x2190, 0x07040102);
	SETREG32(DDRC_BASE+0x2190, 0x02040102);
	// //SETREG32(DDRC_BASE+0x2194, 0x00000402);
	// //SETREG32(DDRC_BASE+0x2194, 0x00000202);
	SETREG32(DDRC_BASE+0x2194, 0x00030202);
	// //SETREG32(DDRC_BASE+0x2240, 0x04000440);
	// //SETREG32(DDRC_BASE+0x2240, 0x04000c40);
	// //SETREG32(DDRC_BASE+0x2240, 0x04120c40);
	SETREG32(DDRC_BASE+0x2240, 0x0c120c40);

	SETREG32(DDRC_BASE+0x0400, 0x00000010);
	SETREG32(DDRC_BASE+0x0404, 0x000000da);
	SETREG32(DDRC_BASE+0x0408, 0x00006201);
	SETREG32(DDRC_BASE+0x0494, 0x00200007);
	SETREG32(DDRC_BASE+0x04b4, 0x0000119b);
	SETREG32(DDRC_BASE+0x04b8, 0x00001144);
	SETREG32(DDRC_BASE+0x0544, 0x0010000e);
	SETREG32(DDRC_BASE+0x0564, 0x00000156);
	SETREG32(DDRC_BASE+0x0568, 0x00005139);
	SETREG32(DDRC_BASE+0x05f4, 0x00110006);
	SETREG32(DDRC_BASE+0x0614, 0x00004223);
	SETREG32(DDRC_BASE+0x0618, 0x0000312a);
	SETREG32(DDRC_BASE+0x06a4, 0x0011000e);
	SETREG32(DDRC_BASE+0x06c4, 0x00005288);
	SETREG32(DDRC_BASE+0x06c8, 0x0000410f);
	SETREG32(DDRC_BASE+0x0754, 0x00110001);
	SETREG32(DDRC_BASE+0x0490, 0x00000001);
	SETREG32(DDRC_BASE+0x0540, 0x00000001);
	SETREG32(DDRC_BASE+0x05f0, 0x00000001);
	SETREG32(DDRC_BASE+0x06a0, 0x00000001);
	SETREG32(DDRC_BASE+0x0750, 0x00000001);

	/* Priority settings for TX and RX. Rx is given
	 * more priority than TX.
	 */
	SETREG32(DDRC_BASE+0x0494, 0x00200001);	//PCFGQOS0_n
	SETREG32(DDRC_BASE+0x0544, 0x02220000);	//PCFGQOS0_n
	SETREG32(DDRC_BASE+0x05f4, 0x02220000);	//PCFGQOS0_n
	SETREG32(DDRC_BASE+0x06a4, 0x02220000);	//PCFGQOS0_n
	SETREG32(DDRC_BASE+0x0754, 0x00000001);	//PCFGQOS0_n

	GETREG32(DDRC_BASE+0x0060);
}

void DDR_MCTL_init_2 (void)
{
	SETREG32(DDRC_BASE+0x0304, 0x00000000);
	// //GETREG32(DDRC_BASE+0x0030);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000008);
	// //GETREG32(DDRC_BASE+0x0030);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000008);
	SETREG32(DDRC_BASE+0x0030, 0x00000000);
	SETREG32(DDRC_BASE+0x0320, 0x00000000);
	SETREG32(DDRC_BASE+0x01b0, 0x00000000);
}

void DDR_MCTL_init_3 (void)
{
	unsigned int get_value,pgsr_ok;

	SETREG32(DDRC_BASE+0x01b0, 0x00000001);
	SETREG32(DDRC_BASE+0x0320, 0x00000001);
	do
	{
		get_value = GETREG32(DDRC_BASE + 0x0324 );
		pgsr_ok =  get_value   & 0x1;
	} while( pgsr_ok != 0x1 );

	do
	{
		get_value = GETREG32(DDRC_BASE + 0x0004 );
		pgsr_ok =  get_value   & 0x1;
	} while( pgsr_ok != 0x1 );

#ifdef CONFIG_FPGA
	SETREG32(DDRC_BASE+0x0030, 0x00000008);
	SETREG32(DDRC_BASE+0x0030, 0x00000008);
	SETREG32(DDRC_BASE+0x0030, 0x00000008);
#else
	// //SETREG32(DDRC_BASE+0x0030, 0x00000000);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000000);
	// //SETREG32(DDRC_BASE+0x0030, 0x00000000);
#endif
	SETREG32(DDRC_BASE+0x0030, 0x00000000);
	// //GETREG32(DDRC_BASE+0x0308);
	GETREG32(DDRC_BASE+0x0308);
}

void DDR_MCTL_DFI_init_complete_ctrl (int on)
{
	unsigned int v;

	SETREG32(DDRC_BASE+0x0320, 0x00000000);
	if (on)
		SETREG32(DDRC_BASE+0x01b0, 0x00000001);
	else
		SETREG32(DDRC_BASE+0x01b0, 0x00000000);
	SETREG32(DDRC_BASE+0x0320, 0x00000001);
	do
	{
		v = GETREG32(DDRC_BASE + 0x0324 );
	} while( (v & 0x1) != 0x1 );
}

void DDRPHY_init0 (void)
{
	unsigned int get_value,pgsr_ok;

	SETREG32(DDRP0_BASE+DCR,0x0000000d);
#ifdef CONFIG_FPGA
	SETREG32(DDRP0_BASE+MR0,0x00000852);
	SETREG32(DDRP0_BASE+MR1,0x00000083);
#else
	SETREG32(DDRP0_BASE+MR0,0x00000c52);
	SETREG32(DDRP0_BASE+MR1,0x000000c3);
#endif

	SETREG32(DDRP0_BASE+MR2,0x00000006);
	//SETREG32(DDRP0_BASE+MR3,0x00000002);
	SETREG32(DDRP0_BASE+MR3,0x00000000);

#ifdef CONFIG_FPGA
	SETREG32(DDRP0_BASE+DTPR0,0x46918692);
	SETREG32(DDRP0_BASE+DTPR1,0x11341088);
	SETREG32(DDRP0_BASE+DTPR2,0x0647a0c8);
	SETREG32(DDRP0_BASE+PGCR,0x018c2e02);
	SETREG32(DDRP0_BASE+DXCCR, 0x00008c40);
#else
	SETREG32(DDRP0_BASE+DTPR0,0x44d7abb2);
	SETREG32(DDRP0_BASE+DTPR1,0x194610d8);
	SETREG32(DDRP0_BASE+DTPR2,0x064790c8);
	//SETREG32(DDRP0_BASE+PGCR,0x018C2e02);//lupin add 20160812
	SETREG32(DDRP0_BASE+PGCR,0x01842e02);
	SETREG32(DDRP0_BASE+DXCCR, 0x00000c40);
	SETREG32(DDRP0_BASE+DSGCR ,0xfa00025f);

	get_value = GETREG32(DDRP0_BASE + DX0GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP0_BASE + DX0GCR,  get_value);
	get_value = GETREG32(DDRP0_BASE + DX1GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP0_BASE + DX1GCR,  get_value);
	get_value = GETREG32(DDRP0_BASE + DX2GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP0_BASE + DX2GCR,  get_value);
	get_value = GETREG32(DDRP0_BASE + DX3GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP0_BASE + DX3GCR,  get_value);
#endif

	SETREG32(DDRP0_BASE+DX0DQSTR ,DXDQSTR[0][0]);//0x3db03000);
	SETREG32(DDRP0_BASE+DX1DQSTR ,DXDQSTR[0][1]);//0x3db03000);
	SETREG32(DDRP0_BASE+DX2DQSTR ,DXDQSTR[0][2]);//0x3db03000);
	SETREG32(DDRP0_BASE+DX3DQSTR ,DXDQSTR[0][3]);//0x3db03000);

	//-> 1103.000 ns: [BENCH] Polling register at address 0x3 on bits [0:0] for value 1 ...
	do
	{
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok = get_value & 0x1;
	} while( !pgsr_ok );

	get_value = 0;
	pgsr_ok = 0;
#if 0
	//-> 1893.000 ns: [BENCH] PHY initialization done...
	//-> 1892.812 ns: [CFG] Data out: Q = 00000007
	//
	//SETREG32(DDRP0_BASE+ACDLLCR,0x40000000);
	SETREG32(DDRP0_BASE+PIR, 0x000000c1);
	//SETREG32(DDRP0_BASE+PIR, 0x00040001);
	//-> 1910.000 ns: [BENCH] Polling register at address 0x3 on bits [3:0] for value 15 ...

	do
	{
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value & 0xf;
	} while( pgsr_ok != 0xf );


	//-> 5849.000 ns: [BENCH] PUB initialization done...
	//-> 5849.062 ns: [CFG] Data out: Q = 0000001f
	//-> 5861.312 ns: [SYS] END OF INITIALIZATION


	SETREG32(DDRP0_BASE+PIR, 0x00000011);
	do
	{
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value & 0xf;
	} while( pgsr_ok != 0xf );
#elif 0
	//SETREG32(DDRP0_BASE+PTR0, 0x003FFFFF);
	//SETREG32(DDRP0_BASE+PTR1, 0x07FFFFFF);
	//DLLSRST + DLLLOCK + ZCAL + ITMSRST
	SETREG32(DDRP0_BASE+PIR, 0x0000001F);
	do
	{
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value & 0x7;
	} while( pgsr_ok != 0x7 );

	////DRAMINIT
	//SETREG32(DDRP0_BASE+PIR, 0x00000041);
	//SKIP DRAMINIT by PHY
	SETREG32(DDRP0_BASE+PIR, 0x00040001);
	do
	{
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value & 0x1;
	} while( !pgsr_ok );

#if 0
	if (bTrain == 2) {
		SETREG32(DDRP0_BASE+PIR, 0x00000011);
		do
		{
			get_value = GETREG32( DDRP0_BASE + PGSR );
			pgsr_ok =  get_value & 0x1;
		} while( !pgsr_ok );

		//QSTRN + RVTRN
		SETREG32(DDRP0_BASE+PIR, 0x00000181);
		do
		{
			get_value = GETREG32( DDRP0_BASE + PGSR );
			pgsr_ok =  get_value & 0x1;
		} while( !pgsr_ok );
	}
#endif
	SETREG32(DDRP0_BASE+PIR, 0x00000011);
	do
	{
		get_value = GETREG32( DDRP0_BASE + PGSR );
		pgsr_ok =  get_value & 0x1;
	} while( !pgsr_ok );
	//udelay(15);
#endif
}

void DDRPHY_init1 (void)
{
	unsigned int get_value,pgsr_ok;

	SETREG32(DDRP1_BASE+DCR,0x0000000d);
#ifdef CONFIG_FPGA
	SETREG32(DDRP1_BASE+MR0,0x00000852);
	SETREG32(DDRP1_BASE+MR1,0x00000083);
#else
	SETREG32(DDRP1_BASE+MR0,0x00000c52);
	//SETREG32(DDRP1_BASE+MR0,0x00000042);
	SETREG32(DDRP1_BASE+MR1,0x000000c3);
#endif

	SETREG32(DDRP1_BASE+MR2,0x00000006);
	//SETREG32(DDRP1_BASE+MR3,0x00000002);
	SETREG32(DDRP1_BASE+MR3,0x00000000);

#ifdef CONFIG_FPGA
	SETREG32(DDRP1_BASE+DTPR0,0x46918692);
	SETREG32(DDRP1_BASE+DTPR1,0x11341088);
	SETREG32(DDRP1_BASE+DTPR2,0x0647a0c8);
	SETREG32(DDRP1_BASE+PGCR,0x018c2e02);
	SETREG32(DDRP1_BASE+DXCCR, 0x00008c40);
#else
	SETREG32(DDRP1_BASE+DTPR0,0x44d7abb2);
	SETREG32(DDRP1_BASE+DTPR1,0x194610d8);
	SETREG32(DDRP1_BASE+DTPR2,0x064790c8);
	//SETREG32(DDRP1_BASE+PGCR,0x018C2e02);//lupin add 20160812
	SETREG32(DDRP1_BASE+PGCR,0x01842e02);
	SETREG32(DDRP1_BASE+DXCCR, 0x00000c40);
	SETREG32(DDRP1_BASE+DSGCR ,0xfa00025f);

	get_value = GETREG32(DDRP1_BASE + DX0GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP1_BASE + DX0GCR,  get_value);
	get_value = GETREG32(DDRP1_BASE + DX1GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP1_BASE + DX1GCR,  get_value);
	get_value = GETREG32(DDRP1_BASE + DX2GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP1_BASE + DX2GCR,  get_value);
	get_value = GETREG32(DDRP1_BASE + DX3GCR);
	get_value &= ~(3 << 9);
	SETREG32(DDRP1_BASE + DX3GCR,  get_value);
#endif

	SETREG32(DDRP1_BASE+DX0DQSTR ,DXDQSTR[1][0]); //0x3db03000);
	SETREG32(DDRP1_BASE+DX1DQSTR ,DXDQSTR[1][1]);//0x3db03000);
	SETREG32(DDRP1_BASE+DX2DQSTR ,DXDQSTR[1][2]);//0x3db03000);
	SETREG32(DDRP1_BASE+DX3DQSTR ,DXDQSTR[1][3]);//0x3db03000);

	//-> 1103.000 ns: [BENCH] Polling register at address 0x3 on bits [0:0] for value 1 ...
	do
	{
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value   & 0x1;
	} while( pgsr_ok != 0x1 );

	get_value = 0;
	pgsr_ok = 0;
#if 0
	//-> 1893.000 ns: [BENCH] PHY initialization done...
	//-> 1892.812 ns: [CFG] Data out: Q = 00000007
	//
	//SETREG32(DDRP1_BASE+ACDLLCR, 0x40000000);
	SETREG32(DDRP1_BASE+PIR, 0x000000c1);
	//SETREG32(DDRP1_BASE+PIR, 0x00040001);
	//-> 1910.000 ns: [BENCH] Polling register at address 0x3 on bits [3:0] for value 15 ...
	do
		{
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value   & 0xf;
	} while( pgsr_ok != 0xf );

	//-> 5849.000 ns: [BENCH] PUB initialization done...
	//-> 5849.062 ns: [CFG] Data out: Q = 0000001f
	//-> 5860.312 ns: [SYS] END OF INITIALIZATION

	SETREG32(DDRP1_BASE+PIR, 0x00000011);
	do
	{
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value   & 0xf;
	} while( pgsr_ok != 0xf );
#elif 0
	//SETREG32(DDRP1_BASE+PTR0, 0x003FFFFF);
	//SETREG32(DDRP1_BASE+PTR1, 0x07FFFFFF);
	//DLLSRST + DLLLOCK + ZCAL + ITMSRST
	SETREG32(DDRP1_BASE+PIR, 0x0000001F);
	do
	{
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value   & 0x7;
	} while( pgsr_ok != 0x7 );

	//DRAMINIT
	//SETREG32(DDRP1_BASE+PIR, 0x00000041);
	//SKIP DRAMINIT by PHY
	SETREG32(DDRP1_BASE+PIR, 0x00040001);
	do
	{
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok =  get_value   & 0x1;
	} while( !pgsr_ok );

#if 0
	if (bTrain == 2) {
		SETREG32(DDRP1_BASE+PIR, 0x00000011);
		do
		{
			get_value = GETREG32( DDRP1_BASE + PGSR );
			pgsr_ok = get_value & 0x1;
		} while(!pgsr_ok);

		//QSTRN + RVTRN
		SETREG32(DDRP1_BASE+PIR, 0x00000181);
		do
		{
			get_value = GETREG32( DDRP1_BASE + PGSR );
			pgsr_ok =  get_value   & 0x1;
		} while( !pgsr_ok );
	}
#endif

	SETREG32(DDRP1_BASE+PIR, 0x00000011);
	do
	{
		get_value = GETREG32( DDRP1_BASE + PGSR );
		pgsr_ok = get_value & 0x1;
	} while(!pgsr_ok);
	//udelay(15);
#endif
}

int DDR_Init (int bp0p1, int bTrain)
{
	int ret = 0;

	//DDR CTRL INIT
	//SETREG32(SCU_BASE + 0x3c, 0x00007d00);
	SETREG32(SCU_BASE + 0x3c, 0x00007F00);
	//deassert DDRPHY core reset and DDRPHY APB reset
	SETREG32(SCU_BASE + 0x44, 0x000c0000);
	SETREG32(SCU_BASE + 0x3c, 0x00007d00);
	//udelay(20);
	DDR_MCTL_init();
	SETREG32(SCU_BASE + 0x3c, 0x00000000);
	DDR_MCTL_init_2();
	SETREG32(SCU_BASE + 0x44, 0x00000000);
	//udelay(1);

	if (bp0p1) {
		//DDR PHY INIT
		//SETREG32(SCU_BASE + 0x44, 0x00080000);
		//wait_cycles(10);
		//udelay(12);
		DDRPHY_init0();

		//SETREG32(SCU_BASE + 0x44, 0x00000000);
		//wait_cycles(10);
		//udelay(12);
		DDRPHY_init1();
	}
	else {
		//DDR PHY INIT
		//SETREG32(SCU_BASE + 0x44, 0x00040000);
		//wait_cycles(10);
		//udelay(12);
		DDRPHY_init1();

		//SETREG32(SCU_BASE + 0x44, 0x00000000);
		//wait_cycles(10);
		//udelay(12);
		DDRPHY_init0();
	}
	//DLLSRST + DLLLOCK + ZCAL + ITMSRST
	DDRPHYPIR(0x0000001F);
	//DRAMINIT
	DDRPHYPIR(0x00000041);
	//Reset ITM
	DDRPHYPIR(0x11);
	DDR_MCTL_init_3();

	DDR_MCTL_DFI_init_complete_ctrl(0);

	switch (bTrain)
	{
	case 1:
		do {
			if (prvSW_DQS_GW_Training(DDRP0_BASE)) {
				ret = -1;
				break;
			}

			if (prvSW_DQS_GW_Training(DDRP1_BASE)) {
				ret = -1;
				break;
			}

			if (prvSW_DQS_Training(DDRP0_BASE)) {
				ret = -1;
				break;
			}
			if (prvSW_DQS_Training(DDRP1_BASE)) {
				ret = -1;
				break;
			}
		} while(0);
		if (ret == 0) {
			DDR_Update_TrainingValue();
		}
		break;
	case 2:
		DDRPHYPIR(0x181);
		if ((GETREG32(DDRP0_BASE + PGSR) & 0x120) ||
			(GETREG32(DDRP1_BASE + PGSR) & 0x120)) {
		    ret = -1;
		}
		if (ret == 0) {
			DDRPHYPIR(0x11);
			ret = DDR_TEST();
		}
		if (ret == 0) {
			DDR_Update_TrainingValue();
		}
		break;
	default:
	case 0:
		ret = DDR_TEST();
		break;
	}

	return ret;
}
#endif	/* DDR_800 | DDR_1066 */
void hal_ddr_init (void)
{
	int reinit_cnt = 0;
	int bTrain = 2;
reinit:
	if (DDR_Init(0, bTrain) && ++reinit_cnt < 300) {
		writel(SCU_BASE + 0x24, 0);
		udelay(5);
		writel(SCU_BASE + 0x24, 0x07);
		goto reinit;
	}
	if (reinit_cnt >= 100)
		printf("DDR training fail!\n");
}

#else	/* GUC_SETTINGS */

static void ddr_phy_init (void)
{
	unsigned int get_value,pgsr_ok;

	writel(DDRP0_BASE + DCR, 0x0000000d);
	writel(DDRP1_BASE + DCR, 0x0000000d);
	writel(DDRP0_BASE + MR0, 0x00000c52);
	writel(DDRP1_BASE + MR0, 0x00000c52);
	writel(DDRP0_BASE + MR1, 0x000000c3);
	writel(DDRP1_BASE + MR1, 0x000000c3);
	writel(DDRP0_BASE + MR2, 0x00000006);
	writel(DDRP1_BASE + MR2, 0x00000006);
	writel(DDRP0_BASE + MR3, 0x00000002);
	writel(DDRP1_BASE + MR3, 0x00000002);
	writel(DDRP0_BASE + DTPR0, 0x44d7abb2);
	writel(DDRP1_BASE + DTPR0, 0x44d7abb2);
	writel(DDRP0_BASE + DTPR1, 0x194610dA);
	writel(DDRP1_BASE + DTPR1, 0x194610dA);
	writel(DDRP0_BASE + DTPR2, 0x064790c8);
	writel(DDRP1_BASE + DTPR2, 0x064790c8);
	writel(DDRP0_BASE + PGCR, 0x01842e02);
	writel(DDRP1_BASE + PGCR, 0x01842e02);
	writel(DDRP0_BASE + DXCCR, 0x00000c40);
	writel(DDRP1_BASE + DXCCR, 0x00000c40);
	writel(DDRP0_BASE + DX0DQSTR, 0x3db03000);
	writel(DDRP1_BASE + DX0DQSTR, 0x3db03000);
	writel(DDRP0_BASE + DX1DQSTR, 0x3db03000);
	writel(DDRP1_BASE + DX1DQSTR, 0x3db03000);
	writel(DDRP0_BASE + DX2DQSTR, 0x3db03000);
	writel(DDRP1_BASE + DX2DQSTR, 0x3db03000);
	writel(DDRP0_BASE + DX3DQSTR, 0x3db03000);
	writel(DDRP1_BASE + DX3DQSTR, 0x3db03000);

	do {
		get_value = readl(DDRP0_BASE + PGSR);
		pgsr_ok = get_value & 0x1;
	} while(pgsr_ok != 0x1);

	do {
		get_value = readl(DDRP1_BASE + PGSR);
		pgsr_ok = get_value & 0x1;
	} while(pgsr_ok != 0x1);

	writel(DDRP0_BASE+PIR, 0x000000c1);
	writel(DDRP1_BASE+PIR, 0x000000c1);

	do {
		get_value = readl(DDRP0_BASE + PGSR);
		pgsr_ok = get_value & 0x1f;
	} while(pgsr_ok != 0x1f);

	do {
		get_value = readl(DDRP1_BASE + PGSR);
		pgsr_ok = get_value & 0x1f;
	} while(pgsr_ok != 0x1f);
}

static void ddr_mctl_init_1 (void)
{
	writel(DDRC_BASE + 0x0304, 0x00000001);
	writel(DDRC_BASE + 0x0030, 0x00000001);
	readl(DDRC_BASE + 0x0004);
	writel(DDRC_BASE + 0x0000, 0x03040008);
	writel(DDRC_BASE + 0x0010, 0x00003030);
	writel(DDRC_BASE + 0x0014, 0x0000a630);
	writel(DDRC_BASE + 0x0020, 0x00000041);
	writel(DDRC_BASE + 0x0024, 0x49af1c53);
	writel(DDRC_BASE + 0x0030, 0x00000008);
	writel(DDRC_BASE + 0x0034, 0x000d4900);
	writel(DDRC_BASE + 0x0038, 0x00a70000);
	writel(DDRC_BASE + 0x0050, 0x0071f070);
	writel(DDRC_BASE + 0x0060, 0x00000000);
	writel(DDRC_BASE + 0x0064, 0x00200038);
	writel(DDRC_BASE + 0x00c0, 0x00000000);
	writel(DDRC_BASE + 0x00d0, 0x00030003);
	writel(DDRC_BASE + 0x00d4, 0x0000000f);
	writel(DDRC_BASE + 0x00d8, 0x00000a06);
	writel(DDRC_BASE + 0x00dc, 0x00c30046);
	writel(DDRC_BASE + 0x00e0, 0x00020000);
	writel(DDRC_BASE + 0x00e4, 0x000a0004);
	writel(DDRC_BASE + 0x00f0, 0x00000000);
	writel(DDRC_BASE + 0x00f4, 0x0000098e);
	writel(DDRC_BASE + 0x0100, 0x080e110b);
	writel(DDRC_BASE + 0x0104, 0x00020213);
	writel(DDRC_BASE + 0x0108, 0x02040607);
	writel(DDRC_BASE + 0x010c, 0x00505000);
	writel(DDRC_BASE + 0x0110, 0x07020307);
	writel(DDRC_BASE + 0x0114, 0x08070404);
	writel(DDRC_BASE + 0x0118, 0x0a0a0006);
	writel(DDRC_BASE + 0x011c, 0x0000070e);
	writel(DDRC_BASE + 0x0120, 0x00002901);
	writel(DDRC_BASE + 0x0138, 0x0000003b);
	writel(DDRC_BASE + 0x0180, 0x40600018);
	writel(DDRC_BASE + 0x0184, 0x00e00070);
	writel(DDRC_BASE + 0x0188, 0x00000000);
	writel(DDRC_BASE + 0x0190, 0x02030002);
	writel(DDRC_BASE + 0x0194, 0x00030202);
	writel(DDRC_BASE + 0x0198, 0x07719001);
	writel(DDRC_BASE + 0x01a0, 0x60400005);
	writel(DDRC_BASE + 0x01a4, 0x003000d8);
	writel(DDRC_BASE + 0x01a8, 0x00000000);
	writel(DDRC_BASE + 0x01b0, 0x00000001);
	writel(DDRC_BASE + 0x022c, 0x00000000);

	/*
		cs 0   : 0x200[4:0]    6	22 16
		cs 1   : 0x200[12:8]   7	22 16
		bank 0 : 0x204 [4:0]   2	23 17	25
		bank 1 : 0x204 [12:8]  3	23 17	26
		bank 2 : 0x204 [20:16] 4	23 17	27
		col 2  : 0x208 [3:0]   2	0	2
		col 3  : 0x208 [11:8]  3	0	3
		col 4  : 0x208 [19:16] 4	0	4
		col 5  : 0x208 [27:24] 5	0	5
		col 6  : 0x20c [3:0]   6	0	6
		col 7  : 0x20c [11:8]  7	0	7
		col 8  : 0x20c [19:16] 8	0	8
		col 9  : 0x20c [27:24] 9	0	9
		col 10 : 0x210 [3:0]   10	f
		col 11 : 0x210 [11:8]  11	f
		row 0  : 0x214 [3:0]   6	4	10
		row 1  : 0x214 [11:8]  7	4	11
		row 2  : 0x214 [19:16] 8
		row 11 : 0x214 [27:24] 17	4	21
		row 12 : 0x218 [3:0]   18	4	22
		row 13 : 0x218 [11:8]  19	4	23
		row 14 : 0x218 [19:16] 20	4	24
		row 15 : 0x218 [27:24] 21	f
		row 2  : 0x224 [3:0]   8	4	12
		row 3  : 0x224 [11:8]  9	4	13
		row 4  : 0x224 [19:16] 10	4	14
		row 5  : 0x224 [27:24] 11	4	15
		row 6  : 0x228 [3:0]   12	4	16
		row 7  : 0x228 [11:8]  13	4	17
		row 8  : 0x228 [19:16] 14	4	18
		row 9  : 0x228 [27:24] 15	4	19
	*/

	writel(DDRC_BASE + 0x0200, 0x00000016);		/*cs				*/
	writel(DDRC_BASE + 0x0204, 0x00171717);		/*bank 2 1 0		*/
	writel(DDRC_BASE + 0x0208, 0x00000000);		/*col 5 4 3 2		*/
	writel(DDRC_BASE + 0x020c, 0x00000000);		/*col 9 8 7 6		*/
	writel(DDRC_BASE + 0x0210, 0x00000f0f);		/*col 11 10			*/
	writel(DDRC_BASE + 0x0214, 0x04040404);		/*row 11 10 1  0	*/
	writel(DDRC_BASE + 0x0218, 0x0f040404);		/*row 15 14 13 12	*/
	writel(DDRC_BASE + 0x0224, 0x04040404);		/*row 5  4  3  2	*/
	writel(DDRC_BASE + 0x0228, 0x04040404);		/*row 9  8  7  6	*/

	writel(DDRC_BASE + 0x0240, 0x0c120c40);
	writel(DDRC_BASE + 0x0244, 0x00000132);
	writel(DDRC_BASE + 0x0250, 0x07f20905);
	writel(DDRC_BASE + 0x0254, 0x0000001e);
	writel(DDRC_BASE + 0x025c, 0x1300d6a1);
	writel(DDRC_BASE + 0x0264, 0xd000cfbe);
	writel(DDRC_BASE + 0x026c, 0x1600e287);
	writel(DDRC_BASE + 0x0300, 0x00000011);
	writel(DDRC_BASE + 0x0304, 0x00000000);
	writel(DDRC_BASE + 0x030c, 0x00000000);
	writel(DDRC_BASE + 0x0320, 0x00000001);
	writel(DDRC_BASE + 0x036c, 0x00000000);

	writel(DDRC_BASE + 0x2020, 0x00000041);
	writel(DDRC_BASE + 0x2024, 0x49af1c53);
	writel(DDRC_BASE + 0x2050, 0x0071f070);
	writel(DDRC_BASE + 0x2064, 0x00208038);
	writel(DDRC_BASE + 0x20dc, 0x00c30046);
	writel(DDRC_BASE + 0x20e0, 0x00020000);
	writel(DDRC_BASE + 0x2100, 0x080e110b);
	writel(DDRC_BASE + 0x2104, 0x00020213);
	writel(DDRC_BASE + 0x2108, 0x02040607);
	writel(DDRC_BASE + 0x210c, 0x00505000);
	writel(DDRC_BASE + 0x2110, 0x07020307);
	writel(DDRC_BASE + 0x2114, 0x08070404);
	writel(DDRC_BASE + 0x2118, 0x0a0a0006);
	writel(DDRC_BASE + 0x211c, 0x0000070e);
	writel(DDRC_BASE + 0x2120, 0x00002901);
	writel(DDRC_BASE + 0x2138, 0x0000003b);
	writel(DDRC_BASE + 0x2180, 0x40600018);
	writel(DDRC_BASE + 0x2190, 0x02040102);
	writel(DDRC_BASE + 0x2194, 0x00030202);
	writel(DDRC_BASE + 0x2240, 0x0c120c40);

	writel(DDRC_BASE + 0x0400, 0x00000010);
	writel(DDRC_BASE + 0x0404, 0x000000da);
	writel(DDRC_BASE + 0x0408, 0x00006201);
	writel(DDRC_BASE + 0x0490, 0x00000001);
	writel(DDRC_BASE + 0x0494, 0x00200007);
	writel(DDRC_BASE + 0x04b4, 0x0000119b);
	writel(DDRC_BASE + 0x04b8, 0x00001144);
	writel(DDRC_BASE + 0x0540, 0x00000001);
	writel(DDRC_BASE + 0x0544, 0x0010000e);
	writel(DDRC_BASE + 0x0564, 0x00000156);
	writel(DDRC_BASE + 0x0568, 0x00005139);
	writel(DDRC_BASE + 0x05f4, 0x00110006);
	writel(DDRC_BASE + 0x0614, 0x00004223);
	writel(DDRC_BASE + 0x0618, 0x0000312a);
	writel(DDRC_BASE + 0x06a0, 0x00000001);
	writel(DDRC_BASE + 0x06a4, 0x0011000e);
	writel(DDRC_BASE + 0x06c4, 0x00005288);
	writel(DDRC_BASE + 0x06c8, 0x0000410f);
	writel(DDRC_BASE + 0x0750, 0x00000001);
	writel(DDRC_BASE + 0x0754, 0x00110001);
	writel(DDRC_BASE + 0x0490, 0x00000001);
	writel(DDRC_BASE + 0x0540, 0x00000001);
	writel(DDRC_BASE + 0x05f0, 0x00000001);
	writel(DDRC_BASE + 0x06a0, 0x00000001);
	writel(DDRC_BASE + 0x0750, 0x00000001);

	readl(DDRC_BASE + 0x0060);
}

static void ddr_mctl_init_2 (void)
{
	writel(DDRC_BASE + 0x0304, 0x00000000);
	readl(DDRC_BASE + 0x0030);
	writel(DDRC_BASE + 0x0030, 0x00000008);
	writel(DDRC_BASE + 0x0320, 0x00000000);
	writel(DDRC_BASE + 0x01b0, 0x00000000);
}

static void ddr_mctl_init_3 (void)
{
	unsigned int get_value, pgsr_ok;

	writel(DDRC_BASE + 0x01b0, 0x00000001);
	writel(DDRC_BASE + 0x0320, 0x00000001);
	do {
		get_value = readl(DDRC_BASE + 0x0324 );
		pgsr_ok = get_value & 0x1;
	} while( pgsr_ok != 0x1 );

	do {
		get_value = readl(DDRC_BASE + 0x0004 );
		pgsr_ok = get_value & 0x1;
	} while( pgsr_ok != 0x1 );

	writel(DDRC_BASE + 0x0030, 0x00000008);
	readl(DDRC_BASE + 0x0308);
}
/* Exported functions implementation -----------------------------------------*/
void hal_ddr_init(void)
{
	static uint8_t initialized;

	if (initialized)
		return;

	/* DDR CTRL INIT */
	/* Reset DDRPHY, DMAC, DDRC */
	writel(SCU_BASE + 0x3c, 0x00007d00);
	writel(SCU_BASE + 0x44, 0x000c0000);
	ddr_mctl_init_1();
	writel(SCU_BASE + 0x3c, 0x00000000);
	ddr_mctl_init_2();
	/* DDR PHY INIT */
	writel(SCU_BASE + 0x44, 0x00000000);
	ddr_phy_init();
	ddr_mctl_init_3();

	initialized = 1;
}
#endif
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
