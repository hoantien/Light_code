/*******************************************************************************
 * Copyright (c) 2016, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification,are strictly prohibited without prior permission of The LightCo.
 *
 * @file	hal_dma.c
 * @author	The LightCo
 * @version	V1.0.0
 * @date	Mar-30-2016
 * @brief	This file contains definitions of the DMA controller.
 *
 ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "cortex_r4.h"
#include "hal_vic.h"
#include "hal_cache.h"
#include "hal_dma.h"

/* Private typedef -----------------------------------------------------------*/
#define DMA_REG(name)	__IO uint32_t name; \
						__IO uint32_t pad_##name

/**
 * @brief dma_regs_t
 *
 * DMA Registers
 */
typedef struct dma_regs
{
	DMA_REG(IDR);		/* DMAC ID register. */
	DMA_REG(CVR);		/* DMAC Component Version Register. */
	DMA_REG(CFGR);		/* DMAC Configuration Register. */
	DMA_REG(CHER);		/* DMAC Channel Enable Register. */
	DMA_REG(RESERVED[2]);
	DMA_REG(INTCHSR);	/* DMAC Interrupt Channel Status Register. */
	DMA_REG(INTCLR);	/* DMAC Interrupt Clear Register. */
	DMA_REG(INTSER);	/* Common Register Interrupt Status Enable Register. */
	DMA_REG(INTSIER);	/* DMAC Common Register Interrupt Signal Enable. */
	DMA_REG(INTSR);		/* DMAC Common Register Interrupt Status Register. */
	DMA_REG(RR);		/* DMAC Reset Register. */
} dma_regs_t;

/**
 * @brief dma_channel_regs_t
 *
 * DMA channel Registers
 */
typedef struct dma_channel_regs
{
	DMA_REG(SAR);		/* Source Address Register. */
	DMA_REG(DAR);		/* Destination Address Register. */
	DMA_REG(BTSR);		/* Block Transfer Size Register. */
	__IO uint32_t CRL;	/* Control Register Low. */
	__IO uint32_t CRH;	/* Control Register High. */
	__IO uint32_t CFGL;	/* Configure Register Low. */
	__IO uint32_t CFGH; /* Configure Register High. */
	DMA_REG(LLP);		/* Linked List Pointer register. */
	DMA_REG(SR);		/* Status Register. */
	DMA_REG(SWHSSRCR);	/* Software Handshake Source Register. */
	DMA_REG(SWHSDSTR);	/* Software Handshake Destination Register. */
	DMA_REG(RESUMEREQR);/* Block Transfer Resume Request Register. */
	DMA_REG(AXIIDR);	/* AXI ID Register. */
	DMA_REG(AXIQOSR);	/* AXI QOS Register. */
	DMA_REG(SSR);		/* Source Status Register. */
	DMA_REG(DSR);		/* Destination Status Register. */
	DMA_REG(SSFAR);		/* Source Status Fetch Address Register. */
	DMA_REG(DSFAR);		/* Destination Status Fetch Address Register. */
	DMA_REG(INTSER);	/* Interrupt Status Enable Register. */
	DMA_REG(INTSR);		/* Interrupt Status Register. */
	DMA_REG(INTSIER);	/* Interrupt Signal Enable Register. */
	DMA_REG(INTCLR);	/* Interrupt Clear Register. */
} dma_channel_regs_t;

/**
 * @brief dma_lli_regs_t
 *
 * DMA lli registers
 */
typedef struct dma_lli_regs
{
	DMA_REG(SAR);
	DMA_REG(DAR);
	__IO uint32_t BTSR;
	__IO uint32_t RESERVED1;
	DMA_REG(LLPR);
	__IO uint32_t CRL;
	__IO uint32_t CRH;
	__IO uint32_t WB_SRCSR;
	__IO uint32_t WB_DSRSR;
	__IO uint32_t RESERVED2;
	__IO uint32_t RESERVED3;
	__IO uint32_t RESERVED4;
	__IO uint32_t RESERVED5;
} dma_lli_regs_t;

/**
 * @brief dma_hw_config_t
 *
 * DMA hardware configuration structure
 */
typedef struct dma_hw_config
{
	dma_channel_regs_t	*ch_addr;
	void				*lli_addr;
	dma_lli_regs_t		*lli;
	uint8_t				used;
	void(*clb_handler)(void);
} dma_hw_config_t;

/* Private define ------------------------------------------------------------*/
#define DMA_CH_1	((dma_channel_regs_t *)(DMAC_BASE + 0x100))
#define DMA_CH_2	((dma_channel_regs_t *)(DMAC_BASE + 0x200))
#define DMA_CH_3	((dma_channel_regs_t *)(DMAC_BASE + 0x300))
#define DMA_CH_4	((dma_channel_regs_t *)(DMAC_BASE + 0x400))
#define DMA_CH_5	((dma_channel_regs_t *)(DMAC_BASE + 0x500))
#define DMA_CH_6	((dma_channel_regs_t *)(DMAC_BASE + 0x600))
#define DMA_CH_7	((dma_channel_regs_t *)(DMAC_BASE + 0x700))
#define DMA_CH_8	((dma_channel_regs_t *)(DMAC_BASE + 0x800))

#define HS_SEL_SRC						BIT3
#define HS_SEL_DST						BIT4
#define CFGH_SRC_PER(n)					(n << 7)
#define CFGH_DST_PER(n)					(n << 12)
#define SRC_TR_WIDTH(n)					((n & 0x7) << 8)
#define DST_TR_WIDTH(n)					((n & 0x7) << 11)
#define SRC_MSIZE(n)					((n & 0xF) << 14)
#define DST_MSIZE(n)					((n & 0xF) << 18)
/** Interrupt status/signal enable register field. **/
#define BLOCK_TFR_DONE					BIT0
#define DMA_TRANSFER_DONE				BIT1
#define SRC_TRANSCOMPLETED				BIT3
#define DST_TRANSCOMPLETED				BIT4
#define SRC_DEC_ERR						BIT5
#define DST_DEC_ERR						BIT6
#define SRC_SLV_ERR						BIT7
#define DST_SLV_ERR						BIT8
#define LLI_RD_DEC_ERR					BIT9
#define LLI_WR_DEC_ERR					BIT10
#define LLI_RD_SLV_ERR					BIT11
#define LLI_WR_SLV_ERR					BIT12
#define SHADOWREG_OR_LLI_INVALID_ERR	BIT13
#define SLVIF_MULTIBLKTYPE_ERR			BIT14
#define SLVIF_DEC_ERR					BIT16
#define SLVIF_WR2RO_ERR					BIT17
#define SLVIF_RD2RWO_ERR				BIT18
#define SLVIF_WRONCHEN_ERR				BIT19
#define SLVIF_SHADOWREG_WRON_VALID_ERR	BIT20
#define SLVIF_WRONHOLD_ERR				BIT21
#define CHLOCK_CLEARED					BIT27
#define CH_SRC_SUSPENDED				BIT28
#define CH_SUSPENDED					BIT29
#define CH_DISABLED						BIT30
#define CH_ABORTED						BIT31

#define SHADOWREG_OR_LLI_LAST			BIT30
#define SHADOWREG_OR_LLI_VALID			BIT31

#define CFG_SRC_MB_TYPE(n)				((n & 0x03) << 0)
#define CFG_DST_MB_TYPE(n)				((n & 0x03) << 2)

#define INT_COMMOM						BIT16
#define INT_DMA_TFR_DONE				BIT1

#define MAX_DMA_BLOCK_TS				32767

/* Private function prototypes -----------------------------------------------*/
static void dma_irq_handler(void);
/* Private variables ---------------------------------------------------------*/
static dma_hw_config_t dma_hw[HAL_DMA_CH_MAX] =
{
	{DMA_CH_1, NULL, NULL, 0, NULL},
	{DMA_CH_2, NULL, NULL, 0, NULL},
	{DMA_CH_3, NULL, NULL, 0, NULL},
	{DMA_CH_4, NULL, NULL, 0, NULL},
	{DMA_CH_5, NULL, NULL, 0, NULL},
	{DMA_CH_6, NULL, NULL, 0, NULL},
	{DMA_CH_7, NULL, NULL, 0, NULL},
	{DMA_CH_8, NULL, NULL, 0, NULL},
};
static dma_regs_t *dma_base = (dma_regs_t *)DMAC_BASE;
static uint8_t dma_init_irq_flag;

/* Exported functions --------------------------------------------------------*/
hal_dma_status_t hal_dma_init(dma_init_type_t *init)
{
	/* Check NULL pointer. */
	if(init == NULL)
		return HAL_DMA_NULL_PTR;
	if(HAL_DMA_CH_MAX == init->chid)
		return HAL_DMA_CH_INVALID;
	if(dma_hw[init->chid].used)
		return HAL_DMA_CH_BUSY;

	uint8_t chid = init->chid;
	uint32_t tmp;

	dma_channel_regs_t *dma = dma_hw[chid].ch_addr;
	/* Set the DMA channel is used. */
	dma_hw[chid].used = 1;
	/* Set the DMA transfer type and flow control. */
	dma->CFGH &= ~0x7;
	dma->CFGH |= (init->trans_type & 0x07);
	/* Clear the source hardware handshaking interface. */
	dma->CFGH &= ~(0x7 << 7);
	/* Clear the destination hardware handshaking interface. */
	dma->CFGH &= ~(0x7 << 12);
	/* Set the source and destination software handshaking selection. */
	dma->CFGH |= HS_SEL_SRC | HS_SEL_DST;
	switch(init->trans_type)
	{

		case MEM_TO_PER:
			/* Set the destination hardware handshaking interface. */
			dma->CFGH &= ~HS_SEL_DST;
			dma->CFGH |= CFGH_DST_PER(init->periph_type);
			break;
		case PER_TO_MEM:
			/* Set the source hardware handshaking interface. */
			dma->CFGH &= ~HS_SEL_SRC;
			dma->CFGH |= CFGH_SRC_PER(init->periph_type);
			break;
		case MEM_TO_MEM:
		case PER_TO_PER:
		default:
			break;
	}
	/* Set the source address */
	dma->SAR = init->src_addr;
	dma->pad_SAR = 0;
	/* Set the destination address. */
	dma->DAR = init->dst_addr;
	dma->pad_DAR = 0;
	/* Calculate the block transfer size. */
	tmp = (init->length + ((1 << init->src_wdata) - 1)) >> init->src_wdata;
	tmp = tmp ? tmp - 1 : 0;
	dma->BTSR = tmp;
	/* Set the DMA control register. */
	dma->CRL &= ~(SRC_TR_WIDTH(0x7) | DST_TR_WIDTH(0x7) |
					SRC_MSIZE(0xF) | DST_MSIZE(0xF));
	dma->CRL |= (SRC_TR_WIDTH(init->src_wdata) | DST_TR_WIDTH(init->dst_wdata) |
					SRC_MSIZE(init->src_bsize) | DST_MSIZE(init->dst_bsize));
	/* Set the source master select. */
	dma->CRL |= init->src_axi & 0x1;
	/* Set the destination master select. */
	dma->CRL |= (init->dst_axi & 0x1) << 2;
	/* Clear the Source/Destination address increment. */
	dma->CRL &= ~((BIT4) | (BIT6));
	switch(init->trans_type)
	{
		case MEM_TO_PER:
			/* Set the destination address no-increment because the device is
			fetching data from a source peripheral FIFO with a fixed address. */
			dma->CRL |= BIT6;
			break;
		case PER_TO_MEM:
			/* Set the source address no-increment because the device is
			fetching data from a source peripheral FIFO with a fixed address. */
			dma->CRL |= BIT4;
			break;
		case MEM_TO_MEM:
		case PER_TO_PER:
		default:
			break;
	}
	/* Set the interrupt enable/signal register. */
	dma->INTSER |= (DMA_TRANSFER_DONE | SHADOWREG_OR_LLI_INVALID_ERR |
					SLVIF_MULTIBLKTYPE_ERR | CH_ABORTED);
	dma->INTSIER |= (DMA_TRANSFER_DONE | SHADOWREG_OR_LLI_INVALID_ERR |
					SLVIF_MULTIBLKTYPE_ERR | CH_ABORTED);
	/* Initialize the call back function if it is used. */
	if(init->clb_func)
		dma_hw[chid].clb_handler = init->clb_func;
	/* Initialize the linked list interface if it is used. */
	if(init->lli)
	{
		uint32_t block_cnt = init->lli_cnt;
		void *lli_addr = NULL;
		dma_hw[chid].lli_addr = malloc(block_cnt * sizeof(dma_lli_regs_t));
		if (dma_hw[chid].lli_addr == NULL)
			return HAL_DMA_NULL_PTR;
		lli_addr = dma_hw[chid].lli_addr;

		if((uint32_t)lli_addr % 64)
		{
			dma_hw[chid].lli = (dma_lli_regs_t *)((uint32_t)lli_addr +
						(64 - ((uint32_t)lli_addr % 64)));
		}
		else
		{
			dma_hw[chid].lli = (dma_lli_regs_t *)dma_hw[chid].lli_addr;
		}
		for(int i = 0; i < block_cnt ; i++)
		{
			dma_hw[chid].lli[i].SAR = init->lli[i].src_addr;
			dma_hw[chid].lli[i].pad_SAR = 0;
			dma_hw[chid].lli[i].DAR = init->lli[i].dst_addr;
			dma_hw[chid].lli[i].pad_DAR = 0;
			tmp = (init->lli[i].length >> init->src_wdata);
			tmp = tmp ? tmp - 1 : 0;
			dma_hw[chid].lli[i].BTSR = tmp;
			dma_hw[chid].lli[i].CRH = dma->CRH;
			dma_hw[chid].lli[i].CRH &= ~SHADOWREG_OR_LLI_LAST;
			dma_hw[chid].lli[i].CRH |= SHADOWREG_OR_LLI_VALID;
			dma_hw[chid].lli[i].CRL = dma->CRL;

			/* The last LLi. */
			if(i == (block_cnt - 1))
			{
				dma_hw[chid].lli[i].LLPR = (uint32_t)&(dma_hw[chid].lli[0]);
				dma_hw[chid].lli[i].CRH |= SHADOWREG_OR_LLI_LAST;
			}
			else
			{
				dma_hw[chid].lli[i].LLPR = (uint32_t)&(dma_hw[chid].lli[i+1]);
			}
		}
		flush_cache((unsigned long)dma_hw[chid].lli,
											sizeof(dma_lli_regs_t) * block_cnt);
		dma->LLP = (uint32_t)dma_hw[chid].lli;
		dma->CFGL |= CFG_SRC_MB_TYPE(LINKED_LIST) |
						CFG_DST_MB_TYPE(LINKED_LIST);
	}
	else
	{
		if((init->length >> init->src_wdata) > MAX_DMA_BLOCK_TS)
		{
			uint32_t block_cnt = 1 + ((init->length >> init->src_wdata) - 1) /
								MAX_DMA_BLOCK_TS;
			void *lli_addr = NULL;
			dma_hw[chid].lli_addr =
								malloc(sizeof(dma_lli_regs_t) * block_cnt);
			if (dma_hw[chid].lli_addr == NULL)
				return HAL_DMA_NULL_PTR;
			lli_addr = dma_hw[chid].lli_addr;
			if((uint32_t)lli_addr % 64)
			{
				dma_hw[chid].lli = (dma_lli_regs_t *)((uint32_t)lli_addr +
							(64 - ((uint32_t)lli_addr % 64)));
			}
			else
			{
				dma_hw[chid].lli = (dma_lli_regs_t *)dma_hw[chid].lli_addr;
			}
			for(int i = 0; i < block_cnt ; i++)
			{
				dma_hw[chid].lli[i].SAR = init->src_addr + i * MAX_DMA_BLOCK_TS;
				dma_hw[chid].lli[i].pad_SAR = 0;
				dma_hw[chid].lli[i].DAR = init->dst_addr + i * MAX_DMA_BLOCK_TS;
				dma_hw[chid].lli[i].pad_DAR = 0;

				dma_hw[chid].lli[i].CRH = dma->CRH;
				dma_hw[chid].lli[i].CRH |= SHADOWREG_OR_LLI_VALID;
				dma_hw[chid].lli[i].CRL = dma->CRL;
				/* The last LLi. */
				if(i == block_cnt - 1)
				{
					dma_hw[chid].lli[i].BTSR =
					(
						(init->length - (((MAX_DMA_BLOCK_TS * (block_cnt - 1))
						<< init->src_wdata) >> init->src_wdata)) - 1
					);
					dma_hw[chid].lli[i].CRH |= SHADOWREG_OR_LLI_LAST;
				}
				else
				{
					dma_hw[chid].lli[i].BTSR = MAX_DMA_BLOCK_TS - 1;
					dma_hw[chid].lli[i].LLPR =
											(uint32_t)&(dma_hw[chid].lli[i+1]);
				}
			}
			flush_cache((unsigned long)dma_hw[chid].lli,
											sizeof(dma_lli_regs_t)*block_cnt);
			dma->LLP = (uint32_t)dma_hw[chid].lli;
			dma->CFGL |= CFG_SRC_MB_TYPE(LINKED_LIST) |
						CFG_DST_MB_TYPE(LINKED_LIST);
		}
	}
	return HAL_DMA_OK;
}
hal_dma_status_t hal_dma_deinit(hal_dma_channel_t chid)
{
	if(HAL_DMA_CH_MAX == chid)
	{
		return HAL_DMA_CH_INVALID;
	}
	hal_dma_channel_disable(chid);
	dma_hw[chid].used = 0;
	dma_hw[chid].clb_handler = NULL;
	return HAL_DMA_OK;
}
hal_dma_status_t hal_dma_channel_enable(hal_dma_channel_t chid)
{
	if(HAL_DMA_CH_MAX == chid)
	{
		return HAL_DMA_CH_INVALID;
	}
	dma_channel_regs_t *dma = dma_hw[chid].ch_addr;
	/* Clear the clear interrupt register. */
	dma->INTCLR = 0xFFFFFFFF;
	/* Enable the DMA channel register. */
	dma_base->CHER |= 0x101 << chid;
	return HAL_DMA_OK;
}
hal_dma_status_t hal_dma_channel_disable(hal_dma_channel_t chid)
{
	if(HAL_DMA_CH_MAX == chid)
	{
		return HAL_DMA_CH_INVALID;
	}
	dma_channel_regs_t *dma = dma_hw[chid].ch_addr;
	/* Clear the clear interrupt register. */
	dma->INTCLR = 0xFFFFFFFF;
	/* Enable the DMA channel register. */
	dma_base->CHER &= ~(0x101 << chid);
	return HAL_DMA_OK;
}
hal_dma_status_t hal_dma_set_channel_priority(hal_dma_channel_t chid,
															uint8_t priority)
{
	if(HAL_DMA_CH_MAX == chid)
	{
		return HAL_DMA_CH_INVALID;
	}
	dma_channel_regs_t *dma = dma_hw[chid].ch_addr;
	dma->CFGH &= ~(0x7 << 17);
	dma->CFGH |= priority << 17;
	return HAL_DMA_OK;
}
uint8_t hal_dma_get_channel_priority(hal_dma_channel_t chid)
{
	if(HAL_DMA_CH_MAX == chid)
	{
		return (uint8_t)HAL_DMA_CH_INVALID;
	}
	dma_channel_regs_t *dma = dma_hw[chid].ch_addr;
	uint8_t priority = 0;
	priority = (dma->CFGH >> 17) & 0x7;
	return priority;
}
hal_dma_status_t hal_dma_is_transfer_completed(hal_dma_channel_t chid)
{
	uint32_t reg = 0;
	hal_dma_status_t ret;
	reg = dma_base->CHER;
	if(!(reg & (1 << chid)))
	{
		ret = HAL_DMA_RESULT_YES;
	}
	else
	{
		ret = HAL_DMA_RESULT_NO;
	}
	return ret;
}
void hal_dma_reset(void)
{
	dma_base->RR = 0x1;
	while(dma_base->RR & 0x1);
}
void hal_dma_enable_global_interrupt(uint8_t irq_priority)
{
	/* Is the DMA interrupt flag initialized. */
	if(!dma_init_irq_flag)
	{
		/* Enable the DMA global interrupt. */
		dma_base->CFGR = 0x03;
		/* Register the interrupt line for the DMA controller. */
		vic_register_irq(DMA_IRQn, dma_irq_handler);
		/* Set the priority of the DMA interrupt. */
		vic_set_priority_irq(DMA_IRQn, irq_priority);
		dma_init_irq_flag = 1;
	}
}
static void dma_irq_handler(void)
{
	register uint32_t common_int = 0;
	common_int = dma_base->INTCHSR;
	if(common_int & INT_COMMOM)
	{
		/* Clear the interrupt register. */
		dma_base->INTCLR = 0xFFFFFFFF;
	}
	for(int i = 0; i < HAL_DMA_CH_MAX; i++)
	{
		if(common_int & (1 << i))
		{
			if(dma_hw[i].ch_addr->INTSR & INT_DMA_TFR_DONE)
			{
				/* Call back function When the DMA transfer completed data. */
				if(dma_hw[i].clb_handler)
				{
					dma_hw[i].clb_handler();
				}
			}
			/* Clear all bits of the channel interrupt register. */
			dma_hw[i].ch_addr->INTCLR = 0xFFFFFFFF;
		}
	}
}
/********** Portions COPYRIGHT 2016 Light.Co., Ltd.******* END OF FILE ********/
