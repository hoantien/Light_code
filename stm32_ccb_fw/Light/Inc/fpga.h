/**
  ******************************************************************************
  * \file    af_ctrl.h
  * \author  Infonam Embedded Team
  * \version V1.0.0
  * \date    25-May-2015
  * \brief   Header file of VCM module related APIs
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FPGA_H
#define __FPGA_H

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <types.h>
#include <stdlib.h>

/* Exported typedef ----------------------------------------------------------*/
typedef enum {
	FPGA_PWR_FAIL = -1,
	FPGA_PWR_OFF = 0,
	FPGA_PWR_GOOD = 1,
} fpga_pg_status_t;

typedef enum {
	FPGA_IRQ_WAITING = 0,
	FPGA_IRQ_TRIGGERED = 1,
} fpga_irq_status_t;

typedef enum {
	FPGA_CMD_TOUT = -1,
	FPGA_CMD_WAITING = 0,
	FPGA_CMD_SUCCESS = 1,
} fpga_cmd_error_t;

typedef enum
{
	FPGA_CAPTURE_VC0 = 0x3,
	FPGA_CAPTURE_VC1 = 0x7,
}fpga_vc_capture_channel_t;

typedef enum
{
	FPGA_CSI_CHANNEL_0 = 0x0,
	FPGA_CSI_CHANNEL_1 = 0x1,
}fpga_csi_channel_t;

#define IRQ_FROM_FPGA_TIMEOUT 100000000
#define CAPTURE_VC_REG_ADDR		0x10B00010
#define CAPTURE_VC_CHANNEL_CMD		0x00A0
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void fpga_init(void);
fpga_cmd_error_t fpga_send_command(UInt16 *tx, UInt16 *rx, UInt16 len, UInt32 timeout);
fpga_pg_status_t fpga_check_cam_pg(void);
fpga_pg_status_t fpga_check_pg(void);
#endif /* __FPGA_H */
/*********** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE***********/
