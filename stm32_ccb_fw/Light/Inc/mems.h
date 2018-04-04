/********************************************************************************
 * Copyright (c) 2015, The LightCo
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are strictly prohibited without prior permission of The LightCo.
 *******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CCB_COMMAND_H
#define __CCB_COMMAND_H

#ifdef __cplusplus
 extern "C" {
#endif

 /* System includes */
#include <stdio.h>
#include <stdint.h>
#include "hal_cci.h"
#include "stm32f4xx_flash.h"


/* Private typedef -----------------------------------------------------------*/
typedef struct {
    __IO uint16_t ASIC_ID;
    __IO uint16_t ASIC_LIGHT_PROTOCOL;
    __IO uint8_t  ASIC_OS_VERSION[8];
    __IO uint16_t ASIC_CALIB_VERSION;
    __IO uint16_t ASIC_CFG_VERSION;
    __IO uint16_t ASIC_FW_CHECKSUM;
    __IO uint16_t ASIC_STATUS;
    __IO uint16_t ASIC_CONFIG;
    __IO uint16_t ASIC_LOG_CTRL;
    __IO uint16_t ASIC_SENSOR_STATUS;
    __IO uint16_t ASIC_TMP1;
    __IO uint16_t ASIC_TMP2;
    __IO uint16_t ASIC_TMP3;
    __IO uint16_t ASIC_TMP4;
    __IO uint8_t  ASIC_DUMP[50];
}CCB_ASIC_Typedef;

/* Private define ------------------------------------------------------------*/

// CCB register memory map
#define CCB_REG_BASE        ((uint32_t)0x081A0000)  // Using the System memory 384KBytes
                                                    // ending for CCB registers
#define CCB_REG_MANF_BASE   (CCB_REG_BASE)
#define CCB_REG_MANF_SIZE   0x20000                 // 128KBytes
#define CCB_REG_PREV_BASE   (CCB_REG_MANF_BASE + CCB_REG_MANF_SIZE)
#define CCB_REG_PREV_SIZE   0x20000                 // 128KBytes
#define CCB_REG_CURR_BASE   (CCB_REG_PREV_BASE + CCB_REG_PREV_SIZE)
#define CCB_REG_CURR_SIZE   0x20000                 // 128KBytes
#define CCB_REG_TOTAL_SIZE  (CCB_REG_MANF_SIZE + CCB_REG_PREV_BASE + CCB_REG_CURR_SIZE)

#define CCB_MEM_BASE        ((uint32_t)0x2002C000)  // Using the SRAM 16KBytes
                                                    // ending for CCB registers
#define CCB_MEM_TOTAL_SIZE  0x4000                  // 2Kbytes
#define CCB_ASIC_OFFSET     0x0000                  // CCB_ASIC offset
#define CCB_ASIC_SIZE       0x100                   // 256Bytes for ASIC registers
#define CCB_CAM0_OFFSET     (CCB_ASIC_OFFSET + CCB_ASIC_SIZE)
#define CCB_CAM0_SIZE       0x100                   // 256Bytes for CAMERA2 registers
#define CCB_CAM1_OFFSET     (CCB_CAM0_OFFSET + CCB_CAM0_SIZE)
#define CCB_CAM1_SIZE       0x100                   // 256Bytes for CAMERA1 registers
#define CCB_CAM2_OFFSET     (CCB_CAM1_OFFSET + CCB_CAM1_SIZE)
#define CCB_CAM2_SIZE       0x100                   // 256Bytes for CAMERA2 registers
#define CCB_CAM3_OFFSET     (CCB_CAM2_OFFSET + CCB_CAM2_SIZE)
#define CCB_CAM3_SIZE       0x100                   // 256Bytes for CAMERA2 registers
#define CCB_CAM4_OFFSET     (CCB_CAM3_OFFSET + CCB_CAM3_SIZE)
#define CCB_CAM4_SIZE       0x100                   // 256Bytes for CAMERA2 registers
#define CCB_CAM5_OFFSET     (CCB_CAM4_OFFSET + CCB_CAM4_SIZE)
#define CCB_CAM5_SIZE       0x100                   // 256Bytes for CAMERA2 registers

// CCI Register declaration
#define CCB_ASIC_MANF_REG   ((CCB_ASIC_Typedef *) CCB_REG_MANF_BASE)
#define CCB_ASIC_PREV_REG   ((CCB_ASIC_Typedef *) CCB_REG_PREV_BASE)
#define CCB_ASIC_REG        ((CCB_ASIC_Typedef *) CCB_REG_CURR_BASE)
#define CCB_ASIC_MEM        ((CCB_ASIC_Typedef *) CCB_MEM_BASE)
#define CCB_ASIC_MEM_RD     ((uint8_t *) CCB_MEM_BASE)
#define CCB_ASIC_CAM_RD     ((__IO uint8_t *)(CCB_MEM_BASE + 0x200))

//m_bitmask for modules selection
#define CCB_M_BITMASK       ((__IO uint8_t *)(CCB_MEM_BASE + 0x3FF0))
#define CCB_M_STATUS        ((__IO uint8_t *)(CCB_MEM_BASE + 0x3FB0))
#define CCB_M_EEPROM        ((uint8_t *)(CCB_MEM_BASE + 0x37B0))
#define CCB_M_TEMP          ((uint8_t*)(CCB_MEM_BASE + 0x021C))

#define CCB_M_WRITE                 ((uint8_t*)(CCB_MEM_BASE + 0x34B0))
#define CCB_CMD_MASK		0x1FFF
#define CCB_FLAGS_MASK		0xE000
#define CCB_INTR_EN			0x08

/* Structure Memmory mapped to CCB_M_WRITE */
typedef struct
{
       volatile uint8_t cam_m_open[16];
       volatile uint8_t cam_m_stream[48];
       volatile uint8_t cam_m_uuid[32];
       volatile uint8_t cam_m_command[4];
       volatile uint8_t cam_m_resolution[128];
       volatile uint8_t cam_m_sensitivity[64];
       volatile uint8_t cam_m_exposure_time[128];
       volatile uint8_t cam_m_focal_length[32];
       volatile uint8_t cam_m_focus_distance[64];
       volatile uint8_t cam_m_fps[32];
       volatile uint8_t cam_m_focus_status[4];
       volatile uint8_t cam_m_lens_hall[32];
       volatile uint8_t cam_m_mirror_status[4];
       volatile uint8_t cam_m_mirror_hall[32];
       volatile uint8_t cam_m_gyro[6];
       volatile uint8_t cam_m_light_active_ucid[2];
       volatile uint8_t cam_m_status[64];
       volatile uint8_t cam_m_temp[32];
       volatile uint8_t cam_m_board_temp[8];
}ccb_cmd_base_t;


// Use-case id
#define CCB_M_UCID          ((__IO uint8_t *)(CCB_MEM_BASE + 0x200 + 0x1000 + 0x2))
#define CCB_M_TOLERANCE     ((__IO uint8_t *)(CCB_MEM_BASE + 0x200 + 0x1000 + 0x4))

// Flash sector declaration
#define CCB_ASIC_MANF_SECTOR    FLASH_Sector_21
#define CCB_ASIC_PREV_SECTOR    FLASH_Sector_22
#define CCB_ASIC_CURR_SECTOR    FLASH_Sector_23

/*********** ASIC Control registers Address *************/
/********************************************************/
#define ASIC_BASE                   (0x0200)
#define ASIC_ID_ADDR                (ASIC_BASE)
#define ASIC_ID_SIZE                (2)
#define ASIC_LIGHT_PROTOCOL_ADDR    (ASIC_ID_ADDR + ASIC_ID_SIZE)
#define ASIC_LIGHT_PROTOCOL_SIZE    (2)
#define ASIC_OS_VERSION_ADDR        (ASIC_LIGHT_PROTOCOL_ADDR + ASIC_LIGHT_PROTOCOL_SIZE)
#define ASIC_OS_VERSION_SIZE        (8)
#define ASIC_CALIB_VERSION_ADDR     (ASIC_OS_VERSION_ADDR + ASIC_OS_VERSION_SIZE)
#define ASIC_CALIB_VERSION_SIZE     (2)
#define ASIC_CFG_VERSION_ADDR       (ASIC_CALIB_VERSION_ADDR + ASIC_CALIB_VERSION_SIZE)
#define ASIC_CFG_VERSION_SIZE       (2)
#define ASIC_FW_CHECKSUM_ADDR       (ASIC_CFG_VERSION_ADDR + ASIC_CFG_VERSION_SIZE)
#define ASIC_FW_CHECKSUM_SIZE       (2)
#define ASIC_STATUS_ADDR            (ASIC_FW_CHECKSUM_ADDR + ASIC_FW_CHECKSUM_SIZE)
#define ASIC_STATUS_SIZE            (2)
#define ASIC_CONFIG_ADDR            (ASIC_STATUS_ADDR + ASIC_STATUS_SIZE)
#define ASIC_CONFIG_SIZE            (2)
#define ASIC_LOG_CTRL_ADDR          (ASIC_CONFIG_ADDR + ASIC_CONFIG_SIZE)
#define ASIC_LOG_CTRL_SIZE          (2)
#define ASIC_SENSOR_STATUS_ADDR     (ASIC_LOG_CTRL_ADDR + ASIC_LOG_CTRL_SIZE)
#define ASIC_SENSOR_STATUS_SIZE     (2)
#define ASIC_TMP1_ADDR              (ASIC_SENSOR_STATUS_ADDR + ASIC_SENSOR_STATUS_SIZE)
#define ASIC_TMP1_SIZE              (2)
#define ASIC_TMP2_ADDR              (ASIC_TMP1_ADDR + ASIC_TMP1_SIZE)
#define ASIC_TMP2_SIZE              (2)
#define ASIC_TMP3_ADDR              (ASIC_TMP2_ADDR + ASIC_TMP2_SIZE)
#define ASIC_TMP3_SIZE              (2)
#define ASIC_TMP4_ADDR              (ASIC_TMP3_ADDR + ASIC_TMP3_SIZE)
#define ASIC_TMP4_SIZE              (2)
#define ASIC_DUMP_ADDR              (ASIC_TMP4_ADDR + ASIC_TMP4_SIZE)
#define ASIC_DUMP_SIZE              (50)

#define CCB_REG_WRITE_FLAG_OFFSET   0x1FFFC
#define CCB_REG_WRITE_FLAG_FAILED   0xFF
#define CCB_REG_WRITE_FLAG_OK       0xAA

/**
 *  CCB Register Structure
 */
typedef struct {
    uint8_t *data;      /**< Specifies Register/Command address */
    uint8_t size;       /**< Specifies the Register/Command length */
} Reg;

/* Private CCB define ------------------------------------------------------------*/
#define CAM_MODULE_OPEN                     0x0000
#define CAM_STREAMING                       0x0002
#define RDI_CAPTURE                         0x0003
#define CAM_SNAPSHOT_UUID                   0x0006
#define CAM_MODULE_RESOLUTION               0x002C
#define CAM_MODULE_STATUS                   0x0028
#define CAM_MODULE_SENSITIVITY              0x0030
#define CAM_MODULE_EXPOSURE_TIME            0x0032
#define CAM_MODULE_FOCAL_LEN                0x003A
#define CAM_MODULE_VCM_POSITION             0x003C
#define CAM_MODULE_FOCUS_STATUS             0x003D
#define CAM_MODULE_FOCUS_DISTANCE           0x0048
#define CAM_MODULE_LENS_POSITION            0x0040
#define CAM_MODULE_LENS_POSITION2           0x0041
#define CAM_MODULE_LENS_HALL                0x0042
#define CAM_MODULE_LENS_NUDGE               0x0043
#define CAM_MODULE_MIRROR_POSITION          0x0044
#define CAM_MODULE_MIRROR_POSITION2         0x0045
#define CAM_MODULE_MIRROR_HALL              0x0046
#define SPI_TRANSFER_FPGA                   0x004E
#define CAM_MODULE_MIRROR_NUDGE             0x0047
#define CAM_MODULE_FPS                      0x0050
#define CAM_MODULE_FINE_NUDGE_LENS          0x0051
#define CAM_MODULE_FINE_NUDGE_MIRROR        0x0052
#define CAM_MODULE_LENS_CALIBRATION         0x0060
#define CAM_MODULE_LENS_CALIBRATION2        0x0061
#define CAM_MODULE_MIRROR_CALIBRATION       0x0062
#define CAM_MODULE_FOCUS_CALIBRATION_DATA   0x0063
#define CAM_MODULE_DEBUG_I2C_READ           0x0080
#define CAM_MODULE_DEBUG_I2C_WRITE          0x0082
#define CAM_MODULE_DEBUG_PIEZO_MONITOR      0x0084
#define CAM_MODULE_DEBUG_PIEZO_MONITOR2     0x0085
#define SPI_TRANSFER_CPLD_SINGLE            0x0090
#define SPI_TRANSFER_CPLD                   0x0091
#define CONFIGURE_CPLD                      0x0092
#define CAM_MODULE_NUDGE_CPLD               0x0093
#define CAM_MODULE_PIEZO_LENS_CONTROL       0x0094
//#define CAM_MODULE_PIEZO_LENS_CALIBRATION   0x0095
#define CAM_MODULE_PIEZO_MIRROR_CONTROL     0x0096
//#define CAM_MODULE_PIEZO_MIRROR_CALIBRATION 0x0097
#define CAM_MODULE_LENS_FREQ_CALIBRATION    0x0098
#define CAM_MODULE_MIRROR_FREQ_CALIBRATION  0x0099
#define SOFTWARE_VERSION                    0x00A0
#define CAM_MODULE_GYRO_STATUS              0x00A1
#define CAM_MODULE_SAVE_POSITION            0x00A2
#define CAM_MODULE_MIRROR_FOCAL_STATUS		0x00A3
#define STM_VERSION							0x00A4
#define CAM_MODULE_TEMP						0x00A5
#define LIGHT_ACTIVE_UCID                   0x1000
#define CAM_MODULE_SET_CAPTURE_PARAM        0x10BB
#define CCB_INTR_SRC                        0x027C

#define CAM_COMMAND_BOARD_TEMPERATURE		0x00A6
/* Private CPLD defines -----------------------------------------------------*/

#define CPLD_WRITE         0x02
#define CPLD_READ          0x0B
#define MAX_CPLD_TXRX_SIZE 0x08

/**
 *  \brief Function to initialize the CCB register memory map
 *
 *  \par Header File:
 *  mems.h
 *
 *  \par Description:
 *  This function load the factory setting to the base address of the ccb asic and
 *  set default value to the ccb asic structure.
 */

/**
 *  \brief command status
 *
 *  \par Header File:
 *  mems.h
 *
 *  \par Description:
 *  This command indicates status of current received command
 */
#define CAM_COMMAND_STATUS          0x0024

void CCB_MEM_Init(void);

/**
 *  \brief Function to backup ccb asic setting on exit.
 *
 *  \par Header File:
 *  mems.h
 *
 *  \par Description:
 *  This function stores current setting to flash before exit.
 */
void CCB_MEM_Exit(void);

/**
 *  \brief Function to update ccb asic setting from source to destination.
 *
 *  \par Header File:
 *  mems.h
 *
 *  \par Parameters:
 *  \param dst (in) Destination address
 *  \param src (in) Pointer to source address
 *
 *  \par Description:
 *  This function copy data from source to destination. The size of copied data is
 *  size of ccb asic structure.
 */
void CCB_MEM_Update(uint32_t dst, uint32_t *src);

/**
 *  \brief [Obsolete] Function to parse command from host and set register data to ccb asic memory
 *
 *  \par Header File:
 *  mems.h
 *
 *  \par Parameters:
 *  \param rx (in) Pointer to buffer that store register data
 */
void CCB_Cmd_Rx_Parser(CCI_Buffer *rx);

/**
 *  \brief [Obsolete] Function to parse command from host and transmit register value to host
 *
 *  \par Header File:
 *  mems.h
 *
 *  \par Parameters:
 *  \param tx (out) Pointer to buffer that store register data
 */
void CCB_Cmd_Tx_Parser(CCI_Buffer *tx);

#ifdef __cplusplus
}
#endif

#endif /* __DEFINE_H */





 /************** Portions COPYRIGHT 2015 Light.Co., Ltd.*****END OF FILE****/
