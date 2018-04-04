#include "drv_vcm.h"
#include "af_ctrl.h"
#include "mems.h"
#include "CCBConfig.h"
#include "FreeRTOS.h"
#include "log.h"
#include "task.h"
#include "hal_i2c_ex.h"
#include "cam_ctrl.h"
#include "Interpolators.h"
#include "semphr.h"

typedef enum {
	BYTE_TYPE = 0,
	WORD_TYPE,
	BLOCK_TYPE
} I2CTYPE;

typedef struct {
	UInt8 addr;
	UInt16 data;
	I2CTYPE type;
} AFRegSetting;

#ifdef BOARD_VERSION_P1
STATIC AFRegSetting init_reg_setting_1[] = {
	{ 0x80, 0x34,   BYTE_TYPE }, /* CLKSEL 1/1, CLKON */
	{ 0x81, 0x20,   BYTE_TYPE }, /* AD 4Time */
	{ 0x84, 0xE0,   BYTE_TYPE }, /* STBY   AD ON,DA ON,OP ON */
	{ 0x87, 0x05,   BYTE_TYPE },
	{ 0xA4, 0x24,   BYTE_TYPE }, /* OCS clock -7% -10% */
	{ 0x3A, 0x0000, WORD_TYPE }, /* HXI offset value */
	{ 0x04, 0x0000,	WORD_TYPE }, /* AF control target value */
	{ 0x02, 0x0000, WORD_TYPE },
	{ 0x18, 0x0000, WORD_TYPE }, /* MS1Z22 Clear(STMV Target Value) */
	{ 0x86, 0x40,   BYTE_TYPE },
	{ 0x40, 0x8010,	WORD_TYPE },/* Hall filter settings */
	{ 0x42, 0x7150, WORD_TYPE },
	{ 0x44, 0x8F90, WORD_TYPE },
	{ 0x46, 0x61B0, WORD_TYPE },
	{ 0x48, 0x65B0, WORD_TYPE },
	{ 0x4A, 0x2870,	WORD_TYPE },
	{ 0x4C, 0x4030, WORD_TYPE },
	{ 0x4E, 0x7FF0, WORD_TYPE },
	{ 0x50, 0x04F0, WORD_TYPE },
	{ 0x52, 0x7610, WORD_TYPE },
	{ 0x54, 0x16C0,	WORD_TYPE },
	{ 0x56, 0x0000, WORD_TYPE },
	{ 0x58, 0x7FF0, WORD_TYPE },
	{ 0x5A, 0x0680, WORD_TYPE },
	{ 0x5C, 0x72F0, WORD_TYPE },
	{ 0x5E, 0x7F70,	WORD_TYPE },
	{ 0x60, 0x7ED0, WORD_TYPE },
	{ 0x62, 0x7FF0, WORD_TYPE },
	{ 0x64, 0x0000, WORD_TYPE },
	{ 0x66, 0x0000,	WORD_TYPE },
	{ 0x68, 0x5130, WORD_TYPE },
	{ 0x6A, 0x72F0, WORD_TYPE },
	{ 0x6C, 0x8010, WORD_TYPE },
	{ 0x6E, 0x0000, WORD_TYPE },
	{ 0x70, 0x0000,	WORD_TYPE },
	{ 0x72, 0x18E0, WORD_TYPE },
	{ 0x74, 0x4E30, WORD_TYPE },
	{ 0x30, 0x0000, WORD_TYPE },
	{ 0x76, 0x0C50, WORD_TYPE },
	{ 0x78, 0x4000,	WORD_TYPE }, /* 39 */
};
STATIC AFRegSetting init_reg_setting_2[] = {
	{ 0x86, 0x60,   BYTE_TYPE },
	{ 0x88, 0x70,   BYTE_TYPE },
	{ 0x4C, 0x4000,	WORD_TYPE }, /* AF control equalizer coefficient gain1 */
	{ 0x83, 0x2C,   BYTE_TYPE }, /* Update with a value of (measurement filter 2 output) * ms22d *ms2x2. 11: Update with a value of RZ. */
	{ 0x85, 0xC0,   BYTE_TYPE }, /* delay register clear */
};
STATIC AFRegSetting init_reg_setting_3[] = {
	{ 0x84, 0xE3,   BYTE_TYPE },
	{ 0x97, 0x00,   BYTE_TYPE },
	{ 0x98, 0x42,   BYTE_TYPE },
	{ 0x99, 0x00,   BYTE_TYPE },
	{ 0x9A, 0x00,   BYTE_TYPE },
	{ 0x87, 0x85,   BYTE_TYPE } /* 5 */
};

STATIC AFRegSetting move_reg_setting[] = {
	{ 0x5A, 0x0800, WORD_TYPE },
	{ 0x83, 0xAC,   BYTE_TYPE },   /* AF control mode setting */
	{ 0xA0, 0x02,   BYTE_TYPE },   /* Step-move setting */
	{ 0x93, 0x40,   BYTE_TYPE },
	{ 0x7C, 0x0180, WORD_TYPE },
	{ 0x7A, 0x7000,	WORD_TYPE }, /* Step-width half border */
	{ 0x7E, 0x7E00, WORD_TYPE }, /* Step-width quarter border */
	{ 0x87, 0x85,   BYTE_TYPE }, /* Servo on */
};
#endif /* BOARD_VERSION_P1 */

/* AF data used to calculate focus distance */
extern AFData AFInfo[MAX_SUPPORTED_CAM];
STATIC Bool af_controller_internal_init(UInt8 ch_id);
STATIC Bool af_controller_internal_stop(AFData *af, UInt8 ch_id);
STATIC Bool af_controller_internal_move(AFData *af, UInt16 wPos, Bool isRealPos, UInt8 ch_id);
STATIC Int16 af_controller_calculate_stmvend(AFData *af);

#ifdef BOARD_VERSION_P1
STATIC void af_controller_write_reg_array(AFRegSetting regarr[],UInt8 size, UInt8 channelid);
#endif /* BOARD_VERSION_P1 */


UInt8 af_controller_read_eeprom(UInt8 addr,  UInt8 ch_id)
{
	uint8_t bData = 0;
	bData = fpga_read_i2c_value(ch_id, CAM_EEPROM_SLVADDR, 0x0000,ADDR16_DATA8);
	if(bData == 0x00)
	{
		log_error("Read EEPROM failed");
		return 0;
	}
	else
	{
		bData =  fpga_read_i2c_value(ch_id, CAM_EEPROM_SLVADDR , addr, ADDR16_DATA8);
		log_debug("Addr: %x Value: 0x%x", addr, bData);
		return bData;
	}

}
#ifdef BOARD_VERSION_P1
STATIC void af_controller_write_reg_array(AFRegSetting regarr[],UInt8 size, UInt8 channelid)
{
	UInt8 i;
	for(i = 0; i < size; i ++)
	{
		if(regarr[i].type == WORD_TYPE)
			fpga_write_i2c_value(channelid,AF_SLAVE_ADDR, regarr[i].addr, regarr[i].data,ADDR8_DATA16,ETrue);
		else
			fpga_write_i2c_value(channelid,AF_SLAVE_ADDR, regarr[i].addr, regarr[i].data,ADDR8_DATA8,ETrue);

	}

}
#endif /* BOARD_VERSION_P1 */

#ifdef BOARD_VERSION_P1_1
static uint8_t map[] ={ 0x0, 0x9a, 0x1, 0x9B,0x09, 0x87, 0xA, 0x12, 0xB, 0x13, 0xc, 0x28, 0xD, 0x29, 0xE, 0x85, 0xF, 0x81, 0x10, 0x82, 0x11, 0x84, 0x12, 0x86, 0x13, 0x88, 0x14,0x8a,
						0x15,0x8E, 0x16,0x92,0x17,0x93, 0x18,0x94, 0x19, 0x9e, 0x1A, 0x9f, 0x1b, 0xa2, 0x1c, 0xa9, 0x1d, 0xaa, 0x1e, 0xab, 0x1f, 0xac,
						0x20, 0xAD, 0x21,0xAE, 0x22, 0xAF, 0x23, 0xB0, 0x24,0xB1, 0x25, 0xB8, 0x26, 0xB9, 0x27, 0xC4, 0x28, 0xC5, 0x29, 0xCC};
void af_print_eeprom_regs(UInt8 chip_id)
{

	UInt8 data;
	int i;
	int map_size = (sizeof (map) / sizeof(uint8_t));
	for ( i = 0; i < map_size; i+=2)
	{
		data = af_controller_read_eeprom(map[i], chip_id);
		log_debug("eeprom reg : 0x%04x data : 0x%04x \n", map[i], data);
		data= fpga_read_i2c_value(chip_id, AF_SLAVE_ADDR, map[i+1], ADDR8_DATA8);
		log_debug("regdat reg : 0x%04x data : 0x%04x \n", map[i+1], data);

	}

}
#endif /* BOARD_VERSION_P1_1 */
#ifdef VCM_PRINT_EEPROM
Bool vcm_print_eeprom(CamDevice_TypeDef *pCam)
{
	uint8_t adds[] = { 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x25, 0x26, 0x27, 0x28, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F };
	int read_cnt = (sizeof (adds) / sizeof(uint8_t));
	int i = 0;
	UInt8 data;
	data = fpga_read_i2c_value(pCam->camera_id, CAM_EEPROM_SLVADDR, 0x0000,ADDR16_DATA8);
	if(data == 0)
	{
		log_error("Read EEPROM failed");
		return EFalse;
	}
	for(i = 0; i < read_cnt; i++)
	{
		data =  fpga_read_i2c_value(pCam->camera_id, CAM_EEPROM_SLVADDR , adds[i], ADDR16_DATA8);
		log_debug("[Slave: 0x%x] 0x%x = 0x%x", CAM_EEPROM_SLVADDR, adds[i], data);
	}
	return ETrue;
}
#endif
/**
 *
 */
Bool af_controller_init(CamDevice_TypeDef *pCam)
{
	AFData *af;

	if(pCam == NULL || pCam->Type != CAM_TYPE_35MM)
	{
		log_error("Unsupported camera");
		return EFalse;
	}

#ifdef VCM_PRINT_EEPROM
	vcm_print_eeprom(pCam);
#endif

	UInt8 ch_id = pCam->camera_id;

	if(pCam->camera_id < 1 || pCam->camera_id > 5)
	{
		log_error("Unsupported camera");
		return EFalse;
	}

	af = &AFInfo[pCam->camera_id-1];
	if (af->IsOn)
	{
		log_printf("AF module for %s has already been initialized.", pCam->Name);
		return ETrue;
	}
	log_debug("Initializing VCM on channel %d", ch_id);
	Bool rs = af_controller_internal_init(ch_id);
	if (rs)
	{
		/* Cached the current position */
		af_controller_get_current_pos(af, ch_id);
		UInt8 result = InitAFInfo(pCam);
		if(result == 0)	rs = EFalse;
	}

	af->IsOn = ETrue;
	return rs;

}

Bool af_controller_move(CamDevice_TypeDef *pCam, UInt16 wPos, Bool isRealPos)
{
	AFData *af;

	if(pCam == NULL || pCam->Type != CAM_TYPE_35MM)
	{
		log_error("Invalid argument");
		return EFalse;
	}
	UInt8 ch_id = pCam->camera_id;

	if(pCam->camera_id < 1 || pCam->camera_id > 5)
	{
		log_error("Unsupported camera");
		return EFalse;
	}

	af = &AFInfo[pCam->camera_id-1];

	if(!af->IsOn)
	{
		log_error("Please initialize AF module in cam %s first", pCam->Name);
		return EFalse;
	}

	if(!af->isValid)
	{
		log_error("AFInfo has not been initialized for cam %s",pCam->Name);
		return EFalse;
	}
	log_debug("Moving VCM to position %x on channel %d", wPos, ch_id);
	return af_controller_internal_move(af, wPos,isRealPos, ch_id);
}

Bool af_controller_stop(CamDevice_TypeDef *pCam)
{
	AFData *af;

	if(pCam == NULL || pCam->Type != CAM_TYPE_35MM)
	{
		log_error("Invalid argument");
		return EFalse;
	}

	UInt8 ch_id = pCam->camera_id;
	if(pCam->camera_id < 1 || pCam->camera_id > 5)
	{
		log_error("Unsupported camera");
		return EFalse;
	}

	af = &AFInfo[pCam->camera_id-1];
	if (!af->IsOn)
	{
		log_printf("AF module in cam %s is not initialized \r\n", pCam->Name);
		return ETrue;
	}
	log_debug("Stop VCM on channel %d", ch_id);
	Bool rs = af_controller_internal_stop(af,ch_id);
	if(rs == EFalse)
	{
		log_error("Failed to stop AF module");
		return rs;
	}
#ifdef BOARD_VERSION_P1_1
	af->IsOn = EFalse;
#endif

	log_printf("Stopped AF module\n\r");
	return rs;
}

STATIC Bool af_controller_internal_init(UInt8 ch_id)
{
	UInt8 bData,cnt = 0;
	UInt16 wData = 0;

#ifdef BOARD_VERSION_P1
	/* Initialize setting*/
	af_controller_write_reg_array(init_reg_setting_1, ARRAY_COUNT(init_reg_setting_1), ch_id);
	//vTaskDelay(500);
	af_controller_write_reg_array(init_reg_setting_2, ARRAY_COUNT(init_reg_setting_2), ch_id);
	bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, 0x85,ADDR8_DATA8);
#else /* BOARD_VERSION_P1_1 */
	bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, AF_CVER,ADDR8_DATA8);
	log_printf("\n VLC898214 Version check : %x", bData);
	fpga_write_i2c_value(ch_id,AF_SLAVE_ADDR, AF_AWDLCTRL ,0x1,ADDR8_DATA8,ETrue);

	bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, AF_AWDLCTRL, ADDR8_DATA8);

#endif /* BOARD_VERSION_P1_1 */

	/* Check if clear register is turned off */
	while(cnt < MAX_TRY_CNT && bData != 0x00)
	{
		cnt++;
		vTaskDelay(10);
#ifdef BOARD_VERSION_P1
		bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, AF_DLYCLR, ADDR8_DATA8);
#else /* BOARD_VERSION_P1_1 */
		bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, AF_AWDLCTRL,ADDR8_DATA8);
#endif
	}

#ifdef BOARD_VERSION_P1
	if(cnt >= MAX_TRY_CNT)
	{
		log_error("Could not clear delay register");
		return EFalse;
	}

	//vTaskDelay(500);
	af_controller_write_reg_array(init_reg_setting_3, ARRAY_COUNT(init_reg_setting_3), ch_id);

	/* Close loop vcm bias & close loop vcm offset */
	bData = af_controller_read_eeprom(EEPROM_AF_CLOSELOOP_VCM_BIAS, ch_id);
	if(bData == 0)
	{
		log_error("EEPROM data is invalid");
		return EFalse;
	}

	fpga_write_i2c_value(ch_id,AF_SLAVE_ADDR, 0x28, bData, ADDR8_DATA8, EFalse);
	bData = af_controller_read_eeprom(EEPROM_AF_CLOSELOOP_VCM_OFFSET, ch_id);
	if(bData == 0)
	{
		log_error("EEPROM data is invalid");
		return EFalse;
	}

	fpga_write_i2c_value(ch_id,AF_SLAVE_ADDR, 0x29, bData, ADDR8_DATA8, ETrue);
	/* Read and set existent position */
	wData = af_controller_get_current_pos(NULL, ch_id);
	fpga_write_i2c_value(ch_id,AF_SLAVE_ADDR, 0x04, wData, ADDR8_DATA16, ETrue);
	fpga_write_i2c_value(ch_id,AF_SLAVE_ADDR, 0x18, wData, ADDR8_DATA16, ETrue);
	/* Set remaining registers */
	af_controller_write_reg_array(move_reg_setting, ARRAY_COUNT(move_reg_setting), ch_id);
#else /* BOARD_VERSION_P1_1 */
	if(cnt >= MAX_TRY_CNT && bData != 0x00)
	{
		log_error("Wait VCM load timeout");
		return EFalse;
	}
	wData = af_controller_get_current_pos(NULL, ch_id);
#endif

	log_printf("Finish initializing AF module. Current: 0x%x\n\r", wData);
	return ETrue;
}


STATIC Bool af_controller_internal_move(AFData *af, UInt16 wPos, Bool isRealPos, UInt8 ch_id)
{
	Int16 stmv_end;
	Bool rs = EFalse;
	UInt8 bData;
	UInt8 tryCnt = 0;

	if (!af || !af->isValid)
	{
		log_debug("(%d): AFInfo not initialized for camera", __LINE__);
		return EFalse;
	}

	if(isRealPos == ETrue)
	{
		af->FocusDistance = wPos;
		stmv_end = af_controller_calculate_stmvend(af);
	}
	else
	{
		stmv_end = (Int16)wPos;
	}
#ifdef BOARD_VERSION_P1
	/* Re-enable AD, DA converter and HALL AMP after stopped */
	fpga_write_i2c_value(ch_id, AF_SLAVE_ADDR, AF_STANDBY_REG, 0xE3, ADDR8_DATA8, EFalse);
	stmv_end = CLAMP(stmv_end, AF_MIN_DAC, AF_MAX_DAC);
#else /* BOARD_VERSION_P1_1 */
	stmv_end = CLAMP(stmv_end, (Int16) af->CalibPosition[0], (Int16) af->CalibPosition[1]);
#endif




	af_controller_get_current_pos(af, ch_id);
	log_debug("Current HALL position: 0x%04x", af->CurrentPosition);
	if (stmv_end == af->CurrentPosition)
	{
		log_debug("Already at position 0x%04x",stmv_end);
		return ETrue;
	}

#ifdef BOARD_VERSION_P1
	/* Setting AG equalizer coefficient register */
	fpga_write_i2c_value(ch_id,AF_SLAVE_ADDR,0x40, 0x7ff0, ADDR8_DATA16, EFalse );
	/* Moving target point and direction */
	UInt16 moving_direction = (wPos > af->CurrentPosition) ?
											MOVE_TO_INFINITY : MOVE_TO_MACRO;
	fpga_write_i2c_value(ch_id, AF_SLAVE_ADDR, AF_DIRECTION_REG,
									moving_direction, ADDR8_DATA16, EFalse);
	fpga_write_i2c_value(ch_id, AF_SLAVE_ADDR, AF_TARGET_POS_REG,
									stmv_end, ADDR8_DATA16, EFalse);

	/* Step-move operation start */
	fpga_write_i2c_value(ch_id, AF_SLAVE_ADDR, 0x8A, 0x0D, ADDR8_DATA8, EFalse );

	/* TODO: non-blocking wait should be implemented here */
	/* Check if target met */
	bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, 0x8F, ADDR8_DATA8);


	while (((bData & 0x01) == 1) && (tryCnt < MAX_TRY_CNT))
	{
		bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, 0x8F, ADDR8_DATA8);
		tryCnt++;
		vTaskDelay(10);
	}
#else /* BOARD_VERSION_P1_1 */

	/* We already write E0 with value = 1 in the initialize steps */
	/* Now, we write the expected position only */
	/* stmv_end should now contain the calculated DAC code */
    stmv_end = stmv_end & 0xFFF0; // 12 bit data, mask out the lower order nibble
	fpga_write_i2c_value(ch_id, AF_SLAVE_ADDR, AF_STMVENDH_REG,
									stmv_end, ADDR8_DATA16, EFalse );

	//time_start = xTaskGetTickCount();
	/* wait for 5 msec before read */
	vTaskDelay(5);

	/* TODO: non-blocking wait should be implemented here */
	/* Check if target met */
	bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, AF_MSSET, ADDR8_DATA8);

	while (((bData & AF_MSSET_CHTGST) == 1) && (tryCnt < MAX_TRY_CNT))
	{
		vTaskDelay(5);
		bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, AF_MSSET, ADDR8_DATA8);
		tryCnt++;

	}

#endif
	if(tryCnt == MAX_TRY_CNT)
	{
		log_error("Failed to move sensor to location 0x%04x", stmv_end);
		rs = EFalse;
	}
	else
    {
        af->CurrentPosition = stmv_end;
        log_debug("Moved sensor to location : 0x%04x \n\r", stmv_end);
		rs = ETrue;
	}
	return rs;

}

UInt16 af_controller_get_current_pos(AFData *af, UInt8 ch_id)
{
	UInt16 wData;
	wData = fpga_read_i2c_value(ch_id,AF_SLAVE_ADDR,0x3c,ADDR8_DATA16);

	if(af)
		af->CurrentPosition = (Int16)wData; // Cache the current position.

	return wData;
}

STATIC Bool af_controller_internal_stop(AFData *af, UInt8 ch_id)
{
	UInt8 bData;

#ifdef BOARD_VERSION_P1
	bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, 0x8A, ADDR8_DATA8);
	if((bData & 0x01) != 0)
	{
		fpga_write_i2c_value(ch_id, AF_SLAVE_ADDR, 0x8A, bData & 0xfe,
															ADDR8_DATA8, ETrue);
	}
#else /* BOARD_VERSION_P1_1 */
	/* Move it to center position */
	af_controller_internal_move(af,0x0000,0,ch_id);
	/* Wait 30ms */
	vTaskDelay(30);
	/* Read 87h */
	bData = fpga_read_i2c_value(ch_id, AF_SLAVE_ADDR, AF_EQENBL, ADDR8_DATA8);
	/* Write 87h with bit 7 is 0 */
	bData &= 0xEF;
	fpga_write_i2c_value(ch_id, AF_SLAVE_ADDR, AF_EQENBL, bData,
														ADDR8_DATA8, EFalse);
	/* Disable AF control equalizer output */
	fpga_write_i2c_value(ch_id, AF_SLAVE_ADDR, AF_PIDZO, 0x00,
														ADDR8_DATA8, EFalse);
#endif
	/* Set all components standby */
	fpga_write_i2c_value(ch_id,AF_SLAVE_ADDR, AF_STANDBY_REG, 0x00,
														ADDR8_DATA8, EFalse);
	return ETrue;
}


STATIC Int16 af_controller_calculate_stmvend(AFData *af)
{
	return ComputeDacCode(af);
}

