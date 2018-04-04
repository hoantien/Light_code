#include "ucid.h"
#include "hal_flash_ex.h"
#include "mems.h"
#include <string.h>
UCSetting *current_uc, *active_uc;
/* TODO: Store it into RAM or MALLOC */
UInt8 flash_data[sizeof(UseCaseManager)];
UseCaseManager *ucs = (UseCaseManager *)flash_data;
#define INIT_USECASE(uc,index) \
			{ ucs->ucids[index].ucid = uc; \
			  ucs->ucids[index].numofelement = 0; \
			}

static Bool execute_setting(ElementSetting* es);
static Bool restore_setting(UInt16 ucid, UCSetting* uc);
static Bool check_prerequire_settings(UCSetting* uc);
static Bool is_ucid_valid(UInt16 ucid);
static UCSetting* get_uc_from_ucid(UInt16 ucid);
static void copy_settings(UCSetting *src,UCSetting *dst);
static void initialize_all_ucs(void);

static void initialize_all_ucs(void)
{
	UCID ucids[] = { UC_DISABLED, UC_UNKNOWN, UC_DEBUG, UC_PREVIEW,
			UC_HIRES_CAPTURE, UC_FOCAL_STACKING, UC_HDR_CAPTURE, UC_VIDEO,
			UC_FTM_QUICK_CHECK, UC_FTM_CALIBRATION
	};

	UInt8 i = 0;

	for(i = 0; i < ARRAY_COUNT(ucids); i++)
		INIT_USECASE(ucids[i],i);

	for(i = ARRAY_COUNT(ucids); i < MAX_USECASE; i++)
		INIT_USECASE(UC_DISABLED,i);

	ucs->numofusecase = MAX_USECASE;
	ucs->is_activated = 0;

}
static void copy_settings(UCSetting *src,UCSetting *dst)
{
	UInt8 i = 0;
	if(dst == NULL || src == NULL) return;
	for(i = 0; i < src->numofelement; i++)
	{
		dst->settings[i].element = src->settings[i].element;
		dst->settings[i].element_value = src->settings[i].element_value;
		dst->settings[i].cam_idx = src->settings[i].cam_idx;
	}
}
static Bool check_prerequire_settings(UCSetting *uc)
{
    Bool rs = ETrue;
    UInt8 i = 0, cam_idx = 0;
    UInt32 checked_cam_mask = 0;
    if(uc)
    {
        for(i = 0; i < uc->numofelement; i++)
        {
            cam_idx = uc->settings[i].cam_idx;
            if((checked_cam_mask & (1 << (cam_idx + 1))) == 0)
            {
                if( !StatusActivated(cam_idx,ERROR_SENSOR_I2C_DETECT_FAILURE) &&
                    !StatusActivated(cam_idx, MODULE_SW_STANDBY))
                {
                    log_error("%s not in SW STANDBY", CamDeviceTbl[cam_idx]->Name);
                    rs = EFalse;
                }
                checked_cam_mask |= (1 << (cam_idx + 1));
            }
        }
    }
    else
    {
        log_error("UCID is NULL");
        rs = EFalse;
    }
    return rs;
}

static Bool is_ucid_valid(UInt16 ucid)
{
	if(ucid >= MAX_USECASE || ucid <= 0)
		return EFalse;
	/* TODO: check if ucid is in the UCID enum */
	return ETrue;
}

static UCSetting* get_uc_from_ucid(UInt16 ucid)
{
	UInt8 i = 0;
	for(i = 0; i < MAX_USECASE; i++)
	{
		if(ucs->ucids[i].ucid == ucid)
			return &(ucs->ucids[i]);
	}
	return NULL;
}

char * element_to_string( ELEMENT element)
{
	switch (element) {
	case BURST_REQUESTED :
		return "BURST_REQUESTED";
		break;
	case RESOLUTION:
		return "RESOLUTION";
		break;
	case SENSITIVITY:
		return "SENSITIVITY";
		break;
	case EXPOSURE_TIME:
		return "EXPOSURE_TIME";
		break;
	case FOCAL_LEN :
		return "FOCAL_LEN";
		break;
	case FOCUS_DISTANCE :
		return "FOCUS_DISTANCE";
		break;
	case FOCUS_STATUS :
		return "FOCUS_STATUS";
		break;
	case FPS                :
		return "FPS";
		break;
	case VCM_POSITION:
		return "VCM_POSITION";
		break;
	case VCM_HALL:
		return "VCM_HALL";
		break;
	case LENS_POSITION:
		return "LENS_POSITION";
		break;
	case LENS_HALL:
		return "LENS_HALL";
		break;
	case MIRROR_POSITION:
		return "MIRROR_POSITION";
		break;
	case MIRROR_HALL:
		return "MIRROR_HALL";
		break;
	case MAX_ELEMENT:
		return "MAX_ELEMENT";
		break;
		default:
			return "UNKNOWN";
			break;
	}
}

Bool add_setting(ELEMENT element,UInt64 value, UInt8 cam_idx, UInt16 ucid)
{
	UInt8 i = 0;
	if(!is_ucid_valid(ucid))
	{
		log_error("Invalid ucid %x",ucid);
		return EFalse;
	}
	if(cam_idx > CAM_NUM_CAMERA_MAX)
	{
		log_error("Invalid camera index %d",cam_idx);
		return EFalse;
	}
	if(CamDeviceTbl[cam_idx] == NULL)
	{
		log_error("Camera not support!");
		return EFalse;
	}
	UCSetting *current_uc = get_uc_from_ucid(ucid);
	if(current_uc == NULL)
		return EFalse;
	for(i = 0; i < current_uc->numofelement; i++)
	{
		if(current_uc->settings[i].element == element && current_uc->settings[i].cam_idx == cam_idx)
		{
			current_uc->settings[i].element_value = value;
			if(ucs->active_ucid == ucid)
			    execute_setting(&(current_uc->settings[i]));
			return ETrue;
		}
	}
	/* Check number of settings have been added before adding new setting */
	if(current_uc->numofelement >= MAX_ELEMENT*CAM_NUM_CAMERA_MAX)
	{
	    log_error("Number of settings are maximum. Can't add more.");
        return EFalse;
	}
	current_uc->settings[current_uc->numofelement].element = element;
	current_uc->settings[current_uc->numofelement].element_value = value;
	current_uc->settings[current_uc->numofelement].cam_idx = cam_idx;
	if(ucs->active_ucid == ucid)
		execute_setting(&(current_uc->settings[current_uc->numofelement]));
	current_uc->numofelement++;
	return ETrue;
}

Bool inherit_setting(UInt16 src_ucid, UInt16 dst_ucid)
{
	UCSetting* src_uc = get_uc_from_ucid(src_ucid);
	UCSetting* dst_uc = get_uc_from_ucid(dst_ucid);
	if(src_uc == NULL || dst_uc == NULL) return EFalse;
	copy_settings(src_uc,dst_uc);
	return ETrue;
}

static Bool execute_setting(ElementSetting* es)
{
	if(!es)
		return EFalse;

	UInt8 cam_idx = es->cam_idx;
	if(cam_idx > CAM_NUM_CAMERA_MAX)
	{
	    log_error("Camera is not valid");
	    return EFalse;
	}
	CamDevice_TypeDef *cam = CamDeviceTbl[cam_idx];
	if(cam == NULL)
		return EFalse;
	if(!StatusActivated(cam_idx, MODULE_SW_STANDBY))
	{
	    log_error("Can't apply setting as camera isn't in SW Standby");
	    return EFalse;
	}

	log_debug("Apply setting ID: 0x%x on %s ", es->element, cam->Name);
	switch(es->element)
	{
		case RESOLUTION:
		{
			log_debug("res_x 0x%x\r\n",(unsigned int)(es->element_value >> 32));
			log_debug("res_y 0x%x\r\n",(unsigned int)(es->element_value & 0xffffffff));
			Set_Resolution(cam,(UInt32)(es->element_value >> 32), (UInt32) es->element_value);
			break;
		}
		case SENSITIVITY:
		{
			Set_Sensitivity(cam,es->element_value);
			break;
		}
		case EXPOSURE_TIME:
		{
			Set_Exposure(cam,es->element_value);
			break;
		}
		case FPS:
		{
			Set_FPS(cam, (UInt16)(es->element_value & 0xFFFF));
			break;
		}
		case VCM_POSITION:
		{
			UInt8 ret;
			if(check_cam_focus_status_actived(cam_idx, MOVING_ERROR))
			{
				reset_cam_focus_status(cam_idx, MOVING_ERROR);
			}
			set_cam_focus_status(cam_idx, MOVING);
			ret = af_controller_move(cam,es->element_value,EFalse);
			if(ret == EFalse)
			{
				reset_cam_focus_status(cam_idx, MOVING);
				set_cam_focus_status(cam_idx, MOVING_ERROR);
			}
			else
			{
				reset_cam_focus_status(cam_idx, MOVING);
				set_cam_focus_status(cam_idx,IDLE);
			}
			break;
		}
		case FOCUS_DISTANCE:
		{
			UInt8 ret;
			if(check_cam_focus_status_actived(cam_idx, MOVING_ERROR))
			{
				reset_cam_focus_status(cam_idx, MOVING_ERROR);
			}

			set_cam_focus_status(cam_idx, MOVING);
		    if(cam->Type == CAM_TYPE_35MM)
		    {
		        ret = af_controller_move(cam,es->element_value,ETrue);
		        if(ret == EFalse)
		        {
					reset_cam_focus_status(cam_idx, MOVING);
					set_cam_focus_status(cam_idx, MOVING_ERROR);
		        }
		        else
		        {
		        	reset_cam_focus_status(cam_idx, MOVING);
					set_cam_focus_status(cam_idx, IDLE);
		        }
		    }
		    else
		    {
		        Error_t e;
                PiezoModule *m;

                m = CameraIdToModule(cam_idx+1); // +1 converts from i to bitmask index
                e = PiezoMoveToDistance(m->Lens, (UInt32) es->element_value);
                if (e)
                {
                    log_debug("MovePiezoToDistance(Lens) returned %d", e);
                }
		    }
			break;
		}
		case LENS_POSITION:
		{
		    if(cam->Type == CAM_TYPE_70MM || cam->Type == CAM_TYPE_150MM)
		    {
		        UInt16 DutyCycle = 90; // DUTY_LO
                UInt16 Tolerance =  5;
                Error_t e;
                PiezoModule *m;

                m = CameraIdToModule(cam_idx+1); // +1 converts from i to bitmask index
                e = MovePiezoToPosition(m->Lens, es->element_value, Tolerance, DutyCycle);
                if (e)
                {
                    log_debug("MovePiezoToPosition(Lens) returned %d", e);
                }
		    }

			break;
		}
		case LENS_HALL:
		{
		    PiezoActuator *a;
            PiezoModule   *m;
            UInt16 data;
            volatile ccb_cmd_base_t * cam_m_status = (ccb_cmd_base_t *) CCB_M_WRITE;

            m = CameraIdToModule(cam_idx + 1); // +1 converts from i to bitmask index
            a = m->Lens;
            if (!a->IsConnected)
            {
                log_error("Please connect piezo Lens of %s before use", cam->Name);
            }
            else
            {
                if (!(a->Hall.IsInitialized))
                {
                    ReadEepromCalibration(cam_idx, &(m->Lens->CalibData), &(m->Mirror->CalibData));
                    InitHallSensor(&(a->Hall));
                }
                data = ReadHallSensor(&(a->Hall));
                cam_m_status->cam_m_lens_hall[cam_idx] = (data>>8) & 0x0ff;
                cam_m_status->cam_m_lens_hall[cam_idx + 1] =  data     & 0x0ff;
                log_printf("%s LENS value : 0x%04x, S:%d, P:%d\n\r",
                        cam->Name,
                        data,
                        a->Hall.Sensitivity, // these fields are valid, see drv_piezo_hall.h
                        a->Hall.Polarity);
            }
			break;
		}
		case MIRROR_POSITION:
		{
		    if(cam->Type == CAM_TYPE_70MM || cam->Type == CAM_TYPE_150MM)
            {
		        UInt16 DutyCycle = 90; // DUTY_LO
                UInt16 Tolerance =  5;
                Error_t e;
                PiezoModule *m;

                m = CameraIdToModule(cam_idx+1); // +1 converts from i to bitmask index
                e = MovePiezoToPosition(m->Mirror, es->element_value, Tolerance, DutyCycle);
                if (e)
                {
                    log_debug("MovePiezoToPosition(Lens) returned %d", e);
                }
            }
			break;
		}
		case MIRROR_HALL:
		{
		    PiezoActuator *a;
            PiezoModule   *m;
            UInt16 data;
            volatile ccb_cmd_base_t * cam_m_status = (ccb_cmd_base_t *) CCB_M_WRITE;

            m = CameraIdToModule(cam_idx + 1); // +1 converts from i to bitmask index
            a = m->Mirror;
            if (!a->IsConnected)
            {
                log_error("Please connect piezo mirror of %s before use", cam->Name);
            }
            else
            {
                if (!(a->Hall.IsInitialized))
                {
                    ReadEepromCalibration(cam_idx, &(m->Lens->CalibData), &(m->Mirror->CalibData));
                    a->Hall.Sensitivity = m->Mirror->CalibData.Sensitivity;
                    a->Hall.Polarity = m->Mirror->CalibData.Polarity;
                    InitHallSensor(&(a->Hall));
                }
                data = ReadHallSensor(&(a->Hall));
                cam_m_status->cam_m_mirror_hall[cam_idx] = (data>>8) & 0x0ff;
                cam_m_status->cam_m_mirror_hall[cam_idx + 1] =  data     & 0x0ff;
                log_printf("%s MIRRORS value : 0x%04x, S:%d, P:%d\n\r",
                        cam->Name,
                        data,
                        a->Hall.Sensitivity, // these fields are valid, see drv_piezo_hall.h
                        a->Hall.Polarity);
            }
            break;
		}
		case VCM_HALL:
		case BURST_REQUESTED:
		case FOCAL_LEN:
		case FOCUS_STATUS:
		{
			break;
		}
		default:
		{
			/* Currently, these are not implemented */
			break;
		}
	}
	return ETrue;
}

static Bool restore_setting(UInt16 ucid, UCSetting* uc)
{
	UInt8 i = 0;
	if(!is_ucid_valid(ucid))
	{
		log_error("Invalid ucid %x",ucid);
		return EFalse;
	}
	if(uc == NULL)
		return EFalse;
	if(uc->ucid != ucid)
		return EFalse;
	if(uc->numofelement == 0)
    {
	    log_warning("No setting has been set for %s", ucid_to_text(ucid));
	    return ETrue;
    }
	log_info("Start apply %d setting(s) for %s", uc->numofelement, ucid_to_text(ucid));
	for(i = 0; i < uc->numofelement; i++)
		execute_setting(&(uc->settings[i]));
	log_info("Done apply settings for %s", ucid_to_text(ucid));
	return ETrue;
}

UInt8 get_last_setting(UInt8 cam_idx, UInt16 ucid, UInt16 element, UInt64 *value)
{
    UInt8 loop_idx = 0;

    if(is_ucid_valid(ucid) == EFalse)
    {
        log_error("UCID %d is not valid", ucid);
        return 0;
    }

    current_uc = get_uc_from_ucid(ucid);
    if(current_uc == NULL)
    {
        log_error("UCID %d is not valid", ucid);
        return 0;
    }

    for(loop_idx = 0; loop_idx < current_uc->numofelement; loop_idx++)
    {
        if(current_uc->settings[loop_idx].cam_idx == cam_idx &&
           current_uc->settings[loop_idx].element == element )
        {
            *value = current_uc->settings[loop_idx].element_value;
            return 1;
        }
    }

    log_debug("Can't find required setting from cache");
    return 0;
}

UInt16 get_active_ucid(void)
{
	return ucs->active_ucid;
}

char* ucid_to_text(UInt16 ucid)
{
    switch(ucid)
    {
        case UC_DISABLED:
        {
            return "UC_DISABLED";
        }
        case UC_PREVIEW:
        {
            return "UC_PREVIEW";
        }
        case UC_FOCAL_STACKING:
        {
            return "UC_FOCAL_STACKING";
        }
        case UC_FTM_CALIBRATION:
        {
            return "UC_FTM_CALIBRATION";
        }
        case UC_FTM_QUICK_CHECK:
        {
            return "UC_FTM_QUICK_CHECK";
        }
        case UC_HDR_CAPTURE:
        {
            return "UC_HDR_CAPTURE";
        }
        case UC_HIRES_CAPTURE:
        {
            return "UC_HIRES_CAPTURE";
        }
        case UC_VIDEO:
        {
            return "UC_VIDEO";
        }
        default:
        {
            return "UC_UNKNOWN";
        }
    }
}

Bool activate_ucid(UInt16 ucid)
{
    Bool rs = EFalse, sta = EFalse;
	active_uc = get_uc_from_ucid(ucid);
	/* Ignore if UC_DISABLED */
	if(ucid == UC_DISABLED)
	{
	    log_error("UCID DISABLED can't be activated");
	    return EFalse;
	}
	if(ucid == get_active_ucid() && ucs->is_activated == 1)
	{
	    log_info("%s is already activated", ucid_to_text(ucid));
	    return ETrue;
	}
	ucs->active_ucid = ucid;
	ucs->is_activated = 0;
	if(active_uc == NULL)
	{
		/* TODO: Enable this */
		log_error("No use-case is corresponding to ucid %d",ucid);
		return EFalse;
	}
#if 0
	/* Check if UCID has settings */
	if(ucid != UC_FTM_QUICK_CHECK && active_uc->numofelement == 0) // No setting
	{
		/* TODO: Enable this */
		Set_CMD_Status(CMD_INVALID_ARG);
		log_error("No setting is specified for this ucid %d",ucid);
		return EFalse;
	}
#endif
	if(ucid == UC_FTM_QUICK_CHECK)
	{

		ucs->active_ucid = ucid;
		/* Check whether all present cameras is open */
		for(UInt8 i = 0; i < CAM_NUM_CAMERA_MAX; i++)
		{
			if(StatusActivated(i,ERROR_SENSOR_I2C_DETECT_FAILURE))
				continue;
			if(!StatusActivated(i,MODULE_OPEN))
			{
//				log_printf("Cam index %x is not open\n\r",i);
				return EFalse;
			}
		}
		/* TODO: FTM QuickCheck Sequence
		* - Send ucid to FPGA
		* - Wait for interrupt pull low
		* - Send status command 0xFF01
		* - Sends 16 x 16 bit commands of 0 (or anything really) so that 16 x 16 bits of status can be read out of FPGA.
		* - Set corresponding CAM_MODULE_STATUS
		*/
		/* Below are code for stm-ccb */
		FTM_cam_modules();
		ucs->is_activated = 1;
		return ETrue;
	}
	/* Check prerequire settings for a ucid */
	sta = check_prerequire_settings(active_uc);
	if(sta == EFalse)
	{
		log_error("Prerequired settings aren't fulfill");
		return EFalse;
	}
	ucs->is_activated = 1;
	rs = restore_setting(ucid,active_uc);
	if(rs == ETrue)
		ucs->active_ucid = ucid;
	else
		log_warning("Some settings are not applied successfully!");
	log_info("%s has been activated", ucid_to_text(ucid));
#ifdef __TEST_UCID
	dumpData(ucs);
#endif
	return rs;
}

Bool save_all_usecase(void)
{
	return ETrue;
}

UInt16 is_active_ucid_activated()
{
    return ucs->is_activated;
}

Bool initialize_usecase_manager(void)
{
	initialize_all_ucs();
#ifdef __TEST_UCID
	dumpData(ucs);
#endif
	return ETrue;
}


#ifdef __TEST_UCID
void dumpData(UseCaseManager *ucm)
{
	log_debug("--------------------------------------");
	log_debug("- Num of use-case: %d",ucm->numofusecase);
	log_debug("- Active use-case %d",ucm->active_ucid);
	log_debug("- Current setting use-case %d",ucm->current_ucid);
	for(int i = 0; i < MAX_USECASE; i ++)
	{
		log_debug("----DUMPING USE-CASE %d-----------",i);
		log_debug("- UCID: %d",ucm->ucids[i].ucid);
		log_debug("- Num of settings %d",ucm->ucids[i].numofelement);
	}
}
#endif
