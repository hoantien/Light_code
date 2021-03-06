#!/bin/bash

# camera bitmask for 16 CAM:A1,A2,A3,A4,A5,
#                           B1,B2,B3,B4,B5,
#                           C1,C2,C3,C4,C5,
#                           C6
cam_bitmask=("0x02 0x00 0x00" "0x04 0x00 0x00" "0x08 0x00 0x00" "0x10 0x00 0x00" "0x20 0x00 0x00"
             "0x40 0x00 0x00" "0x80 0x00 0x00" "0x00 0x01 0x00" "0x00 0x02 0x00" "0x00 0x04 0x00" \
             "0x00 0x08 0x00" "0x00 0x10 0x00" "0x00 0x20 0x00" "0x00 0x40 0x00" "0x00 0x80 0x00" \
             "0x00 0x00 0x01")

# camera bitmask for camera groups: (A1,B2,C3), (A2,B3,C4),
#                                   (A3,B4,C5), (A4,B5,C6),
#                                   (A5,B1,C1),(A1,B5,C2)
cam_m_bitmask=("0x82 0x20 0x00" "0x04 0x41 0x00" \
               "0x08 0x82 0x00" "0x10 0x04 0x01" \
               "0x60 0x08 0x00" "0x02 0x14 0x00")

# camera bitmask for camera groups: (A1,A2,A3), (A4,A5,A1),
#cam_m_bitmask=("0x0E 0x00 0x00" "0x32 0x00 0x00")

# CMD Test result
cmd_test_result=("CAM_MODULE_OPEN.txt" \
				 "CAM_STREAMING.txt" \
				 "CAM_COMMAND_STATUS.txt" \
				 "CAM_MODULE_STATUS.txt" \
				 "CAM_MODULE_RESOLUTION.txt" \
				 "CAM_MODULE_SENSITIVITY.txt" \
				 "CAM_MODULE_EXPOSURE_TIME.txt" \
				 "CAM_MODULE_FPS.txt" \
				 "CAM_MODULE_FOCAL_LEN.txt" \
				 "CAM_MODULE_FOCUS_DISTANCE.txt" \
				 "CAM_MODULE_FOCUS_STATUS.txt" \
				 "CAM_SNAPSHOT_UUID.txt")


################# TEST START CAM_MODULE_OPEN (0x0000) ################# 
sudo echo 'TEST START CAM_MODULE_OPEN (0x0000)'
sleep 1
	# Test Open with G enabled in SW_Standby mode
	#sudo ./FT4222_FLASH cmd 0x08 0x8 0x00 0x00 0x00 0x00 0x01 0x00 0x00 0x02
	sleep 1

	# Test Open with G enabled in HW_Standby mode
	#sudo ./FT4222_FLASH cmd 0x08 0x8 0x00 0x00 0x00 0x00 0x01 0x00 0x00 0x01
	sleep 1

	# - Test Open 16 cameras in SW_Standby mode
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x8 0x00 0x00 0x00 0x00 ${cam_bitmask[$CAM]} 0x02
		sleep 1
	done
	
	# - Test Open 16 cameras in HW_Standby mode
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x8 0x00 0x00 0x00 0x00 ${cam_bitmask[$CAM]} 0x01
		sleep 1
	done

	# Test open selected camera with m-bitmask in SW_Standby mode
	for CAMS in ${!cam_m_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0xA 0x00 0x00 0x00 0x00 ${cam_m_bitmask[$CAMS]} 0x02 0x02 0x02
		sleep 1
	done

	# Test Close all of cameras with G enabled.
	#sudo ./FT4222_FLASH cmd 0x08 0x8 0x00 0x00 0x00 0x00 0x01 0x00 0x00 0x00
	sleep 1

	# Test Close cameras with m_bitmask
	for CAMS in ${!cam_m_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0xA 0x00 0x00 0x00 0x00 ${cam_m_bitmask[$CAMS]} 0x00 0x00 0x00
		sleep 1
	done

	# Test Close one camera.
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x8 0x00 0x00 0x00 0x00 ${cam_bitmask[$CAM]} 0x00
		sleep 1
	done
echo 'TEST END CAM_MODULE_OPEN (0x0000)'
################# TEST END CAM_MODULE_OPEN (0x0000) ################# 

################# TEST START CAM_STREAMING (0x0002) ################# 
echo 'TEST START CAM_STREAMING (0x0002)'
sleep 1
	# Test stream-on 16 cameras .
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0xA 0x00 0x00 0x02 0x00 ${cam_bitmask[$CAM]} 0x11 0x00 0x00
		sleep 1
	done
	
	# Test stream on all camera with G enabled.
	#sudo ./FT4222_FLASH cmd 0x08 0xA 0x00 0x00 0x02 0x00 0x01 0x00 0x00 0x11 0x00 0x00
	sleep 1
	
	# Test stream on cameras with m_bitmask
	for CAMS in ${!cam_m_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x10 0x00 0x00 0x02 0x00 ${cam_m_bitmask[$CAMS]} 0x11 0x00 0x00 0x11 0x00 0x00 0x11 0x00 0x00
		sleep 1
	done
	
	# Test stream-off 16 cameras	
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0xA 0x00 0x00 0x02 0x00 ${cam_bitmask[$CAM]} 0x10 0x00 0x00
		sleep 1
	done
	
	# Test stream-off all camera with G enabled.
	#sudo ./FT4222_FLASH cmd 0x08 0xA 0x00 0x00 0x02 0x00 0x01 0x00 0x00 0x10 0x00 0x00
	sleep 1
	
	# Test stream-off cameras with m_bitmask
	for CAMS in ${!cam_m_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x10 0x00 0x00 0x02 0x00 ${cam_m_bitmask[$CAMS]} 0x10 0x00 0x00 0x10 0x00 0x00 0x10 0x00 0x00
		sleep 1
	done
echo 'TEST END CAM_STREAMING (0x0002)'
################# TEST START CAM_STREAMING (0x0002) #################

################# TEST START CAM_COMMAND_STATUS (0x0024) ################# 
echo 'TEST START CAM_COMMAND_STATUS (0x0024)'
sleep 1
	# Test to get status code for transaction 0x1234
	sudo ./FT4222_FLASH cmd 0x08 0x06 0x00 0x00 0x24 0x00 0x34 0x12
	sleep 1
echo 'TEST END CAM_COMMAND_STATUS (0x0024)'
################# TEST END CAM_COMMAND_STATUS (0x0024) ################# 

################# TEST START CAM_MODULE_STATUS (0x0028) ################# 
echo 'TEST START CAM_MODULE_STATUS (0x0028)'
sleep 1
	# Test to get module status of cam A1, B2
	sudo ./FT4222_FLASH cmd 0x08 0x07 0x00 0x00 0x28 0x00 0x82 0x00 0x00
	sleep 1
echo 'TEST END CAM_MODULE_STATUS (0x0028)'
################# TEST END CAM_MODULE_STATUS (0x0028) ################# 

################# TEST START CAM_MODULE_RESOLUTION (0x002C) #################
echo 'TEST START CAM_MODULE_RESOLUTION (0x002C)'
sleep 1
	# Test to set resolution for cam A1
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x11 0x00 0x00 0x2C 0x00 ${cam_bitmask[$CAM]} 0x03 0x00 0x80 0x07 0x00 0x00 0x38 0x04 0x00 0x00
		sleep 1
	done
	
	# Test to set resolution with m-bitmask
	for CAMS in ${!cam_m_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x21 0x00 0x00 0x2C 0x00 ${cam_m_bitmask[$CAMS]} 0x03 0x00 0x80 0x07 0x00 0x00 0x38 0x04 0x00 0x00 0x80 0x07 0x00 0x00 0x38 0x04 0x00 0x00 0x80 0x07 0x00 0x00 0x38 0x04 0x00 0x00
		sleep 1		
	done
	
	# Test to set resolution with G enabled
	#sudo ./FT4222_FLASH cmd 0x08 0x11 0x00 0x00 0x2C 0x00 0x01 0x00 0x00 0x03 0x00 0x80 0x07 0x00 0x00 0x38 0x04 0x00 0x00
	sleep 1
echo 'TEST END CAM_MODULE_RESOLUTION (0x002C)'
################# TEST START CAM_MODULE_RESOLUTION (0x002C) ################# 

################# TEST START CAM_MODULE_SENSITIVITY (0x0030) #################
echo 'TEST START CAM_MODULE_SENSITIVITY (0x0030)'
sleep 1
	# Test to set sensitivity 200 for 16 cameras
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x0D 0x00 0x00 0x30 0x00 ${cam_bitmask[$CAM]} 0x03 0x00 0x01 0x10 0x00 0x00
		sleep 1
	done
	
	# Test to set sensitivity 200 with m-bitmask
	for CAMS in ${!cam_m_bitmask[*]}
	do		
		sudo ./FT4222_FLASH cmd 0x08 0x15 0x00 0x00 0x30 0x00 ${cam_m_bitmask[$CAMS]} 0x03 0x00 0x01 0x10 0x00 0x00 0x01 0x10 0x00 0x00 0x01 0x10 0x00 0x00
		sleep 1
	done
	
	# Test to set sensitivity 200 with G enabled.
	#sudo ./FT4222_FLASH cmd 0x08 0x0D 0x00 0x00 0x30 0x00 0x01 0x00 0x00 0x03 0x00 0x01 0x10 0x00 0x00
	sleep 1
echo 'TEST END CAM_MODULE_SENSITIVITY (0x0030)'
################# TEST END CAM_MODULE_SENSITIVITY (0x0030) #################

################# TEST START CAM_MODULE_EXPOSURE_TIME (0x0032) #################
echo 'TEST START CAM_MODULE_EXPOSURE_TIME (0x0032)'
sleep 1
	# Test to set exposure time for 16 cameras
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x11 0x00 0x00 0x32 0x00 ${cam_bitmask[$CAM]} 0x03 0x00 0xE8 0x03 0x00 0x00 0x00 0x00 0x00 0x00
		sleep 1
	done

	# Test to set exposure time with m-bitmask
	for CAMS in ${!cam_m_bitmask[*]}
	do	
		sudo ./FT4222_FLASH cmd 0x08 0x21 0x00 0x00 0x32 0x00 ${cam_m_bitmask[$CAMS]} 0x03 0x00 0xE8 0x03 0x00 0x00 0x00 0x00 0x00 0x00 0xE8 0x03 0x00 0x00 0x00 0x00 0x00 0x00 0xE8 0x03 0x00 0x00 0x00 0x00 0x00 0x00
		sleep 1	
	done
	
	# Test to set exposure time with G enabled
	#sudo ./FT4222_FLASH cmd 0x08 0x11 0x00 0x00 0x32 0x00 0x01 0x00 0x00 0x03 0x00 0xE8 0x03 0x00 0x00 0x00 0x00 0x00 0x00
	sleep 1
echo 'TEST END CAM_MODULE_EXPOSURE_TIME (0x0032)'
################# TEST END CAM_MODULE_EXPOSURE_TIME (0x0032) #################

################# TEST START CAM_MODULE_FPS (0x0050) #################
echo 'TEST START CAM_MODULE_FPS (0x0050)'
sleep 1
	# Test to set fps for 16 cameras
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x0B 0x00 0x00 0x50 0x00 ${cam_bitmask[$CAM]} 0x03 0x00 0x1E 0x00
		sleep 1
	done
	
	# Test to Set fps with m-bitmask
	for CAMS in ${!cam_m_bitmask[*]}
	do	
		sudo ./FT4222_FLASH cmd 0x08 0x0F 0x00 0x00 0x50 0x00 ${cam_m_bitmask[$CAMS]} 0x03 0x00 0x1E 0x00 0x1E 0x00 0x1E 0x00
		sleep 1	
	done
	
	# Test to Set fps with G enabled.
	#sudo ./FT4222_FLASH cmd 0x08 0x0B 0x00 0x00 0x50 0x00 0x01 0x00 0x00 0x03 0x00 0x1E 0x00
	sleep 1
echo 'TEST END CAM_MODULE_FPS (0x0050)'
################# TEST END CAM_MODULE_FPS (0x0050) #################

################# TEST START CAM_MODULE_FOCAL_LEN (0x003A) #################
echo 'TEST START CAM_MODULE_FOCAL_LEN (0x003A)'
sleep 1
	# Test to Set focal len for 16 cameras
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x0B 0x00 0x00 0x3A 0x00 ${cam_bitmask[$CAM]} 0x03 0x00 0x32 0x00
		sleep 1
	done
	
	# Test to Set focal len with m-bitmask
	for CAMS in ${!cam_m_bitmask[*]}
	do		
		sudo ./FT4222_FLASH cmd 0x08 0x0F 0x00 0x00 0x3A 0x00 ${cam_m_bitmask[$CAMS]} 0x03 0x00 0x32 0x00 0x32 0x00 0x32 0x00
		sleep 1
	done

	# Test to set focal len with G enabled.
	#sudo ./FT4222_FLASH cmd 0x08 0x0B 0x00 0x00 0x3A 0x00 0x01 0x00 0x00 0x03 0x00 0x32 0x00
	sleep 1
echo 'TEST END CAM_MODULE_FOCAL_LEN (0x003A)'
################# TEST END CAM_MODULE_FOCAL_LEN (0x003A) #################

################# TEST START CAM_MODULE_FOCUS_DISTANCE (0x0048) #################
echo 'TEST START CAM_MODULE_FOCUS_DISTANCE (0x0048)'
sleep 1
	# Test to set focus distance for 16 cameras
	for CAM in ${!cam_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x0D 0x00 0x00 0x48 0x00 ${cam_bitmask[$CAM]} 0x03 0x00 0x96 0x00 0x00 0x00
		sleep 1
	done
	
	# Test to set focus distance with m-bitmask
	for CAMS in ${!cam_m_bitmask[*]}
	do
		sudo ./FT4222_FLASH cmd 0x08 0x15 0x00 0x00 0x48 0x00 ${cam_m_bitmask[$CAMS]} 0x03 0x00 0x96 0x00 0x00 0x00 0x96 0x00 0x00 0x00 0x96 0x00 0x00 0x00
		sleep 1
	done
	
	# Test to set focus distance with G enabled
	#sudo ./FT4222_FLASH cmd 0x08 0x0D 0x00 0x00 0x48 0x00 0x01 0x00 0x00 0x03 0x00 0x96 0x00 0x00 0x00
	sleep 1
echo 'TEST END CAM_MODULE_FOCUS_DISTANCE (0x0048)' 
################# TEST END CAM_MODULE_FOCUS_DISTANCE (0x0048) #################

################# TEST START CAM_SNAPSHOT_UUID (0x0006) #################
echo 'TEST START CAM_SNAPSHOT_UUID (0x0006)' 
sudo ./FT4222_FLASH cmd 0x08 0x14 0x00 0x00 0x06 0x00 0x09 0x08 0x07 0x06 0x05 0x04 0x03 0x02 0x01 0x00 0x0A 0x0B 0x0C 0x0D 0x0E 0x0F
sleep 1
echo 'TEST END CAM_SNAPSHOT_UUID (0x0006)'
################# TEST END CAM_SNAPSHOT_UUID (0x0006) #################
