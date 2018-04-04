#Manual control
adb shell  "echo 1 > /sys/class/light_ccb/common/manual_control"
#open camera sw standby:
python camera_script_asb.py -c b5 -o sw
#ucid preview::
python camera_script_asb.py -u preview
#stream on tx 0, vc0:
python camera_script_asb.py -c b5 -s on -tx 0 -vc 0
