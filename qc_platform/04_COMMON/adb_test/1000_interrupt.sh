echo 4 0x0001 0x00 0x90 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0002 0x00 0x90 0x01 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0003 0x00 0x90 0x02 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0004 0x00 0x90 0x03 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0005 0x00 0x90 0x04 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0006 0x00 0x90 0x05 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0007 0x00 0x90 0x06 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0008 0x00 0x90 0x07 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x0009 0x00 0x90 0x08 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x000A 0x00 0x90 0x09 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 4 0x000B 0x00 0x90 0x0A 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br > /data/output

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 4 0x027C > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

cat /data/output
