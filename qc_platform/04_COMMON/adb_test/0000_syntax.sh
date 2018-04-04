echo 6 0x0001 0x00 0x00 0x02 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 5 0x0000 0x00 0x00 0x02 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 1 0x0000 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br > /data/output

echo 6 0x0002 0x00 0x00 0x02 0x00 0x00 0x01 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 5 0x0000 0x00 0x00 0x02 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 1 0x0000 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 6 0x0003 0x00 0x00 0x02 0x00 0x00 0x02 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 5 0x0000 0x00 0x00 0x02 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 1 0x0000 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 7 0x0004 0x00 0x00 0x06 0x00 0x00 0x01 0x02 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 5 0x0000 0x00 0x00 0x06 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 2 0x0000 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 6 0x0005 0x00 0x00 0x01 0x00 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 5 0x0000 0x00 0x00 0x01 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 16 0x0000 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 7 0x0006 0x00 0x00 0x01 0x00 0x00 0x02 0x02 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 5 0x0000 0x00 0x00 0x01 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 16 0x0000 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 6 0x0007 0x00 0x00 0x06 0x00 0x00 0x02 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 5 0x0000 0x00 0x00 0x06 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 2 0x0000 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

echo 6 0x0008 0x00 0x00 0x06 0x00 0x00 0x03 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 5 0x0000 0x00 0x00 0x06 0x00 0x00 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_w
echo 2 0x0000 > /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br
cat /sys/class/i2c-adapter/i2c-11/i2c-0/0-0010/i2c_br >> /data/output

cat /data/output
