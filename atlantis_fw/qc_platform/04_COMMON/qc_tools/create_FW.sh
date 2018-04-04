#!/bin/bash
# Help
echo "Usage:"
echo "\$create_FW.sh [s1] [s2] [sb_tool_path] [merge_img_tool] [out_path] [out_file]"

s1=$1
s2=$2
sb_tool_path=$3
merge_img_tool=$4
out_path=$5
out_file=$6

# Remove old files
rm -rf $out_path/$out_file $out_path/AsicFwS1.bin.imghdr $out_path/AsicFwS2.bin.imghdr $out_path/S2_sys.sysimg $out_path/S1_sys.sysimg

# Change to executable for SB tool
chmod 777 $sb_tool_path/sbtool

$sb_tool_path/sbtool phase0bimghdr $s1 $out_path/AsicFwS1.bin.imghdr relative 0x154 relative 0x154
$sb_tool_path/sbtool phase0bsyshdr $sb_tool_path/light_cert_0630 12345678 0x1234 $out_path/S1_sys.sysimg $out_path/AsicFwS1.bin.imghdr
$sb_tool_path/sbtool phase0bimghdr $s2 $out_path/AsicFwS2.bin.imghdr relative 0x154 relative 0x154
$sb_tool_path/sbtool phase0bsyshdr $sb_tool_path/light_cert_0630 12345678 0x1234 $out_path/S2_sys.sysimg $out_path/AsicFwS2.bin.imghdr
echo "\$python $merge_img_tool $out_path $out_path/S1_sys.sysimg $s2 $out_pathS2_sys.sysimg $s2"

# Merge images
python $merge_img_tool $out_path/$out_file $out_path/S1_sys.sysimg $s1 $out_path/S2_sys.sysimg $s2
