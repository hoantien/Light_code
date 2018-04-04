#!/user/bin/python
import sys
import os
import struct
import array

if len(sys.argv) < 2:
	sys.exit(0)

output_file=sys.argv[1]
file_count = len(sys.argv)
i = 2
of = open(output_file, "w")
of.write("")
of.close()
while i < file_count:
	of = open(output_file, "ab+")
	of.seek(0, os.SEEK_END)
	with open(sys.argv[i], mode="rb") as file:
		print "Reading "+ sys.argv[i]
		fileContent = file.read()
		of.write(fileContent)
	of.close()
	i += 1

