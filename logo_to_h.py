#!/usr/bin/python
# read an ascii ppm generated by gimp
#
import sys
fn = sys.argv[1]
fh = open(fn, "r")
out = open("src/logo.h", "w")

header    = fh.readline()
comment   = fh.readline()
dimension = fh.readline()
width     = int(dimension.split()[0])
height    = int(dimension.split()[1])
maxcol    = fh.readline()

x = 0
y = 0
byte_out = 0
byte_count = 0;
# width has to be a multiple of 8:
if not (width % 8 == 0):
    print("ERROR: image width has to be a multiple of 8!")
    exit(0)

out.write("#ifndef LOGO_H__\n")
out.write("#define LOGO_H__\n")
out.write("#include <stdint.h>\n")
out.write("\n")
out.write("#define LOGO_WIDTH " + str(width) + "\n")
out.write("#define LOGO_HEIGHT " + str(height) + "\n")
out.write("\n")
out.write("static const uint8_t logo_data[LOGO_HEIGHT * (LOGO_WIDTH/8)] = {\n")
out.write("    ");

count = 0;

for line in fh:
    #print(line)
    # skip blue and green:
    fh.next()
    fh.next()

    byte_out = (byte_out << 1)
    if (int(line) != 0) : 
        byte_out = byte_out | 1

    if (byte_count == 8):
        # byte finished!
        out.write(hex(byte_out) + ", ") 
        byte_count = 0
        byte_out   = 0
    if (x == width):
        x = 0
        y = y + 1 

    if ((count % (8*16)) == 0):
        out.write("\n    ")
    count = count + 1
    print (str(x) + " " + str(y))
    x = x + 1
    byte_count = byte_count + 1

out.write("\n};")
out.write("\n#endif  // LOGO_H__\n")
fh.close()
out.close()