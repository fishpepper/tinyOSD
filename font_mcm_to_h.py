#!/usr/bin/python
# read an ascii ppm generated by gimp
#
import sys
import png

if len(sys.argv) < 2:
    print "usage: " + sys.argv[0] + " file.mcm\n\n"
    sys.exit(1)

fn = sys.argv[1]
fh = open(fn, "r")

#debug output image
output_w = 192
output_h = 287
pngh = open('font.png', 'wb')
pngw = png.Writer(output_w, output_h, greyscale=True)


#skip MAX7456 coment
version = fh.readline().strip()

font_data = []

for n in range(256):
    for l in range(54):
        line = fh.readline().strip()
        byte = int(line, 2)
        font_data.append(byte)
    for l in range(10):
        # skip 10 dummy bytes
        fh.readline()

# reformat to tinyOSD dataset
font_black = []
font_white = []

for data in font_data:
    for i in range(4):
        bitset = (data & 0xC0)>>6
        if (bitset == 0b00):
            # black
            font_black.append(1)
            font_white.append(0)
        elif (bitset == 0b10):
            # white
            font_black.append(0)
            font_white.append(1)
        else:
            # transparent
            font_black.append(0)
            font_white.append(0)
        data = data << 2
    
# debug output png
# font data is one char after the other (12px x 18px)
pngdata = []
for y in range(output_h):
    row = []
    for x in range(output_w):
        # fetch index. we have chars from left to right, top to bot
        char = (y/18)*16+(x/12)
        #print "char %d at %d x %d\n" %(char, x, y)
        idx  = char*(12*18) + (x%12) + ((y%18)*12) 
        if (font_black[idx] == 1):
            color = 0
        elif (font_white[idx] == 1):
            color = 255
        else:
            color = 128
        row.append(color)
    pngdata.append(row)

#print(pngdata)

pngw.write(pngh, pngdata)

pngh.close()


