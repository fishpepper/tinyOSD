#!/usr/bin/python
from __future__ import print_function
import math

print ("static const uint32_t video_cos_table[90] = {")
for x in range(90):
    if (x % 8 == 0):
        print()
    print(hex(int(math.cos(x *  3.14 / 180.0) * 128)) + ",", end='')
print("};")
print()
