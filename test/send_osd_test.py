#!/usr/bin/python
import serial
import time
import sys

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0, parity=serial.PARITY_NONE, rtscts=0)

def tinyosd_set_char_xy(x, y, c):
    pos = y*35 + x
    tinyosd_set_char(pos, c)

def tinyosd_set_char(pos, c):
    tinyosd_send_buf(0x05, [(pos>>8)&0xFF]) #DMAH
    tinyosd_send_buf(0x06, [(pos   )&0xFF]) #DMAL
    tinyosd_send_buf(0x07, [ord(c) &0xFF]) #DMAI


def tinyosd_send_buf(address, buf):
    #[HEADER] LEN ADDR <n DATA> [CSUM]
    txbuf = []

    length = len(buf) & 0xFF

    txbuf.append(0x80) #header
    txbuf.append(length)
    txbuf.append(address & 0xFF)
    for i in range(length):
        txbuf.append(buf[i])
    
    #calc csum
    csum = 0
    for x in txbuf:
        csum ^= x

    txbuf.append(csum)

    #debug
    sys.stdout.write("TX: ")
    for x in txbuf:
        sys.stdout.write(" " + hex(x))
    sys.stdout.write("\n")
    sys.stdout.flush()

    ser.write(txbuf)

for (y) in range (13):
    for x in range(35):
        tinyosd_set_char_xy(x, y,'*')

for (y) in range (13):
    for x in range(35):
        tinyosd_set_char_xy(x, y,' ')


for (y) in range (13):
    for x in range(35):
        tinyosd_set_char_xy(x, y,'X')
        time.sleep(0.01)
