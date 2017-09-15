#!/usr/bin/python
#
# This file is part of efm8load. efm8load is free software: you can
# redistribute it and/or modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation, version 3.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 51
# Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Copyright 2016 fishpepper.de
#
from __future__ import print_function
import serial
import argparse
import sys
import operator
import crcmod
from crcmod.predefined import *
from intelhex import IntelHex
from Crypto import Random
from Crypto.Cipher import AES

# example:
# python encrypt_hex.py  -s 64 -i 000102030405060708090A0B0C0D0E0F -k 603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4 ../../bin/tinyOSD.hex

# if you are missing the intelhex package, you can install it by 
# pip install intelhex --user
# the same goes for the crcmod
# pip install crcmod --user

class STM32Encrypt:
    """Encrypt a intel hex file using AES256"""

    def __init__(self, filename, flash_size, debug = False):
        self.debug           = debug
        self.filename        = filename
        self.flash_size      = int(flash_size) * 1024
        self.page_size       = 2048
        self.flash_start     = 0x8000000
        self.app_start       = self.flash_start + 4*self.page_size
        self.app_end         = self.flash_start + self.flash_size 
        # crc module
        self.crcmod          = mkPredefinedCrcFun("crc-32-mpeg") #crc-32") #0x104c11db7, initCrc=0, xorOut=0xFFFFFFFF)
        # hex parser
        self.ih = IntelHex()
        # open file

        self.open_file()

    def open_file(self):
        print("> reading hex file '%s'" % (self.filename))
        
        # read hex file
        self.ih.loadhex(self.filename)

        # verify that the reserved flash reigion is empty:
        address = self.app_end; 
        while(address < self.app_start + self.flash_size):
            if (self.ih[address] != 0xFF):
                print("> ERROR: reserved region at flash end is not empty! FLASH[0x%X] = 0x%X (expected 0xFF)\n" % (address, self.ih[address]))
                sys.exit(1)
            address += 1


        # calc checksum over datablob, start at app start
        self.data = ""
        address = self.app_start
        print("> input data    : 0x%08X - 0x%08X" % (self.app_start, self.app_end))
        while(address < self.app_end):
             c = chr(self.ih[address])
             self.data = self.data + c
             address = address + 1

        print("> got data      : " + 
            " ".join("{:02x}".format(ord(c)) for c in self.data[0:16])+ 
            " ... " + 
            " ".join("{:02x}".format(ord(c)) for c in self.data[-32:])) 

        crc_data = self.data[:-32]
        print("> calc crc over %d bytes" % (len(crc_data)))

        crc = self.crcmod(crc_data)
        self.data = self.data[:-4] + chr((crc>>0)&0xFF) + chr((crc>>8)&0xFF) + chr((crc>>16)&0xFF) + chr((crc>> 24)&0xFF)
 
        print("> added crc     : " +
            " ".join("{:02x}".format(ord(c)) for c in self.data[0:16])+
            " ... " +
            " ".join("{:02x}".format(ord(c)) for c in self.data[-32:])+
            " [%d bytes, CRC32 = 0x%08X]" % (len(self.data), crc))


    def encrypt(self, key, iv):
        # parse into array:
        self.key = key.decode("hex")
        self.iv  = iv.decode("hex")

        # show debug info
        print("> encoding...")
        print("> using key     : " + " ".join("{:02x}".format(ord(c)) for c in self.key))
        print("> using iv      : " + " ".join("{:02x}".format(ord(c)) for c in self.iv))
        # make sure that the key has the correct lenght:
        if len(self.key) != 32:
            print("> ERROR: key length mismatch, input key has to be 32 bytes as hex string!\n")
            sys.exit(1)
        if len(self.iv) != 16:
            print("> ERROR: iv length mismatch, input iv has to be 16 bytes as hex string!\n")
            sys.exit(1)

        # calculate AES256 CBC
        cipher = AES.new(self.key, AES.MODE_CBC, self.iv)
        encrypted = cipher.encrypt(self.data)
 
        print("> got encyrpted : " +
             " ".join("{:02x}".format(ord(c)) for c in encrypted[:16]) + 
             " ... " +
             " ".join("{:02x}".format(ord(c)) for c in encrypted[-32:]) +
             " (%d bytes) " % (len(encrypted)))
         
        # store data in ihex formatter
        address = self.app_start
        i = 0
        while(address < self.app_end):
            self.ih[address] = ord(encrypted[i])
            #print("ih[0x%X] = %X\n" % (address,self.ih[address]))
            address = address + 1
            i = i + 1

        output_fn = self.filename[:-3] + "aes.hex"
        print("> writing hex file '%s'" % (output_fn))

        # write hex
        self.ih.write_hex_file(output_fn)
        

if __name__ == "__main__":
    argp = argparse.ArgumentParser(description='encrypt_hex - a plain python encryption tool for intel hex files')

    argp.add_argument('-s', '--size', help='device flash size in KBytes, e.g. 64')
    argp.add_argument('-i', '--iv', help='initialization vector in hex format e.g. 0102....0F (16 bytes)')
    argp.add_argument('-k', '--key', help='encryption key in hex format e.g. AFFE1287817....BE (64 bytes)')
    argp.add_argument('filename', help='input filename')
    args = argp.parse_args()

    print("###########################################")
    print("# encrypt_hex.py - (c) 2017 fishpepper.de #")
    print("###########################################")
    print("")

    ecrypt = STM32Encrypt(args.filename, args.size, debug=False)

    if (args.filename) and (args.key) and (args.iv):
        ecrypt.encrypt(args.key, args.iv)
    else:
        argp.print_help()
        sys.exit(1)

    print 
    sys.exit(0)
