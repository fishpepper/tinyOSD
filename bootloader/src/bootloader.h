/*
    Copyright 2016 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    author: fishpepper <AT> gmail.com
*/

#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_
#include <stdint.h>

#define BOOTLOADER_COMMAND_INIT           0x7F
#define BOOTLOADER_COMMAND_GET            0x00
#define BOOTLOADER_COMMAND_GET_VERSION    0x01
#define BOOTLOADER_COMMAND_GET_ID         0x02
#define BOOTLOADER_COMMAND_READ_MEMORY    0x11
#define BOOTLOADER_COMMAND_GO             0x21
#define BOOTLOADER_COMMAND_WRITE_MEMORY   0x31
#define BOOTLOADER_COMMAND_ERASE          0x43

#define BOOTLOADER_RESPONSE_ACK           0x79
#define BOOTLOADER_RESPONSE_NACK          0x1F

void bootloader_init(void);
void bootloader_main(void);
void bootloader_jump_to_app(void);

#endif  // BOOTLOADER_H_
