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

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>

void serial_init(void);
void serial_process(void);

#define CRC8_FROM_HEADER (0x89)
#define PROTOCOL_FRAME_MAX_LEN 64

#define PROTOCOL_HEADER                   0x80

#define PROTOCOL_CMD_SET_REGISTER         0x0
#define PROTOCOL_CMD_FILL_REGION          0x2
#define PROTOCOL_CMD_WRITE                0x1
// 0x2
// 0x3
// 0x4
// 0x5
// 0x7
#define PROTOCOL_CMD_WRITE_BUFFER_H       0x8
#define PROTOCOL_CMD_WRITE_BUFFER_V       0x9
// 0xA
// 0xB
// 0xC
// 0xD
// 0xE
#define PROTOCOL_CMD_SPECIAL              0xF

#define PROTOCOL_CMD_SPECIAL_SUBCMD_STICKSTATUS  0x00


#define PROTOCOL_DEVICE_OSD               0x0
#define PROTOCOL_DEVICE_VTX               0x1
#define PROTOCOL_DEVICE_CAM               0x2
#endif  // SERIAL_H_
