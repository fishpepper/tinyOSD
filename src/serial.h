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

//#define PROTOCOL_FRAME_MAX_LEN 64


#define OPENTCO_PROTOCOL_HEADER 0x80
#define OPENTCO_CRC8_FROM_HEADER (0x89)

#define OPENTCO_MAX_DATA_LENGTH       60
#define OPENTCO_MAX_FRAME_LENGTH     (OPENTCO_MAX_DATA_LENGTH + 4)

// 0x01..0x07 = valid device ids
#define OPENTCO_DEVICE_OSD                           0x00
#define OPENTCO_DEVICE_VTX                           0x01
#define OPENTCO_DEVICE_CAM                           0x02
//
#define OPENTCO_DEVICE_MAX                           0x07

// 0x08..0x0F = valid device response ids
#define OPENTCO_DEVICE_RESPONSE                      0x08
#define OPENTCO_DEVICE_OSD_RESPONSE                  (OPENTCO_DEVICE_RESPONSE | OPENTCO_DEVICE_OSD)
#define OPENTCO_DEVICE_VTX_RESPONSE                  (OPENTCO_DEVICE_RESPONSE | OPENTCO_DEVICE_VTX)
#define OPENTCO_DEVICE_CAM_RESPONSE                  (OPENTCO_DEVICE_RESPONSE | OPENTCO_DEVICE_CAM)


#define OPENTCO_OSD_COMMAND_REGISTER_ACCESS          0x00
#define OPENTCO_OSD_COMMAND_FILL_REGION              0x01
#define OPENTCO_OSD_COMMAND_WRITE                    0x02
#define OPENTCO_OSD_COMMAND_WRITE_BUFFER_H           0x08
#define OPENTCO_OSD_COMMAND_WRITE_BUFFER_V           0x09
#define OPENTCO_OSD_COMMAND_SPECIAL                  0x0F

#define OPENTCO_REGISTER_ACCESS_MODE_READ            0x80
#define OPENTCO_REGISTER_ACCESS_MODE_WRITE           0x00

#define OPENTCO_OSD_REGISTER_STATUS                  0x00  // R/W
//
#define OPENTCO_OSD_REGISTER_VIDEO_FORMAT            0x01  // R/W
#define OPENTCO_OSD_REGISTER_INVERT                  0x02  // R/W
#define OPENTCO_OSD_REGISTER_BRIGHTNESS_BLACK        0x03  // R/W
#define OPENTCO_OSD_REGISTER_BRIGHTNESS_WHITE        0x04  // R/W
#define OPENTCO_MAX_REGISTER                         0x0F

#define OPENTCO_OSD_COMMAND_SPECIAL_SUB_STICKSTATUS  0x00
#define OPENTCO_OSD_COMMAND_SPECIAL_SUB_SPECTRUM     0x01


#endif  // SERIAL_H_
