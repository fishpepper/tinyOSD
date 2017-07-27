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

#define TINYOSD_COMMAND_SET_STATUS        0x00
#define TINYOSD_COMMAND_FILL_SCREEN       0x01
#define TINYOSD_COMMAND_WRITE_STICKDATA   0x07
//
//
#define TINYOSD_COMMAND_WRITE_PAGE_0      0x10
#define TINYOSD_COMMAND_WRITE_PAGE_1      0x11
#define TINYOSD_COMMAND_WRITE_PAGE_2      0x12
#define TINYOSD_COMMAND_WRITE_PAGE_3      0x13

#endif  // SERIAL_H_
