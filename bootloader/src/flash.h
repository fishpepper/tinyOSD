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

#ifndef FLASH_H_
#define FLASH_H_
#include <stdint.h>

void flash_init(void);
void flash_read(uint16_t address, __xdata uint8_t *buf, uint16_t len);
uint8_t flash_write_data(uint16_t address, __xdata uint8_t *buf, uint16_t len);
uint8_t flash_erase_page(uint8_t page);
static void flash_trigger_write(void);

#if ((FLASH_SIZE) > 65535)
    #error "ERROR: maximum supported flash size is 64k!"
#endif  // ((DEVICE_FLASH_SIZE) > 65535)

#define PAGECOUNT_BOOTLOADER ((BOOTLOADER_SIZE) / (FLASH_PAGESIZE))
#define PAGECOUNT_FLASH      ((FLASH_SIZE) / (FLASH_PAGESIZE))


#endif // FLASH_H_
