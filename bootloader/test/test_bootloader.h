#pragma once
#include <stdint.h>
#include <stdio.h>

extern void flash_unlock(void);
extern void flash_lock(void);
extern void flash_program_half_word(uint32_t a, uint16_t buf);
extern void flash_erase_page(uint32_t a);
extern uint8_t serial_getc();
extern void serial_putc(uint8_t x);
extern void crc_reset();
extern uint32_t crc_calculate_block();
extern void led_on();

#define DEBUG_PRINTF(format, ...) printf(format, ##__VA_ARGS__)

#define GLOBAL_INT_DISABLE() {}
#define GLOBAL_INT_ENABLE()  {}
#define RELOCATE_VTABLE(__a) {}

#include "../src/config.h"

extern uint8_t FLASH[];

extern uint8_t *flash_ptr_u8(uint32_t a);
extern uint16_t *flash_ptr_u16(uint32_t a);
extern uint32_t *flash_ptr_u32(uint32_t a);

#define FLASH_U8_PTR(__a) flash_ptr_u8(__a)
#define FLASH_U16_PTR(__a) flash_ptr_u16(__a)
#define FLASH_U32_PTR(__a) flash_ptr_u32(__a)

