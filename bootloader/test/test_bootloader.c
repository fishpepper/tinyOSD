#define BUILD_TEST 1

#include "test_bootloader.h"
#include "../src/bootloader.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

//  ./ihex2bin -a 0x8000000 -i ../priv/tinyOSD/bin/tinyOSD.aes.hex -o test.bin

void flash_unlock(void){}
void flash_lock(void){}
void flash_program_half_word(uint32_t a, uint16_t val){
	uint16_t *flash_ptr = flash_ptr_u16(a);

	//printf("FLASH[%X] = %04X\n", a, val);
	*flash_ptr = val;
}
void flash_erase_page(uint32_t a){}
uint8_t serial_getc(){return 0;}
void serial_putc(uint8_t x) {}
uint32_t crc;
void crc_reset(){ crc = 0xFFFFFFFFF;}

#define CRC32POLY 0x04C11DB7 /* CRC-32 Polynom */
uint32_t crc_32( uint32_t crc, uint32_t data){
  int i;

  uint32_t data2 = (data&0xFF)<<24 | ((data>>8)&0xFF)<<16  | ((data>>16)&0xFF)<<8 | ((data>>24)&0xFF)<<0;

  crc = crc ^ data2;

  for(i=0; i<32; i++)
    if (crc & 0x80000000)
      crc = (crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
    else
      crc = (crc << 1);

  return(crc);
}

uint32_t crc_calculate_block(uint32_t *p, uint32_t len) { 

/*uint8_t *u = (uint8_t *)p;
for(int i=0; i<len*4; i++){
printf("%02X", *u++);
}*/
	while(len){
		len--;
		crc = crc_32(crc, *p++);
	}
	return crc;

}
void led_on(){}

uint8_t FLASH[DEVICE_FLASH_SIZE*2];
uint8_t DEVICEID[] = {
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC
};


uint8_t *flash_ptr_u8(uint32_t a){
	//printf("fetch flash ptr 8 for 0x%X\n", a);
        a = a - DEVICE_FLASH_START;
        if (a == 0x17FF7A10) {
		// device id
		return (uint8_t*)DEVICEID;		
	}
        if (a >= DEVICE_FLASH_SIZE){
                printf("ERROR: access out of bounds (size = %X, accessed element = %X\n",DEVICE_FLASH_SIZE,a);
                exit(1);
        }

        return (uint8_t*)&FLASH[a];
}
uint16_t *flash_ptr_u16(uint32_t a){
        //printf("fetch flash ptr 16 for 0x%X\n", a);
        if ((a & 1) != 0){
                printf("ERROR: alignment mismatch, requested u16 access to 0x%X\n",a);
                exit(1);
        }
        a = a - DEVICE_FLASH_START;
        if (a >= DEVICE_FLASH_SIZE){
                printf("ERROR: access out of bounds (size = %X, accessed element = %X\n",DEVICE_FLASH_SIZE,a);
                exit(1);
        }

        return (uint16_t *)&FLASH[a];
}
uint32_t *flash_ptr_u32(uint32_t a){
        //printf("fetch flash ptr 32 for 0x%X\n", a);
        if ((a & 3) != 0){
                printf("ERROR: alignment mismatch, requested u32 access to 0x%X\n",a);
                exit(1);
        }
        a = a - DEVICE_FLASH_START;
        if (a >= DEVICE_FLASH_SIZE){
                printf("ERROR: access out of bounds (size = %X, accessed element = %X\n",DEVICE_FLASH_SIZE,a);
                exit(1);
        }

        return (uint32_t *)&FLASH[a];
}

// prints string as hex
static void phex(uint8_t* str, uint8_t len){
    unsigned char i;
    for(i = 0; i < len; ++i) {
        printf("%.2x", str[i]);
        if (i%16 == 15) printf("\n");
    }
    printf("\n");
}

static void printflash(){
    int i;
    for(i = 0; i < 20*32; ++i) {
        printf("%.2x", FLASH[i]);
        if (i%32 == 31) printf("\n");
    }
    printf("...\n");
    for(i = BOOTLOADER_SIZE-20*32; i < BOOTLOADER_SIZE; ++i) {
        printf("%.2x", FLASH[i]);
        if (i%32 == 31) printf("\n");
    }
    printf("[MAGIC PAGE]\n");
    for(i = BOOTLOADER_SIZE; i < BOOTLOADER_SIZE+20*32; ++i) {
        printf("%.2x", FLASH[i]);
        if (i%32 == 31) printf("\n");
    }
    printf("...\n");
    for(i = BOOTLOADER_SIZE + DEVICE_PAGE_SIZE - 20*32; i < BOOTLOADER_SIZE + DEVICE_PAGE_SIZE; ++i) {
        printf("%.2x", FLASH[i]);
        if (i%32 == 31) printf("\n");
    }
    printf("[APP START]\n");
    for(i = BOOTLOADER_SIZE + DEVICE_PAGE_SIZE; i < BOOTLOADER_SIZE + DEVICE_PAGE_SIZE + 20*32; ++i) {
        printf("%.2x", FLASH[i]);
        if (i%32 == 31) printf("\n");
    }
    printf("...\n");
    uint32_t end = BOOTLOADER_APP_END - DEVICE_FLASH_START;
    for(i = end-20*32; i < end; ++i) {
        printf("%.2x", FLASH[i]);
        if (i%32 == 31) printf("\n");
    }

    printf("\n");
}



int main(void){
	// init flash content:
        FILE * file;
        file = fopen( "test.bin" , "r");
	for(int i=0; i<DEVICE_FLASH_SIZE; i++){
		if (!file) {
			printf("ERROR reading file %d\n",i);
		}

		FLASH[i] = getc(file);
	}

        printflash();

	bootloader_init();
	aes_init();

	bootloader_jump_to_app();
        printflash();

	bootloader_jump_to_app();
}
