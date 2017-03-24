/* linker script for STM32F051R8T6, 64K flash, 8K RAM) */

/* define memory regions */
MEMORY { 
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 64K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

/* Include the common ld script */
INCLUDE libopencm3_stm32f0.ld
