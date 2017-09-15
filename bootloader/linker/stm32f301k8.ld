/* linker script for STM32F301K8, 64K flash, 16K RAM) */

/* define memory regions */
MEMORY { 
	rom (rx) : ORIGIN = 0x08000000, LENGTH = 5*2K
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}

/* Include the common ld script */
INCLUDE libopencm3_stm32f3.ld
