// datasheets STM32F301
// RM: http://www.st.com/resource/en/reference_manual/dm00094350.pdf
// DS: http://www.keil.com/dd/docs/datashts/st/stm32f3xx/dm00093332.pdf

#define BOOTLOADER_ALLOW_READ 1

#define CPU_CLOCK 48000000

// timeout waiting for bootloader data
#define BOOTLOADER_TIMEOUT_MS        3000
#define BOOTLOADER_MAX_BAD_COMMANDS  32

#define DEVICE_FLASH_SIZE            (64 * 1024)
#define DEVICE_PAGE_SIZE             ( 2 * 1024)

#define BOOTLOADER_SIZE              0x00005000
#define BOOLOADER_APP_START          (0x08000000 + BOOTLOADER_SIZE)
#define BOOLOADER_APP_END            (0x08000000 + DEVICE_FLASH_SIZE)

#define BOOLOADER_SIZE_PAGES         (BOOTLOADER_SIZE / DEVICE_PAGE_SIZE)
#define BOOLOADER_FLASH_SIZE_PAGES   (DEVICE_FLASH_SIZE / DEVICE_PAGE_SIZE)

// STM32f301
#define BOOTLOADER_VERSION     0x40
#define BOOTLOADER_DEVICE_ID   0x0439

#define SERIAL_OVERRUN_DETECTION_DISABLED 0

#ifndef LED_GPIO
    #define LED_GPIO GPIOA
    #define LED_PIN  GPIO14
#endif

#ifndef SERIAL_GPIO
    #define SERIAL_GPIO      GPIOA
    #define SERIAL_GPIO_AF   GPIO_AF7
    #define SERIAL_UART      USART2
    #define SERIAL_UART_RCC  RCC_USART2
    #define SERIAL_TX_PIN    GPIO2
    #define SERIAL_RX_PIN    GPIO3
    #define SERIAL_UART_BAUDRATE 115200
#endif

// misc
#define NVIC_PRIO_SYSTICK 0

/*
#define DEBUG_GPIO      GPIOA
#define DEBUG_GPIO_AF   GPIO_AF7
#define DEBUG_UART      USART2
#define DEBUG_UART_RCC  RCC_USART2
#define DEBUG_TX_PIN    GPIO2
#define DEBUG_UART_BAUDRATE 115200
*/
