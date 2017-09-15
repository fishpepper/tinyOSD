// datasheets STM32F301
// RM: http://www.st.com/resource/en/reference_manual/dm00094350.pdf
// DS: http://www.keil.com/dd/docs/datashts/st/stm32f3xx/dm00093332.pdf

#define BOOTLOADER_ALLOW_READ 1

#define CPU_CLOCK 48000000

// timeout waiting for bootloader data
#define BOOTLOADER_TIMEOUT_MS        3000
#define BOOTLOADER_MAX_BAD_COMMANDS  32

#define DEVICE_FLASH_START           0x08000000
#define DEVICE_FLASH_SIZE            (64 * 1024)
#define DEVICE_PAGE_SIZE             ( 2 * 1024)

#define BOOTLOADER_SIZE              (5*DEVICE_PAGE_SIZE)
#define BOOTLOADER_MAGIC_PAGE        (DEVICE_FLASH_START + BOOTLOADER_SIZE)
#define BOOTLOADER_APP_START         (BOOTLOADER_MAGIC_PAGE + DEVICE_PAGE_SIZE)
#define BOOTLOADER_APP_END           (DEVICE_FLASH_START + DEVICE_FLASH_SIZE)

#define BOOTLOADER_SIZE_PAGES        (BOOTLOADER_SIZE / DEVICE_PAGE_SIZE)
#define BOOTLOADER_FLASH_SIZE_PAGES  (DEVICE_FLASH_SIZE / DEVICE_PAGE_SIZE)

// magic page:
// [CRC       :  4 bytes]
// [SIGNATURE : 16 bytes]
// [UNIQUE ID : 12 bytes]
// [empty               ]
#define FLASH_CRC_OFFSET       (                   0)
#define FLASH_SIGNATURE_OFFSET (FLASH_CRC_OFFSET + 4)
#define FLASH_DEVICE_ID_OFFSET (FLASH_SIGNATURE_OFFSET + 16)

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

