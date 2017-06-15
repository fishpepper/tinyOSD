// datasheets STM32F301
// RM: http://www.st.com/resource/en/reference_manual/dm00094350.pdf
// DS: http://www.keil.com/dd/docs/datashts/st/stm32f3xx/dm00093332.pdf

#define CAMOSD_RUNCAM 0
#define CAMOSD_VM275  1
#define EVALBOARD     2


//#define BOARD CAMOSD_VM275
#define BOARD EVALBOARD

#if (BOARD == CAMOSD_VM275)
    // vm275
    #define LED_GPIO GPIOA
    #define LED_PIN  GPIO5
    #define VIDEO_BSYNC_VOLTAGE_MV 100
    #define VIDEO_DAC_VCC       1.8
#elif (BOARD == CAMOSD_RUNCAM)
    // camOSD runcam
    #define LED_GPIO GPIOA
    #define LED_PIN  GPIO5
    #define VIDEO_BSYNC_VOLTAGE_MV 210
    #define VIDEO_DAC_VCC       1.8
#else
    // eval board
#if 1
    //disabled
    #define LED_GPIO GPIOB
    #define LED_PIN  GPIO0
#else
    #define LED_GPIO GPIOE
    #define LED_PIN  GPIO14
#endif
    #define VIDEO_BSYNC_VOLTAGE_MV 100
    #define VIDEO_DAC_VCC       3.0
#endif


#define VIDEO_ADC ADC1
#define VIDEO_ADC_CHANNEL 10
#define VIDEO_ADC_GPIO GPIOA
#define VIDEO_ADC_PIN GPIO6

/*#define DEBUG_GPIO      GPIOA
#define DEBUG_GPIO_AF   GPIO_AF1
#define DEBUG_UART      USART2
#define DEBUG_UART_RCC  RCC_USART2
#define DEBUG_TX_PIN    GPIO14
#define DEBUG_UART_BAUDRATE 115200*/

#define DEBUG_GPIO      GPIOA
#define DEBUG_GPIO_AF   GPIO_AF7
#define DEBUG_UART      USART2
#define DEBUG_UART_RCC  RCC_USART2
#define DEBUG_TX_PIN    GPIO2
#define DEBUG_UART_BAUDRATE 115200

#define SERIAL_GPIO      GPIOA
#define SERIAL_GPIO_AF   GPIO_AF7
#define SERIAL_UART      USART2
#define SERIAL_UART_RCC  RCC_USART2
#define SERIAL_TX_PIN    GPIO2
#define SERIAL_RX_PIN    GPIO3
#define SERIAL_UART_BAUDRATE 115200

#define SERIAL_OVERRUN_DETECTION_DISABLED 1

#define NVIC_PRIO_COMPARATOR 0*64
#define NVIC_PRIO_TIMER1     1*64
#define NVIC_PRIO_SYSTICK    2*64
#define NVIC_PRIO_DMA1       1*64
#define NVIC_PRIO_PENDSV     2*64

#define VIDEO_GPIO          GPIOA
#define VIDEO_INPUT_PIN     GPIO7
#define VIDEO_DAC_OUT_PIN   GPIO4


#define VIDEO_BLACK_GPIO              GPIOB
#define VIDEO_BLACK_MOSI_PIN          GPIO5
#define VIDEO_BLACK_MOSI_AF           GPIO_AF6
#define VIDEO_BLACK_DMA_CH            DMA_CHANNEL3
#define VIDEO_BLACK_DMA_IRQ           NVIC_DMA1_CHANNEL3_IRQ
#define VIDEO_BLACK_DMA_IRQ_HANDLER   DMA1_CHANNEL3_IRQHandler
#define VIDEO_BLACK_TIMER_DMA_CH      DMA_CHANNEL2

#define VIDEO_WHITE_GPIO              GPIOA
#define VIDEO_WHITE_MOSI_PIN          GPIO11
#define VIDEO_WHITE_MOSI_AF           GPIO_AF5
#define VIDEO_WHITE_DMA_CH            DMA_CHANNEL5
#define VIDEO_WHITE_DMA_IRQ           NVIC_DMA1_CHANNEL5_IRQ
#define VIDEO_WHITE_DMA_IRQ_HANDLER   DMA1_CHANNEL5_IRQHandler
#define VIDEO_WHITE_TIMER_DMA_CH      DMA_CHANNEL4

#define VIDEO_COMP_EXTI_SOURCE         GPIOA
#define VIDEO_COMP_EXTI_SOURCE_LINE    EXTI21
#define VIDEO_COMP_EXTI_IRQN           NVIC_COMP123_IRQ
#define VIDEO_COMP                     COMP2

#define VIDEO_SPI_WHITE           SPI1
#define VIDEO_SPI_WHITE_RCC       RCC_SPI1

#define VIDEO_SPI_BLACK           SPI2
#define VIDEO_SPI_BLACK_RCC       RCC_SPI2

#define VIDEO_DMA_WHITE         DMA1
#define VIDEO_DMA_BLACK         DMA1
#define VIDEO_DMA_CHANNEL_WHITE DMA_CHANNEL3

#define VIDEO_MUX_

//#define DEBUG_GPIO      GPIOA
//#define DEBUG_GPIO_AF   GPIO_AF1
//#define DEBUG_UART      USART2
//#define DEBUG_UART_RCC  RCC_USART2
//#define DEBUG_TX_PIN    GPIO14
//#define DEBUG_UART_BAUDRATE 115200
//
//#define SERIAL_GPIO      GPIOA
//#define SERIAL_GPIO_AF   GPIO_AF1
//#define SERIAL_UART      USART
//#define SERIAL_UART_RCC  RCC_USART1
//#define SERIAL_TX_PIN    GPIO9
//#define SERIAL_RX_PIN    GPIO10
//#define SERIAL_UART_BAUDRATE 115200



