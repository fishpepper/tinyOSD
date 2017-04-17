#define LED_GPIO GPIOC
#define LED_PIN  GPIO8


#define DEBUG_GPIO      GPIOB
#define DEBUG_GPIO_AF   GPIO_AF0
#define DEBUG_UART      USART1
#define DEBUG_UART_RCC  RCC_USART1
#define DEBUG_TX_PIN    GPIO6

#define NVIC_PRIO_COMPARATOR 0*64
#define NVIC_PRIO_TIMER1     1*64
#define NVIC_PRIO_SYSTICK    2*64
#define NVIC_PRIO_DMA1       1*64
#define NVIC_PRIO_PENDSV     2*64

#define VIDEO_GPIO          GPIOA
#define VIDEO_INPUT_PIN     GPIO1
#define VIDEO_DAC_OUT_PIN   GPIO4
#define VIDEO_DAC_VCC       3.0

#define VIDEO_GPIO_BLACK          GPIOB

#define VIDEO_COMP_EXTI_SOURCE         GPIOA
#define VIDEO_COMP_EXTI_SOURCE_LINE    EXTI21
#define VIDEO_COMP_EXTI_IRQN           NVIC_ADC_COMP_IRQ

#define VIDEO_SPI_WHITE           SPI1
#define VIDEO_SPI_WHITE_RCC       RCC_SPI1

#define VIDEO_SPI_BLACK           SPI2
#define VIDEO_SPI_BLACK_RCC       RCC_SPI2

#define VIDEO_DMA_WHITE         DMA1
#define VIDEO_DMA_CHANNEL_WHITE DMA_CHANNEL3



