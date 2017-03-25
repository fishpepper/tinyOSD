#define LED_GPIO GPIOC
#define LED_PIN  GPIO8


#define DEBUG_GPIO      GPIOB
#define DEBUG_GPIO_AF   GPIO_AF0
#define DEBUG_UART      USART1
#define DEBUG_UART_RCC  RCC_USART1
#define DEBUG_TX_PIN    GPIO6

#define NVIC_PRIO_COMPARATOR 0*64
#define NVIC_PRIO_SYSTICK    1*64

#define COMPOSITE_VIDEO_GPIO          GPIOA
#define COMPOSITE_VIDEO_INPUT_PIN     GPIO1
#define COMPOSITE_VIDEO_DAC_OUT_PIN   GPIO4
#define COMPOSITE_VIDEO_DAC_VCC       3.0

#define COMPOSITE_VIDEO_COMP_EXTI_SOURCE         GPIOA
#define COMPOSITE_VIDEO_COMP_EXTI_SOURCE_LINE    EXTI21
#define COMPOSITE_VIDEO_COMP_EXTI_IRQN           NVIC_ADC_COMP_IRQ


