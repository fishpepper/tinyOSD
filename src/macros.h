#ifndef MACROS_H__
#define MACROS_H__

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define GPIO_RCC(GPIO)  (((0x14) << 5) + (17 + ((GPIO - PERIPH_BASE_AHB2)/0x0400)))

#define DEFINE_TO_STR(x) #x
#define DEFINE_TO_STR_VAL(x) DEFINE_TO_STR(x)

#define min(a,b) \
  __extension__ ({ __typeof__ (a) _mina = (a); \
  __typeof__ (b) _minb = (b); \
  _mina < _minb ? _mina : _minb; })
#define max(a,b) \
  __extension__ ({ __typeof__ (a) _maxa = (a); \
  __typeof__ (b) _maxb = (b); \
  _maxa > _maxb ? _maxa : _maxb; })



#ifdef UNUSED
#elif defined(__GNUC__)
# define UNUSED(x) UNUSED_ ## x __attribute__((unused))
#elif defined(__LCLINT__)
# define UNUSED(x) /*@unused@*/ x
#else
# define UNUSED(x) x
#endif

// relocate function to run from sram (no waitstates!)
#define RUN_FROM_RAM __attribute__ ((long_call, section (".data")))


/*#define CRC_SET_INIT(__crc) { \
    CRC_INIT = (__crc); \
    CRC_CR = 0; \
    CRC_CR |= CRC_CR_RESET; \

}
#define CRC_UPDATE(__crc, __val) { CRC_DR = __val; __crc = CRC_DR;}*/

// ***************************************************
// using those macros is a big speedup (no lib calls!)
// ***************************************************
#define DMA_SET_NUMBER_OF_DATA(_dma, _ch, _val) { DMA_CNDTR(_dma, _ch) = (_val); }
#define DMA_SET_MEMORY_ADDRES_NOCHECK(_dma, _ch, _address) { DMA_CMAR(_dma, _ch) = (uint32_t) (_address); }

//#define DMA_CLEAR_INTERRUPT_FLAGS(_dma, _ch, _flags) { DMA_IFCR(_dma) = ((_flags) << DMA_FLAG_OFFSET(_ch)); }
#define DMA_CLEAR_INTERRUPT_FLAGS_MULTI(_dma, _flags) { DMA_IFCR(_dma) = (_flags); }
#define DMA_DISABLE_CHANNEL(_dma, _ch) { DMA_CCR(_dma, _ch) &= ~DMA_CCR_EN; }
#define DMA_ENABLE_CHANNEL(_dma, _ch) { DMA_CCR(_dma, _ch) |= DMA_CCR_EN; }

#define TIMER_CLEAR_FLAG(_tim, _flags) { TIM_SR(_tim) = ~(_flags); }
#define TIMER_DISABLE_IRQ(_tim, _irqs) { TIM_DIER(_tim) &= ~(_irqs); }
#define TIMER_ENABLE_IRQ(_tim, _irqs) { TIM_DIER(_tim) |= (_irqs); }
#define TIMER_GET_FLAG(_tim, _flag) (TIM_SR(_tim) & (_flag))
#define TIMER_SET_DMA_ON_COMPARE_EVENT(_tim) {  TIM_CR2(_tim) &= ~TIM_CR2_CCDS; }
#define TIMER_CLEAR_DMA_ON_COMPARE_EVENT(_tim) {  TIM_CR2(_tim) |= TIM_CR2_CCDS; }

#define GPIO_SET(__port, __gpios)    { GPIO_BSRR(__port) = __gpios; }
#define GPIO_CLEAR(__port, __gpios)  { GPIO_BSRR(__port) = (__gpios)<<16; }
#define GPIO_TOGGLE(__port, __gpios) { uint32_t p = GPIO_ODR(__port); \
    GPIO_BSRR(__port) = ((p & (__gpios)) << 16) | (~p & (__gpios)); }



// ***************************************************
// manual loop unrolling
// ***************************************************
#define _UNROLL_LOOP_1(__cmd)  {__cmd;}
#define _UNROLL_LOOP_2(__cmd)  {_UNROLL_LOOP_1(__cmd); _UNROLL_LOOP_1(__cmd); }
#define _UNROLL_LOOP_4(__cmd)  {_UNROLL_LOOP_2(__cmd); _UNROLL_LOOP_2(__cmd); }
#define _UNROLL_LOOP_8(__cmd)  {_UNROLL_LOOP_4(__cmd); _UNROLL_LOOP_4(__cmd); }
#define _UNROLL_LOOP_16(__cmd) {_UNROLL_LOOP_8(__cmd); _UNROLL_LOOP_8(__cmd); }
#define _UNROLL_LOOP_32(__cmd) {_UNROLL_LOOP_16(__cmd); _UNROLL_LOOP_16(__cmd); }
#define _UNROLL_LOOP_64(__cmd) {_UNROLL_LOOP_32(__cmd); _UNROLL_LOOP_32(__cmd); }

#define UNROLL_LOOP(__n, __cmd) \
    { \
        if(((__n)&1)!=0){_UNROLL_LOOP_1(__cmd);}\
        if(((__n)&2)!=0){_UNROLL_LOOP_2(__cmd);}\
        if(((__n)&4)!=0){_UNROLL_LOOP_4(__cmd);}\
        if(((__n)&8)!=0){_UNROLL_LOOP_8(__cmd);}\
        if(((__n)&16)!=0){_UNROLL_LOOP_16(__cmd);}\
        if(((__n)&32)!=0){_UNROLL_LOOP_32(__cmd);}\
        if(((__n)&64)!=0){_UNROLL_LOOP_64(__cmd);}\
    }

#endif  // MACROS_H__
