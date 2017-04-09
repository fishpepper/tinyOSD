/*
    Copyright 2016 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/ or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.

    author: fishpepper <AT> gmail.com
*/

#include "video.h"
#include "config.h"
#include "macros.h"
#include "debug.h"
#include "delay.h"
#include "clocksource.h"
#include "led.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/comparator.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/syscfg.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencmsis/core_cm3.h>

static void video_init_rcc(void);
static void video_init_gpio(void);
static void video_init_timer(void);
static void video_init_comparator(void);
static void video_init_comparator_interrupt(void);
static void video_init_dac(void);
static void video_init_spi(void);
static void video_init_spi_dma(void);
static void video_init_pendsv(void);

static void video_dma_prepare(uint8_t page);
static void video_dma_trigger(void);

static void video_set_dac_value_mv(uint16_t target);
static void video_set_dac_value_raw(uint16_t target);

volatile uint8_t video_buffer[2][100];

volatile uint32_t video_dbg;
volatile uint32_t video_line;
volatile uint32_t video_field;
volatile uint16_t video_sync_last_compare_value;




// void TIM1_CC_IRQHandler(void) {
void TIM1_CC_IRQHandler(void) {
    if (timer_get_flag(TIM1, TIM_SR_CC2IF)) {
        timer_clear_flag(TIM1, TIM_SR_CC2IF);
        //timer_disable_irq(TIM1, TIM_DIER_CC2IE);
        //while(1) led_toggle();
       // led_toggle();
        //debug("CC MATCH\n");
        led_off();
    }
}

void video_init(void) {
    debug_function_call();

    // uint16_t tmp = 0;

    video_init_rcc();
    video_init_gpio();

    video_init_timer();

    video_init_comparator();
    video_init_comparator_interrupt();
    //video_init_pendsv();

    video_init_dac();

    video_init_spi();
    video_init_spi_dma();

    video_set_dac_value_mv(100);

    //timer_set_dma_on_compare_event(TIM1);

    led_off();
video_dma_prepare(0);
while(1){

    debug("T1CNT = "); debug_put_hex32(TIM1_CNT); debug(" ");
    debug("T2CNT = "); debug_put_hex32(TIM2_CNT); debug(" ");
    debug_put_newline();
    delay_us(100000);

};
    //timer_clear_flag(TIM1, TIM_SR_CC2IF);
    //timer_set_oc_value(TIM1, TIM_OC2, 0x2F00);
    //timer_enable_irq(TIM1, TIM_DIER_CC2DE | TIM_DIER_CC2IE);


    /*video_dma_prepare(0);

    debug("waiting for dma irq\n");
    //video_dma_trigger();

    //debug_put_hex32(SPI1_SR);
    //debug_put_newline();
    //SPI1_SR = 2;

    //debug_put_hex32(SPI1_SR);
    while(1){

        debug("T1CNT = "); debug_put_hex32(TIM1_CNT); debug(" ");
        debug("T2CNT = "); debug_put_hex32(TIM2_CNT); debug(" ");
        debug_put_newline();
        delay_us(100000);

    };

    while (1) {
        // video_set_dac_value_mv(tmp);

        if (1) {  // video_dbg > 250) {
            // debug_put_uint16(video_line);
            // debug_put_newline();
            // video_dbg = 0;
        }
    /*if (EXTI_EMR){
            debug("EXIT: 0x");
            debug_put_hex32(EXTI_EMR);
            debug_put_newline();
            EXTI_EMR = 0;
    }*/
            //

}

static void video_init_pendsv(void) {
    debug_function_call();

    nvic_set_priority(NVIC_PVD_IRQ, NVIC_PRIO_PENDSV);
    nvic_enable_irq(NVIC_PVD_IRQ);
}

static void video_init_spi(void) {
    debug_function_call();

    // SPI NVIC
    // nvic_set_priority(NVIC_SPI2_IRQ, 3);
    // nvic_enable_irq(NVIC_SPI2_IRQ);

    // clean start
    spi_reset(VIDEO_SPI_WHITE);

    // set up spi
    // - master mode
    // - baud prescaler = apb_clk/2 = 24/2 = 12MHz!
    // - CPOL low
    // - CPHA 1
    // - 8 bit crc (?)
    // - MSB first
    spi_init_master(VIDEO_SPI_WHITE,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_2,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_CRCL_8BIT,
                    SPI_CR1_MSBFIRST);

    // set NSS to software
    // NOTE: setting NSS high is important! even when controling it on our
    //       own. otherwise spi will not send any data!
    spi_enable_software_slave_management(VIDEO_SPI_WHITE);
    spi_set_nss_high(VIDEO_SPI_WHITE);

    // Enable SPI periph
    spi_enable(VIDEO_SPI_WHITE);



    // set fifo to quarter full(=1 byte)
    //spi_fifo_reception_threshold_8bit(VIDEO_SPI_WHITE);
}

void DMA1_CHANNEL2_3_IRQHandler(void) {
    // disable TIM2 (sends pulses to DMA)
    timer_disable_counter(TIM2);

    // clear flag
    dma_clear_interrupt_flags(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_TCIF);

    // disable OC match dma trigger
    //timer_disable_irq(TIM2, TIM_DIER_CC2DE);

    // disable dma
    dma_disable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);

    // clear OC2REF, next trigger will re initiate transfer
    timer_set_oc_mode(TIM1, TIM_OC2, TIM_CCMR1_OC2M_FORCE_LOW);

    // toggle led
    led_off();

    //debug("DMA IRQ EXIT\n");
}

static void video_init_spi_dma(void) {
    debug_function_call();

    // start disabled
    dma_disable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);

    // DMA NVIC
    nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, NVIC_PRIO_DMA1);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ); //

    // start with clean init
    dma_channel_reset(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);

    // source and destination 1 Byte
    dma_set_memory_size(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_CCR_MSIZE_8BIT);
    dma_set_peripheral_size(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_CCR_PSIZE_8BIT);

    // automatic memory destination increment enable.
    dma_enable_memory_increment_mode(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);
    // source address increment disable
    dma_disable_peripheral_increment_mode(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);
    // Location assigned to peripheral register will be target
    dma_set_read_from_memory(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);
    // source and destination start addresses
    dma_set_peripheral_address(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, (uint32_t)&(SPI_DR(VIDEO_SPI_WHITE)));
    // target address will be set later
    //dma_set_memory_address(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, 0);
    // chunk of data to be transfered, will be set later
    dma_set_number_of_data(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, 1);
    // very high prio
    dma_set_priority(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_CCR_PL_VERY_HIGH);

    // enable tx complete int
    dma_enable_transfer_complete_interrupt(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);  // OK

    //debug_put_hex32(DMA_CCR(VIDEO_DMA_WHITE, 3));
}

void video_dma_prepare(uint8_t page) {
    // debug_function_call();

    // disable dma during config
    dma_disable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);

    video_buffer[0][0] = 0b00000001;
    video_buffer[0][1] = 0b00000111;
    video_buffer[0][2] = 0b00111111;
    video_buffer[0][3] = 0b01010101;


    // TX: transfer buffer to slave
    dma_set_memory_address(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, (uint32_t)video_buffer[page]);
    dma_set_number_of_data(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, 4);

    //timer_set_oc_mode(TIM1, TIM_OC2, TIM_CCMR1_OC2M_FORCE_LOW);


    // clear all dma if
    dma_clear_interrupt_flags(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_TCIF);
//    DMA1_IFCR = DMA1_ISR;
//debug_put_hex32(DMA1_ISR); debug_put_newline();

#if 1
    // HACK/WORKAROUND: seems like one DMA tx is pending
    // how do we clear it? no idea. Instead: redirect this first dma to
    // an unused peripheral
    dma_set_peripheral_address(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, (uint32_t)&(TIM7_CNT));
    // enable dma channel
    dma_enable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);
    // wait for some time
    delay_us(1);

    // disable dma during config
    dma_disable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);

    // back to spi
    dma_set_peripheral_address(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, (uint32_t)&(SPI_DR(VIDEO_SPI_WHITE)));
    // TX: transfer buffer to slave
    dma_set_memory_address(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, (uint32_t)video_buffer[page]);
    dma_set_number_of_data(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, 4);
#endif

    // enable dma channel
    dma_enable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);
    //while(1);
}

#if 0
// waking up from sleep mode is quite deterministic
// disable all interrupts except tim1 compare2 int
void pend_sv_handler(void) {
    ///led_toggle();
    //debug("sleeping\n");

    // disable all irqs EXCPET CC match here!
    //nvic_disable_irq(VIDEO_COMP_EXTI_IRQN);

    timer_enable_irq(TIM1, TIM_DIER_CC2IE);
    nvic_set_priority(NVIC_TIM1_CC_IRQ, NVIC_PRIO_TIMER1);
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    // Clear pendSV flag
    SCB_ICSR |= SCB_ICSR_PENDSVCLR;

    // do NOT deep sleep...
    SCB_SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    //SCB_SCR |= SCB_SCR_SEVEONPEND;
    //SCB_SCR |= SCB_SCR_SLEEPONEXIT;
    /*uint32_t s_ = 0;
    while(1){
        uint32_t s = TIM1_SR & (1<<2);
        if (s!= s_){
            debug_put_hex32(s);
            debug_put_newline();
        }
        s_ = s;
    }*/
    //debug_put_hex32(TIM1_SR & (1<<2));

    /*uint32_t i;
    for(i=0; i<10; i++) debug_put_hex32(TIM1_SR & (1<<2));*/

    // sleep now!
    led_off();
    __WFI();
    //__asm__("wfe");


    //why does cc int not trigger resume?

    led_on();

    //debug("AWAKE\n");
    //while(1) led_toggle();

    // re-enable all irqs
   // nvic_enable_irq(VIDEO_COMP_EXTI_IRQN);
}
#endif


void video_dma_trigger(void) {
    // debug_function_call();

    // trigger the SPI TX + RX dma
    spi_enable_tx_dma(VIDEO_SPI_WHITE);

    // wait for completion
    //while (!dma_get_interrupt_flag(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_TCIF)) {}

    // disable DMA
    //dma_disable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);
}

static void video_init_rcc(void) {
    debug_function_call();

    // DAC clock
    rcc_periph_clock_enable(RCC_DAC);

    // for EXTI
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);

    // timer1 clock
    rcc_periph_clock_enable(RCC_TIM1);

    // timer2 clock
    rcc_periph_clock_enable(RCC_TIM2);

    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(VIDEO_GPIO));

    // spi
    rcc_periph_clock_enable(VIDEO_SPI_WHITE_RCC);

    // enable DMA Peripheral Clock
    rcc_periph_clock_enable(RCC_DMA);
}

static void video_init_gpio(void) {
    debug_function_call();

    // set video input pin as input
    gpio_mode_setup(VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VIDEO_INPUT_PIN);

    // set dac to output
    gpio_mode_setup(VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VIDEO_DAC_OUT_PIN);
    gpio_set_output_options(VIDEO_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, VIDEO_DAC_OUT_PIN);

    // set spi to output
    // init sck (5, for dbg), MOSI (7)
    uint32_t spi_gpios = GPIO5 | GPIO7;
    // set mode
    gpio_mode_setup(VIDEO_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, spi_gpios);
    gpio_set_output_options(VIDEO_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, spi_gpios);
}

static void video_init_comparator(void) {
    debug_function_call();

    // start disabled
    comp_disable(COMP1);

    // set comparator inputs
    // inp = PA1
    // inm = DAC_OUT_1 (PA4) -> INM4
    comp_select_input(COMP1, COMP_CSR_INSEL_INM4);

    // IC1 output
    comp_select_output(COMP1, COMP_CSR_OUTSEL_TIM1_IC1);

    // hysteresis
    comp_select_hyst(COMP1, COMP_CSR_HYST_MED);

    // speed --> FAST!
    comp_select_speed(COMP1, COMP_CSR_SPEED_HIGH);

    // enable
    comp_enable(COMP1);
}

static void video_init_timer(void) {
    debug_function_call();

    uint16_t prescaler;

    timer_disable_counter(TIM1);

    // reset TIMx peripheral
    timer_reset(TIM1);

    timer_enable_irq(TIM1, TIM_DIER_CC2IE);

    // Set the timers global mode to:
    // - use no divider
    // - alignment edge
    // - count direction up
    timer_set_mode(TIM1,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);

    // input compare trigger
    timer_ic_set_input(TIM1, TIM_IC1, TIM_IC_IN_TI1);
    timer_ic_set_polarity(TIM1, TIM_IC1, TIM_IC_BOTH);
    timer_ic_set_prescaler(TIM1, TIM_IC1, TIM_IC_PSC_OFF);
    timer_ic_set_filter(TIM1, TIM_IC1, TIM_IC_OFF);
    timer_ic_enable(TIM1, TIM_IC1);

    // set CC2 as output to internals
    timer_ic_set_input(TIM1, TIM_IC2, TIM_CCMR1_CC2S_OUT);

    //OC2REF is set on compare match
    //timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM2);

    // set up oc2 interrupt
    nvic_set_priority(NVIC_TIM1_CC_IRQ, NVIC_PRIO_TIMER1);
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    // timer_set_dma_on_compare_event(TIM1);
    timer_disable_oc_preload(TIM1, TIM_OC2);

    // line frequency
    // NTSC (color) 15734 Hz = 63.56 us per line
    // PAL          15625 Hz = 64.00 us per line (54 us line content)
    // -> set up timer to overflow every 100us (= 0.1ms) = 10kHz
    prescaler = 1;
    // debug("cvideo: tim1 presc ");
    // debug_put_uint16(prescaler);
    timer_set_prescaler(TIM1, prescaler - 1);
    // timer_set_repetition_counter(TIM1, 0);

    // timer_enable_preload(TIM1);
    timer_continuous_mode(TIM1);
    timer_set_period(TIM1, 0xFFFF);

    // RM0091
    // p. 430 -> gate timer2 by compare match of timer1
    // the idea:
    // TIM1 CC2 match -> master mode -> set as output
    // TIM2 is disabled
    // TIM2 input trigger from timer 1 TS 000
    //            trigger mode SMS = 110
    // TIM2 period = 1 (= smaller than spi transfer length! -> continous triggering!)
    // TIM2 CC2 match = period
    // TIM1 CC2 match -> enables TIM2 -> TIM2 counts -> CC2 match triggers repeatedly!

#if 1
    // start disabled
    timer_disable_counter(TIM2);

    // reset TIMx peripheral
    timer_reset(TIM2);

    // Set the timer global mode to:
    // - use no divider, alignment edge, count direction up
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // prescaler
    timer_set_prescaler(TIM2, 0);
    // continuous mode
    timer_continuous_mode(TIM2);
    // period
    uint32_t period = 2;
    timer_set_period(TIM2, period);
    // set up oc2
    timer_set_oc_value(TIM2, TIM_OC2, period/2);
    //timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_FROZEN);
    //timer_disable_oc_preload(TIM2, TIM_OC2);
    // clear timer
    // TIM2_CNT = 0;

    // set TIM1 as master -> OC2 is sent as signal
    timer_set_master_mode(TIM1, TIM_CR2_MMS_COMPARE_OC2REF);
    TIM1_SMCR |= TIM_SMCR_MSM;

    // set TIM2 as slave
    // set ITR0 = connected to TIM1
    timer_slave_set_trigger(TIM2, TIM_SMCR_TS_ITR0);
    // set trigger mode to TriggerMode (start counting on trigger)
    timer_slave_set_mode(TIM2, TIM_SMCR_SMS_TM);


    // enable DMA trigger to ch2
    timer_enable_irq(TIM2, TIM_DIER_CC2DE);


    timer_clear_flag(TIM1, TIM_SR_CC2IF);
#endif
    //timer_enable_counter(TIM2);



    debug("TIM1 CCMR1 = 0x"); debug_put_hex32(TIM1_CCMR1); debug_put_newline();
    debug("TIM1 CCER  = 0x"); debug_put_hex32(TIM1_CCER); debug_put_newline();
    debug("TIM1 SMCR  = 0x"); debug_put_hex32(TIM1_SMCR); debug_put_newline();
    debug("TIM1 CR2   = 0x"); debug_put_hex32(TIM1_CR2); debug_put_newline();
    debug_put_newline();
    debug("TIM2 CCMR1 = 0x"); debug_put_hex32(TIM2_CCMR1); debug_put_newline();
    debug("TIM2 CCER  = 0x"); debug_put_hex32(TIM2_CCER); debug_put_newline();
    debug("TIM2 SMCR  = 0x"); debug_put_hex32(TIM2_SMCR); debug_put_newline();
    debug("TIM2 CR2   = 0x"); debug_put_hex32(TIM2_CR2); debug_put_newline();
//while(1);
    // start timer 1
    timer_enable_counter(TIM1);
}

static void video_init_comparator_interrupt(void) {
    debug_function_call();

    // set up exti source
    exti_set_trigger(VIDEO_COMP_EXTI_SOURCE_LINE, EXTI_TRIGGER_BOTH);
    exti_enable_request(VIDEO_COMP_EXTI_SOURCE_LINE);

    // enable irq
    nvic_enable_irq(VIDEO_COMP_EXTI_IRQN);
    nvic_set_priority(VIDEO_COMP_EXTI_IRQN, NVIC_PRIO_COMPARATOR);
}

void ADC_COMP_IRQHandler(void) {
    // well, this irq is only called on comp interrupts -> skip checking...
    // if (exti_get_flag_status(VIDEO_COMP_EXTI_SOURCE_LINE) != 0) {
    // clear flag
    // exti_reset_request(VIDEO_COMP_EXTI_SOURCE_LINE);
    EXTI_PR = VIDEO_COMP_EXTI_SOURCE_LINE;  // clear flag

    // calc duration
    uint16_t current_compare_value = TIM_CCR1(TIM1);
    uint16_t pulse_len = current_compare_value - video_sync_last_compare_value;
    video_dbg = pulse_len;

    // we trigger on both edges
    // check if this was rising or falling edge:
    if (!(COMP_CSR(COMP1) & (1<<14))) {
        // falling edge -> this was measuring the field length
        if ((pulse_len > VIDEO_SYNC_VSYNC_MIN) && (pulse_len < VIDEO_SYNC_VSYNC_MAX)) {
            // this was the last half line -> hsync!
            video_field = VIDEO_FIRST_FIELD;
            //led_on();
            // spi_send(VIDEO_SPI_WHITE, 0x7FF7);

            // timer_set_dma_on_update_event(TIM1);

            // timer_enable_oc_output(TIM1, TIM_OC2);
            // timer_disable_oc_clear(TIM1, TIM_OC2);
            // timer_set_oc_slow_mode(TIM1, TIM_OC2);
            // timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_TOGGLE);
            //
            // video_dma_trigger();
           // timer_disable_preload(TIM1);
        }
    } else  {
        // rising edge -> this was measuring the a sync part
        if (pulse_len < VIDEO_SYNC_SHORT_MAX) {
            // all short sync pulses are shortsyncs
            // new (half)frame -> init line counter
            video_line = video_field;


        } else if (pulse_len < VIDEO_SYNC_HSYNC_MAX) {
            // this is longer than a short sync and not a broad sync

            // increment video field
            video_line += 2;

          //  dma_disable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);
            // video_dma_trigger();

            led_on();
            timer_disable_counter(TIM2);
            TIM2_CNT = 0;
            timer_set_oc_mode(TIM1, TIM_OC2, TIM_CCMR1_OC2M_FORCE_LOW);

            //video_dma_trigger();
            // prepare line data transfer
            // set oc match to start of line data
            timer_disable_preload(TIM1);
            timer_disable_preload(TIM2);
            timer_set_oc_value(TIM1, TIM_OC2, current_compare_value + _US_TO_CLOCKS(45));

            // set mode to toggle on compare match (was low -> will get high)
            //timer_set_oc_mode(TIM1, TIM_OC2, TIM_CCMR1_OCM_PWM2); //TOGGLE);
            timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM2);

            led_off();
            video_dma_prepare(0);
            delay_us(5);
            led_on();

            //timer_enable_irq(TIM1, TIM_DIER_CC2IE);
            /*timer_clear_flag(TIM1, TIM_SR_CC2IF);
*/
            //
/*            timer_set_oc_value(TIM1, TIM_OC2, TIM1_CNT + 60); //current_compare_value+150);
delay_us(45000);
while (!dma_get_interrupt_flag(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_TCIF)) {}

// disable DMA
dma_disable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);
*/
            // prepare for the next field
            video_field = 1 - VIDEO_FIRST_FIELD;
        } else {
            // this is a broad sync
            // prepare video transmission
            // configure timer compare match output
            // video_dma_trigger();
        }
    }

    // store current value for period measurements
    video_sync_last_compare_value = current_compare_value;
}



static void video_init_dac(void) {
    debug_function_call();

    // start with disabled dac
    dac_disable(CHANNEL_1);
    dac_disable_waveform_generation(CHANNEL_1);

    // set default value and enable output
    video_set_dac_value_mv(0);
    dac_enable(CHANNEL_1);

    // software update trigher
    dac_set_trigger_source(DAC_CR_TSEL1_SW);
}


// set dac to a given voltage level
static void video_set_dac_value_mv(uint16_t target) {
    uint32_t tmp = target;
    debug("cvideo: set_dac_value_mv(");
    debug_put_fixed1p3(target);
    debug(")\n");

    tmp = (tmp * 0x0FFF) / (VIDEO_DAC_VCC * 1000);
    video_set_dac_value_raw(tmp);
}

static void video_set_dac_value_raw(uint16_t target) {
    debug("cvideo: set_dac_value_raw(0x");
    debug_put_hex16(target);
    debug(")\n");

    dac_load_data_buffer_single(target, RIGHT12, CHANNEL_1);
    dac_software_trigger(CHANNEL_1);
}

