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
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/syscfg.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencmsis/core_cm3.h>

static void video_init_rcc(void);
static void video_init_gpio(void);
static void video_init_timer(void);
static void video_init_comparator(void);
static void video_init_comparator_interrupt(void);
static void video_init_dac(void);
static void video_set_dac_value_mv(uint16_t target);
static void video_set_dac_value_raw(uint16_t target);

volatile uint32_t video_dbg;
volatile uint32_t video_line;
volatile uint32_t video_field;
volatile uint16_t video_sync_last_compare_value;

void video_init(void) {
    // uint16_t tmp = 0;

    video_init_rcc();
    video_init_gpio();

    video_init_timer();

    video_init_comparator();
    video_init_comparator_interrupt();

    video_init_dac();


    video_set_dac_value_mv(100);
    while (1) {
        // video_set_dac_value_mv(tmp);

        if (1) {  // video_dbg > 250) {
            debug_put_uint16(video_dbg);
            debug_put_newline();
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
}

static void video_init_rcc(void) {
    // DAC clock
    rcc_periph_clock_enable(RCC_DAC);

    // for EXTI
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);

    // timer1 clock
    rcc_periph_clock_enable(RCC_TIM1);

    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(VIDEO_GPIO));
}

static void video_init_gpio(void) {
    // set video input pin as input
    gpio_mode_setup(VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VIDEO_INPUT_PIN);

    // set dac to output
    gpio_mode_setup(VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VIDEO_DAC_OUT_PIN);
    gpio_set_output_options(VIDEO_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, VIDEO_DAC_OUT_PIN);
}

static void video_init_comparator(void) {
    debug("cvideo: comparator init\n");

    // start disabled
    comp_disable(COMP1);

    // set comparator inputs
    // inp = PA1
    // inm = DAC_OUT_1 (PA4) -> INM4
    comp_select_input(COMP1, COMP_CSR_INSEL_INM4);

    // no output
    comp_select_output(COMP1, COMP_CSR_OUTSEL_TIM1_IC1);

    // hysteresis
    comp_select_hyst(COMP1, COMP_CSR_HYST_MED);

    // speed --> FAST!
    comp_select_speed(COMP1, COMP_CSR_SPEED_HIGH);

    // enable
    comp_enable(COMP1);
}

static void video_init_timer(void) {
    uint16_t prescaler;

    // reset TIMx peripheral
    timer_reset(TIM1);

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

    // line frequency
    // NTSC (color) 15734 Hz = 63.56 us per line
    // PAL          15625 Hz = 64.00 us per line (54 us line content)
    // -> set up timer to overflow every 100us (= 0.1ms) = 10kHz
    prescaler = 1;
    debug("cvideo: tim1 presc ");
    debug_put_uint16(prescaler);
    timer_set_prescaler(TIM1, prescaler - 1);
    timer_set_repetition_counter(TIM1, 0);

    timer_enable_preload(TIM1);
    timer_continuous_mode(TIM1);
    timer_set_period(TIM1, 0xFFFF);

    // start timer
    timer_enable_counter(TIM1);
}

static void video_init_comparator_interrupt(void) {
    debug("cvideo: comparator init ISR\n");

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
    uint16_t pulse_len = video_sync_last_compare_value - current_compare_value;

    // we trigger on both edges
    // check if this was rising or falling edge:
    if (!(COMP_CSR(COMP1) & (1<<14))) {
        // falling edge -> this was measuring the field length
        if ((pulse_len > VIDEO_SYNC_VSYNC_MIN) && (pulse_len < VIDEO_SYNC_VSYNC_MAX)) {
            // this was the last half line -> hsync!
            video_field = VIDEO_FIRST_FIELD;
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

            // prepare for the next field
            video_field = 1 - VIDEO_FIRST_FIELD;
        } else {
            // this is a broad sync
            // prepare video transmission
        }
    }


    led_toggle();
}

static void video_init_dac(void) {
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
    debug("cvideo: dac set ");
    debug_put_fixed1p3(target);
    debug_put_newline();

    tmp = (tmp * 0x0FFF) / (VIDEO_DAC_VCC * 1000);
    video_set_dac_value_raw(tmp);
}

static void video_set_dac_value_raw(uint16_t target) {
    debug("cvideo: dac set raw 0x");
    debug_put_hex16(target);
    debug_put_newline();

    dac_load_data_buffer_single(target, RIGHT12, CHANNEL_1);
    dac_software_trigger(CHANNEL_1);
}

