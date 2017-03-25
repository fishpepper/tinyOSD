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

#include "composite_video.h"
#include "config.h"
#include "macros.h"
#include "debug.h"
#include "delay.h"
#include "led.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/comparator.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/syscfg.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencmsis/core_cm3.h>

static void composite_video_init_rcc(void);
static void composite_video_init_gpio(void);
static void composite_video_init_comparator(void);
static void composite_video_init_comparator_interrupt(void);
static void composite_video_init_dac(void);
static void composite_video_set_dac_value_mv(uint16_t target);
static void composite_video_set_dac_value_raw(uint16_t target);

void composite_video_init(void) {
    uint16_t tmp = 0;

    composite_video_init_rcc();
    composite_video_init_gpio();

    composite_video_init_comparator();
    composite_video_init_comparator_interrupt();

    composite_video_init_dac();


    composite_video_set_dac_value_mv(100);
    while (1) {
        tmp += 1;
        if (tmp >= 300) tmp = 0;
        //composite_video_set_dac_value_mv(tmp);
        delay_ms(1);
    /*if (EXTI_EMR){
            debug("EXIT: 0x");
            debug_put_hex32(EXTI_EMR);
            debug_put_newline();
            EXTI_EMR = 0;
    }*/
            //
    }
}

static void composite_video_init_rcc(void) {
    // DAC clock
    rcc_periph_clock_enable(RCC_DAC);

    // for EXTI
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);

    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(COMPOSITE_VIDEO_GPIO));
}

static void composite_video_init_gpio(void) {
    // set video input pin as input
    gpio_mode_setup(COMPOSITE_VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, COMPOSITE_VIDEO_INPUT_PIN);

    // set dac to output
    gpio_mode_setup(COMPOSITE_VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, COMPOSITE_VIDEO_DAC_OUT_PIN);
    gpio_set_output_options(COMPOSITE_VIDEO_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, COMPOSITE_VIDEO_DAC_OUT_PIN);
}

static void composite_video_init_comparator(void) {
    debug("cvideo: comparator init\n");

    // start disabled
    comp_disable(COMP1);

    // set comparator inputs
    // inp = PA1
    // inm = DAC_OUT_1 (PA4) -> INM4
    comp_select_input(COMP1, COMP_CSR_INSEL_INM4);

    // no output
    comp_select_output(COMP1, COMP_CSR_OUTSEL_NONE);

    // hysteresis
    comp_select_hyst(COMP1, COMP_CSR_HYST_NO);

    // speed --> FAST!
    comp_select_speed(COMP1, COMP_CSR_SPEED_HIGH);

    // enable
    comp_enable(COMP1);
}

static void composite_video_init_comparator_interrupt(void) {
    debug("cvideo: comparator init ISR\n");

    // set up exti source
    exti_set_trigger(COMPOSITE_VIDEO_COMP_EXTI_SOURCE_LINE, EXTI_TRIGGER_FALLING);
    exti_enable_request(COMPOSITE_VIDEO_COMP_EXTI_SOURCE_LINE);

    // enable irq
    nvic_enable_irq(COMPOSITE_VIDEO_COMP_EXTI_IRQN);
    nvic_set_priority(COMPOSITE_VIDEO_COMP_EXTI_IRQN, NVIC_PRIO_COMPARATOR);
}

void ADC_COMP_IRQHandler(void) {
    if (exti_get_flag_status(COMPOSITE_VIDEO_COMP_EXTI_SOURCE_LINE) != 0) {
        exti_reset_request(COMPOSITE_VIDEO_COMP_EXTI_SOURCE_LINE);
        led_on();
        delay_us(10);
        led_off();
    }
}

static void composite_video_init_dac(void) {
    // start with disabled dac
    dac_disable(CHANNEL_1);
    dac_disable_waveform_generation(CHANNEL_1);

    // set default value and enable output
    composite_video_set_dac_value_mv(0);
    dac_enable(CHANNEL_1);

    // software update trigher
    dac_set_trigger_source(DAC_CR_TSEL1_SW);
}


// set dac to a given voltage level
static void composite_video_set_dac_value_mv(uint16_t target) {
    uint32_t tmp = target;
    debug("cvideo: dac set ");
    debug_put_fixed1p3(target);
    debug_put_newline();

    tmp = (tmp * 0x0FFF) / (COMPOSITE_VIDEO_DAC_VCC * 1000);
    composite_video_set_dac_value_raw(tmp);
}

static void composite_video_set_dac_value_raw(uint16_t target) {
    debug("cvideo: dac set raw 0x");
    debug_put_hex16(target);
    debug_put_newline();

    dac_load_data_buffer_single(target, RIGHT12, CHANNEL_1);
    dac_software_trigger(CHANNEL_1);
}

