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

#include "adc.h"
#include "debug.h"
#include "led.h"
#include "delay.h"
#include "config.h"
#include "macros.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>


// internal functions
static void adc_init_rcc(void);
static void adc_init_gpio(void);
static void adc_init_mode(void);

void adc_init(void) {
    debug_function_call();

    adc_init_rcc();
    adc_init_gpio();
    adc_init_mode();
}

static void adc_init_rcc(void) {
    debug_function_call();

    // enable adc clock
    rcc_periph_clock_enable(RCC_ADC);

    // enable adc gpio clock
    rcc_periph_clock_enable(GPIO_RCC(VIDEO_ADC_GPIO));

    // start with adc off
    adc_power_off(VIDEO_ADC);

    // ADC CLOCK = 48 / 4 = 12MHz
    adc_set_clk_source(VIDEO_ADC, ADC_CLKSOURCE_PCLK_DIV4);

    // run calibration
    adc_calibrate(VIDEO_ADC);
}

static void adc_init_gpio(void) {
    debug_function_call();

    // set up analog inputs
    gpio_mode_setup(VIDEO_ADC_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VIDEO_ADC_PIN);

    /*gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF2, GPIO9);*/
}

uint16_t adc_get_value(void) {
    uint16_t val = 10;
    if (ADC_ISR(VIDEO_ADC) & ADC_ISR_EOC){
        val = ADC_DR(VIDEO_ADC);
        //debug_put_hex16(val); debug_put_newline();
        return val;
    }
    //ADC_CR(VIDEO_ADC) |= ADC_CR_ADSTART;

    return val;
}

static void adc_init_mode(void) {
    debug_function_call();

    // set mode to continous
    //adc_set_continuous_conversion_mode(VIDEO_ADC);
    adc_disable_discontinuous_mode(VIDEO_ADC);
    adc_set_operation_mode (VIDEO_ADC, ADC_MODE_SEQUENTIAL);

    // ext trigger: TIM1_TRGO
    adc_enable_external_trigger_regular(VIDEO_ADC, ADC_CFGR1_EXTSEL_VAL(0), ADC_CFGR1_EXTEN_RISING_EDGE);

    // right 12-bit data alignment in ADC reg
    adc_set_right_aligned(VIDEO_ADC);
    adc_set_resolution(VIDEO_ADC, ADC_RESOLUTION_12BIT);

    // adc_enable_temperature_sensor();
    adc_disable_analog_watchdog(VIDEO_ADC);

    // configure channel
    uint8_t channels[] = { VIDEO_ADC_CHANNEL };

    // sample times for all channels
    adc_set_sample_time_on_all_channels(VIDEO_ADC, ADC_SMPTIME_041DOT5);

    adc_set_regular_sequence(VIDEO_ADC, sizeof(channels), channels);

    adc_power_on(VIDEO_ADC);

    // wait for ADC starting up
    int i;
    for (i = 0; i < 800000; i++) {
        asm("nop");
    }

    ADC_CR(VIDEO_ADC) |= ADC_CR_ADSTART;
}
