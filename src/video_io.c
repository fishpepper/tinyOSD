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

#include "video_io.h"
#include "config.h"
#include "macros.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dac.h>
#include "debug.h"

static void video_io_init_rcc(void);
static void video_io_init_gpio(void);
static void video_io_init_dac(void);
static void video_io_set_dac_value_raw(uint16_t taregt);

void video_io_init(void) {
    video_io_init_rcc();
    video_io_init_gpio();
    video_io_init_dac();
}


static void video_io_init_rcc(void) {
    debug_function_call();

    // DAC clock
    rcc_periph_clock_enable(RCC_DAC1);

    // for EXTI / COMP (syscfg and comp share clock enable)
    rcc_periph_clock_enable(RCC_SYSCFG);

    // timer1 clock
    rcc_periph_clock_enable(RCC_TIM1);

    // timer2 clock
    //rcc_periph_clock_enable(RCC_TIM2);

    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(VIDEO_GPIO));
    rcc_periph_clock_enable(GPIO_RCC(VIDEO_WHITE_GPIO));
    rcc_periph_clock_enable(GPIO_RCC(VIDEO_BLACK_GPIO));

    // spi
    rcc_periph_clock_enable(VIDEO_SPI_WHITE_RCC);
    rcc_periph_clock_enable(VIDEO_SPI_BLACK_RCC);

    // enable DMA Peripheral Clock
    rcc_periph_clock_enable(RCC_DMA1);
}

static void video_io_init_gpio(void) {
    debug_function_call();

    // set video input pin as input
    gpio_mode_setup(VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VIDEO_INPUT_PIN);

    // set dac to output
    gpio_mode_setup(VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VIDEO_DAC_OUT_PIN);
    gpio_set_output_options(VIDEO_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, VIDEO_DAC_OUT_PIN);

    // set spi mosi to output
    gpio_mode_setup(VIDEO_WHITE_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, VIDEO_WHITE_MOSI_PIN);
    //gpio_mode_setup(VIDEO_WHITE_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, VIDEO_WHITE_MOSI_PIN);
    gpio_set_af(VIDEO_WHITE_GPIO, VIDEO_WHITE_MOSI_AF, VIDEO_WHITE_MOSI_PIN);
    gpio_set_output_options(VIDEO_WHITE_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, VIDEO_WHITE_MOSI_PIN);
    gpio_clear(VIDEO_WHITE_GPIO, VIDEO_WHITE_MOSI_PIN);

    // set spi mosi to output
    gpio_mode_setup(VIDEO_BLACK_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, VIDEO_BLACK_MOSI_PIN);
    //gpio_mode_setup(VIDEO_BLACK_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, VIDEO_BLACK_MOSI_PIN);
    gpio_set_af(VIDEO_BLACK_GPIO, VIDEO_BLACK_MOSI_AF, VIDEO_BLACK_MOSI_PIN);
    gpio_set_output_options(VIDEO_BLACK_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, VIDEO_BLACK_MOSI_PIN);
    gpio_clear(VIDEO_BLACK_GPIO, VIDEO_BLACK_MOSI_PIN);

    // set WHITE DAC to output:
    uint32_t pins = VIDEO_MUX_WHITE_DAC0 | VIDEO_MUX_WHITE_DAC1 | VIDEO_MUX_WHITE_DAC2 | VIDEO_MUX_WHITE_DAC3;
    gpio_mode_setup(VIDEO_MUX_WHITE_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pins);
    gpio_clear(VIDEO_MUX_WHITE_GPIO, pins);
    gpio_set(VIDEO_MUX_WHITE_GPIO, VIDEO_MUX_WHITE_DAC2);
    gpio_set(VIDEO_MUX_WHITE_GPIO, VIDEO_MUX_WHITE_DAC1);
    gpio_set(VIDEO_MUX_WHITE_GPIO, VIDEO_MUX_WHITE_DAC0);

    // set BLACK DAC to output:
    pins = VIDEO_MUX_BLACK_DAC0 | VIDEO_MUX_BLACK_DAC1 | VIDEO_MUX_BLACK_DAC2 | VIDEO_MUX_BLACK_DAC3;
    gpio_mode_setup(VIDEO_MUX_BLACK_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pins);
    gpio_clear(VIDEO_MUX_BLACK_GPIO, pins);
    gpio_set(VIDEO_MUX_BLACK_GPIO, VIDEO_MUX_BLACK_DAC1);



}


static void video_io_init_dac(void) {
    debug_function_call();

    // start with disabled dac
    dac_disable(CHANNEL_1);
    dac_disable_waveform_generation(CHANNEL_1);

    // set default value and enable output
    video_io_set_dac_value_mv(0);
    dac_enable(CHANNEL_1);

    // software update trigher
    dac_set_trigger_source(DAC_CR_TSEL1_SW);
}


// set dac to a given voltage level
void video_io_set_dac_value_mv(uint16_t target) {
    //debug_function_call_fixed1p3(target);

    uint32_t tmp = target;
    tmp = (tmp * 0x0FFF) / (VIDEO_DAC_VCC * 1000);
    video_io_set_dac_value_raw(tmp);
}

static void video_io_set_dac_value_raw(uint16_t target) {
    //debug_function_call_h32(target);

    dac_load_data_buffer_single(target, RIGHT12, CHANNEL_1);
    dac_software_trigger(CHANNEL_1);
}

