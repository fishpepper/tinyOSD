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
static void video_io_set_dac_value_mv(uint16_t target);
static void video_io_set_dac_value_raw(uint16_t taregt);

void video_io_init(void) {
    video_io_init_rcc();
    video_io_init_gpio();
    video_io_init_dac();
    video_io_set_dac_value_mv(100);
}


static void video_io_init_rcc(void) {
    debug_function_call();

    // DAC clock
    rcc_periph_clock_enable(RCC_DAC);

    // for EXTI
    rcc_periph_clock_enable(RCC_SYSCFG_COMP);

    // timer1 clock
    rcc_periph_clock_enable(RCC_TIM1);

    // timer2 clock
    //rcc_periph_clock_enable(RCC_TIM2);

    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(VIDEO_GPIO));
    rcc_periph_clock_enable(GPIO_RCC(VIDEO_GPIO_BLACK));

    // spi
    rcc_periph_clock_enable(VIDEO_SPI_WHITE_RCC);
    rcc_periph_clock_enable(VIDEO_SPI_BLACK_RCC);

    // enable DMA Peripheral Clock
    rcc_periph_clock_enable(RCC_DMA);
}

static void video_io_init_gpio(void) {
    debug_function_call();

    // set video input pin as input
    gpio_mode_setup(VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VIDEO_INPUT_PIN);

    // set dac to output
    gpio_mode_setup(VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VIDEO_DAC_OUT_PIN);
    gpio_set_output_options(VIDEO_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, VIDEO_DAC_OUT_PIN);

    // set spi to output
    // init sck (5, for dbg), MOSI (7)
    uint32_t spi_gpios = GPIO3 | GPIO5;
    // set mode
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, spi_gpios);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, spi_gpios);
    gpio_set(GPIOB, GPIO3);

    // set spi to output
    // init sck (13, for dbg), MOSI (15)
    spi_gpios = GPIO15 | GPIO13;
    // set mode
    gpio_mode_setup(VIDEO_GPIO_BLACK, GPIO_MODE_AF, GPIO_PUPD_NONE, spi_gpios);
    gpio_set_output_options(VIDEO_GPIO_BLACK, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, spi_gpios);
    gpio_set(VIDEO_GPIO_BLACK, GPIO15);
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
static void video_io_set_dac_value_mv(uint16_t target) {
    debug_function_call_fixed1p3(target);

    uint32_t tmp = target;
    tmp = (tmp * 0x0FFF) / (VIDEO_DAC_VCC * 1000);
    video_io_set_dac_value_raw(tmp);
}

static void video_io_set_dac_value_raw(uint16_t target) {
    debug_function_call_h32(target);

    dac_load_data_buffer_single(target, RIGHT12, CHANNEL_1);
    dac_software_trigger(CHANNEL_1);
}
