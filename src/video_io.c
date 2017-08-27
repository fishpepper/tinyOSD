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
#include "video.h"
#include "config.h"
#include "macros.h"
#include "delay.h"
#include "serial.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dac.h>
#include "debug.h"

static void video_io_init_rcc(void);
static void video_io_init_gpio(void);
static void video_io_init_dac(void);
static void video_io_set_dac_value_raw(uint16_t taregt);

static void video_io_set_white_level_raw(uint8_t byte);
static void video_io_set_black_level_raw(uint8_t byte);

void video_io_init(void) {
    video_io_init_rcc();
    video_io_init_gpio();
    video_io_init_dac();

    video_io_set_level_mv(WHITE, 1000);
    video_io_set_level_mv(BLACK,   0);
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
    //gpio_set_output_options(VIDEO_GPIO, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, VIDEO_DAC_OUT_PIN);

    // set spi mosi to output
    gpio_mode_setup(VIDEO_WHITE_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, VIDEO_WHITE_MOSI_PIN);
    gpio_set_af(VIDEO_WHITE_GPIO, VIDEO_WHITE_MOSI_AF, VIDEO_WHITE_MOSI_PIN);
    gpio_set_output_options(VIDEO_WHITE_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, VIDEO_WHITE_MOSI_PIN);
    GPIO_CLEAR(VIDEO_WHITE_GPIO, VIDEO_WHITE_MOSI_PIN);

    // set spi mosi to output
    gpio_mode_setup(VIDEO_BLACK_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, VIDEO_BLACK_MOSI_PIN);
    gpio_set_af(VIDEO_BLACK_GPIO, VIDEO_BLACK_MOSI_AF, VIDEO_BLACK_MOSI_PIN);
    gpio_set_output_options(VIDEO_BLACK_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, VIDEO_BLACK_MOSI_PIN);
    GPIO_CLEAR(VIDEO_BLACK_GPIO, VIDEO_BLACK_MOSI_PIN);
/*
    gpio_mode_setup(VIDEO_WHITE_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, VIDEO_WHITE_MOSI_PIN);
    gpio_mode_setup(VIDEO_BLACK_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, VIDEO_BLACK_MOSI_PIN);
    while(1){
        GPIO_TOGGLE(VIDEO_WHITE_GPIO, VIDEO_WHITE_MOSI_PIN);
        GPIO_TOGGLE(VIDEO_BLACK_GPIO, VIDEO_BLACK_MOSI_PIN);
    }
*/
    // set WHITE DAC to output:
    uint32_t pins = VIDEO_MUX_WHITE_DAC0 | VIDEO_MUX_WHITE_DAC1 | VIDEO_MUX_WHITE_DAC2 | VIDEO_MUX_WHITE_DAC3;
    gpio_mode_setup(VIDEO_MUX_WHITE_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pins);
    GPIO_CLEAR(VIDEO_MUX_WHITE_GPIO, pins);
    //GPIO_SET(VIDEO_MUX_WHITE_GPIO, VIDEO_MUX_WHITE_DAC2);
    //GPIO_SET(VIDEO_MUX_WHITE_GPIO, VIDEO_MUX_WHITE_DAC1);
    //GPIO_SET(VIDEO_MUX_WHITE_GPIO, VIDEO_MUX_WHITE_DAC0);

    // set BLACK DAC to output:
    pins = VIDEO_MUX_BLACK_DAC0 | VIDEO_MUX_BLACK_DAC1 | VIDEO_MUX_BLACK_DAC2 | VIDEO_MUX_BLACK_DAC3;
    gpio_mode_setup(VIDEO_MUX_BLACK_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pins);
    GPIO_CLEAR(VIDEO_MUX_BLACK_GPIO, pins);
    //GPIO_SET(VIDEO_MUX_BLACK_GPIO, VIDEO_MUX_BLACK_DAC1);
}



void video_io_set_level_mv(uint8_t port, uint16_t mv){
    ///debug_function_call_u16(mv);

    ///debug("video_io: v_white = "); debug_put_uint16(v_white); debug_put_newline();

    // limit to be inside allowed range of 0..700 (+200) mV
    mv = min(900, mv);

    //debug("video_io: v_set   = "); debug_put_uint16(mv); debug_put_newline();

    // add offset to pal levels:
    //   0 % = video signal min + black level
    // 100 % = video signal min + black level + 700mV
    // allow for a bit more, set black level to 250mV, max is 250+900 = 1150mV
    mv = mv + video_min_level + 250;

    // input: target voltage in mv
    // the output is terminated with 75 Ohm
    // minimum 1/Rtotal = 1/8R + 1/4R + 1/2R + 1/R
    // Rtotal = 8R / 15
    // Vmax   = CPU_VOLTAGE * VIDEO_LEVEL_TERMINATION_R / (VIDEO_LEVEL_TERMINATION_R + Rtotal)
    float r_total = (8.0 * VIDEO_LEVEL_R2R_1R) / 15.0;
    float mv_max   = (1000.0 * CPU_VOLTAGE) * VIDEO_LEVEL_TERMINATION_R / (VIDEO_LEVEL_TERMINATION_R + r_total);

    // every lsb will give 1/15th of v_max:
    float mv_lsb   = mv_max / 15.0;

    float raw = mv / mv_lsb;

    if (raw > 15.0) {
        raw = 15;
    }

    if (VTX_FEATURE_ENABLED(OPENTCO_OSD_FEATURE_INVERT)) {
        // switch colors
        //raw = 15 - raw;
        port = (port == WHITE) ? BLACK : WHITE;
    }

    if (port == WHITE) {
        video_io_set_white_level_raw((uint8_t)raw);
    } else {
        video_io_set_black_level_raw((uint8_t)raw);
    }
}

static void video_io_set_white_level_raw(uint8_t byte){
    // input: raw value, lower 4 bit will be set on R2R DAC
    uint32_t out = 0;

    // stm32 BSRR register is tricky:
    // writing to lower 16 bits set the I/O pin
    // writing to higher 16 bits clear the I/O pin
    if (byte & (1<<0)) { out |= VIDEO_MUX_WHITE_DAC0; } else { out |= (VIDEO_MUX_WHITE_DAC0)<<16; }
    if (byte & (1<<1)) { out |= VIDEO_MUX_WHITE_DAC1; } else { out |= (VIDEO_MUX_WHITE_DAC1)<<16; }
    if (byte & (1<<2)) { out |= VIDEO_MUX_WHITE_DAC2; } else { out |= (VIDEO_MUX_WHITE_DAC2)<<16; }
    if (byte & (1<<3)) { out |= VIDEO_MUX_WHITE_DAC3; } else { out |= (VIDEO_MUX_WHITE_DAC3)<<16; }

    // execute:
    GPIO_BSRR(VIDEO_MUX_WHITE_GPIO) = out;
}

static void video_io_set_black_level_raw(uint8_t byte){
    // input: raw value, lower 4 bit will be set on R2R DAC
    uint32_t out = 0;

    // stm32 BSRR register is tricky:
    // writing to lower 16 bits set the I/O pin
    // writing to higher 16 bits clear the I/O pin
    if (byte & (1<<0)) { out |= VIDEO_MUX_BLACK_DAC0; } else { out |= (VIDEO_MUX_BLACK_DAC0)<<16; }
    if (byte & (1<<1)) { out |= VIDEO_MUX_BLACK_DAC1; } else { out |= (VIDEO_MUX_BLACK_DAC1)<<16; }
    if (byte & (1<<2)) { out |= VIDEO_MUX_BLACK_DAC2; } else { out |= (VIDEO_MUX_BLACK_DAC2)<<16; }
    if (byte & (1<<3)) { out |= VIDEO_MUX_BLACK_DAC3; } else { out |= (VIDEO_MUX_BLACK_DAC3)<<16; }

    // execute:
    GPIO_BSRR(VIDEO_MUX_BLACK_GPIO) = out;
}

static void video_io_init_dac(void) {
    debug_function_call();

    // start with disabled dac
    dac_disable(CHANNEL_1);
    dac_disable_waveform_generation(CHANNEL_1);
    //dac_buffer_disable(CHANNEL_1);

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
    //dac_load_data_buffer_single(target, RIGHT12, CHANNEL_1);
    dac_load_data_buffer_single(target, RIGHT12, CHANNEL_1);
    dac_software_trigger(CHANNEL_1);
}

