/*
    Copyright 2016 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    author: fishpepper <AT> gmail.com
*/

#include "clocksource.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>

uint32_t rcc_timer_frequency;

void clocksource_init(void) {
    // set clock source
    clocksource_hse_in_8_out_48();
}

void clocksource_hse_in_8_out_48(void) {
    // see RM0366 p. 92 for clock tree

    // enable internal high-speed oscillator
    // we will run from hsi during setup
    rcc_osc_on(RCC_HSI);
    rcc_wait_for_osc_ready(RCC_HSI);

    // Select HSI as SYSCLK source.
    rcc_set_sysclk_source(RCC_CFGR_SW_HSI);

    // Enable external high-speed oscillator 8MHz
    rcc_osc_on(RCC_HSE);
    rcc_wait_for_osc_ready(RCC_HSE);

    // disable pll during setup (madatory!)
    rcc_osc_off(RCC_PLL);
    rcc_wait_for_osc_not_ready(RCC_PLL);

    // set up HSE 8MHz to PLL out 48 MHz
    // PL OUT = HSE/2 * 12 = 4 * 12 = 48
    rcc_set_prediv(RCC_CFGR2_PREDIV_DIV2);
    rcc_set_pll_source(RCC_CFGR_PLLSRC_HSE_PREDIV);
    rcc_set_pll_multiplier(RCC_CFGR_PLLMUL_PLL_IN_CLK_X12);

    // start pll
    rcc_osc_on(RCC_PLL);
    rcc_wait_for_osc_ready(RCC_PLL);

    // set up prescalers for AHB, ADC, ABP1, ABP2.
    // do this before setting sysclock source to PLL
    // otherwise we might run the peripherals at a frequency
    // that exceeds the limits
    rcc_set_hpre(RCC_CFGR_HPRE_DIV_NONE);    // 48MHz (max: 72)
    rcc_set_ppre2(RCC_CFGR_PPRE2_DIV_2);     // 24MHz (max: 72)
    rcc_set_ppre1(RCC_CFGR_PPRE1_DIV_2);     // 24MHz (max: 36)

    // set flash waitstates
    flash_set_ws(FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1WS);

    // finally select PLL as SYSCLK source
    rcc_set_sysclk_source(RCC_CFGR_SW_PLL);

    // set the peripheral clock frequencies used */
    rcc_ahb_frequency  = 48000000;
    rcc_apb1_frequency = 24000000;
    rcc_apb2_frequency = 24000000;
}

