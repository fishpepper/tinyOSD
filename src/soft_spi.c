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

#include "soft_spi.h"
#include "macros.h"
#include "config.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void soft_spi_init_gpio(void);
static void soft_spi_init_rcc(void);

void soft_spi_init(void) {
    soft_spi_init_rcc();
    soft_spi_init_gpio();
}


static void soft_spi_init_rcc(void) {
    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(SOFT_SPI_GPIO));
}




static void soft_spi_init_gpio(void) {
    // set MOSI, SCK and CS pin as output
    uint32_t pins = SOFT_SPI_PIN_CS | SOFT_SPI_PIN_SCK | SOFT_SPI_PIN_MOSI;

    gpio_mode_setup(SOFT_SPI_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pins);
    gpio_set_output_options(SOFT_SPI_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, pins);
}

