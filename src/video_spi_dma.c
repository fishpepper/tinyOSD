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

#include "video_spi_dma.h"
#include "video_timer.h"
#include "video.h"
#include "config.h"
#include "macros.h"
#include "debug.h"

#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencmsis/core_cm3.h>

static volatile uint32_t video_spi_dma_cr_trigger[2];

static void video_spi_dma_init_spi(uint32_t spi);
static void video_spi_dma_init_dma(void);

void video_spi_dma_init(void) {
    debug_function_call();

    //set up both spi ports
    video_spi_dma_init_spi(VIDEO_SPI_WHITE);
    video_spi_dma_init_spi(VIDEO_SPI_BLACK);

    // set up dma
    video_spi_dma_init_dma();
}


static void video_spi_dma_init_spi(uint32_t spi) {
    debug_function_call();

    // SPI NVIC
    // nvic_set_priority(NVIC_SPI2_IRQ, 3);
    // nvic_enable_irq(NVIC_SPI2_IRQ);

    // clean start
    spi_reset(spi);

    // set up spi
    // - master mode
    // - baud prescaler = apb_clk/2 = 48/2 = 24MHz!
    // - CPOL low
    // - CPHA 1
    // - 8 bit crc (?)
    // - MSB first
    spi_init_master(spi,
                    SPI_CR1_BAUDRATE_FPCLK_DIV_2,
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    SPI_CR1_CRCL_8BIT,
                    SPI_CR1_MSBFIRST);

    // set NSS to software
    // NOTE: setting NSS high is important! even when controling it on our
    //       own. otherwise spi will not send any data!
    spi_enable_software_slave_management(spi);
    spi_set_nss_high(spi);

    // Enable SPI periph
    spi_enable(spi);

    // set fifo to quarter full(=1 byte)
    spi_fifo_reception_threshold_8bit(spi);
}



static void video_spi_dma_init_dma(void) {
    debug_function_call();

    // start disabled
    dma_disable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL2);
    dma_disable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL3);

    dma_disable_channel(VIDEO_DMA_BLACK, DMA_CHANNEL4);
    dma_disable_channel(VIDEO_DMA_BLACK, DMA_CHANNEL5);

    // TIM1_CH1 = DMA CH2
    // -> write SPI  SPI_CR2(spi) |= SPI_CR2_TXDMAEN
    // this will initiate SPI transfer using DMA CH3

    // TIM1_CH4 = DMA CH4
    // write SPI txdma
    // this will initiate SPI tx on DMA CH5

    // SPI CH3 -> WHITE pixel
    nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, NVIC_PRIO_DMA1);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ);

    // SPI CH5 -> BLACK pixel
    nvic_set_priority(NVIC_DMA1_CHANNEL4_5_IRQ, NVIC_PRIO_DMA1);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_5_IRQ);


    /***********************************************************/

    // start with clean init
    dma_channel_reset(VIDEO_DMA_WHITE, DMA_CHANNEL3);

    // source and destination size
    dma_set_memory_size(VIDEO_DMA_WHITE, DMA_CHANNEL3, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(VIDEO_DMA_WHITE, DMA_CHANNEL3, DMA_CCR_PSIZE_16BIT);

    // auto memory destination increment mode
    dma_enable_memory_increment_mode(VIDEO_DMA_WHITE, DMA_CHANNEL3);
    // source address increment disable
    dma_disable_peripheral_increment_mode(VIDEO_DMA_WHITE, DMA_CHANNEL3);

    // Location assigned to peripheral register will be target
    dma_set_read_from_memory(VIDEO_DMA_WHITE, DMA_CHANNEL3);

    // source and destination start addresses
    dma_set_peripheral_address(VIDEO_DMA_WHITE, DMA_CHANNEL3, (uint32_t)&(SPI_DR(VIDEO_SPI_WHITE)));
    dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL3,
                           (uint32_t)&video_line.buffer[WHITE][video_line.currently_rendering][0]);

    // write full len
    dma_set_number_of_data(VIDEO_DMA_WHITE, DMA_CHANNEL3, VIDEO_BUFFER_WIDTH/2);

    // very high DMA_CHANNEL3
    dma_set_priority(VIDEO_DMA_WHITE, DMA_CHANNEL3, DMA_CCR_PL_VERY_HIGH);

    // enable tx complete int
    dma_enable_transfer_complete_interrupt(VIDEO_DMA_WHITE, DMA_CHANNEL3);

    // start with clean init
    dma_channel_reset(VIDEO_DMA_BLACK, DMA_CHANNEL5);

    // source and destination size
    dma_set_memory_size(VIDEO_DMA_BLACK, DMA_CHANNEL5, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(VIDEO_DMA_BLACK, DMA_CHANNEL5, DMA_CCR_PSIZE_16BIT);

    // auto memory destination increment mode
    dma_enable_memory_increment_mode(VIDEO_DMA_BLACK, DMA_CHANNEL5);
    // source address increment disable
    dma_disable_peripheral_increment_mode(VIDEO_DMA_BLACK, DMA_CHANNEL5);

    // Location assigned to peripheral register will be target
    dma_set_read_from_memory(VIDEO_DMA_BLACK, DMA_CHANNEL5);

    // source and destination start addresses
    dma_set_peripheral_address(VIDEO_DMA_BLACK, DMA_CHANNEL5, (uint32_t)&(SPI_DR(VIDEO_SPI_BLACK)));
    dma_set_memory_address(VIDEO_DMA_BLACK, DMA_CHANNEL5,
                           (uint32_t)&video_line.buffer[BLACK][video_line.currently_rendering][0]);

    // write full len
    dma_set_number_of_data(VIDEO_DMA_BLACK, DMA_CHANNEL5, VIDEO_BUFFER_WIDTH/2);

    // very high prio
    dma_set_priority(VIDEO_DMA_BLACK, DMA_CHANNEL5, DMA_CCR_PL_VERY_HIGH);

    // enable tx complete int
    dma_enable_transfer_complete_interrupt(VIDEO_DMA_BLACK, DMA_CHANNEL5);


    /***********************************************************/

    // start with clean init
    dma_channel_reset(VIDEO_DMA_WHITE, DMA_CHANNEL2);

    // source and destination size
    dma_set_memory_size(VIDEO_DMA_WHITE, DMA_CHANNEL2, DMA_CCR_MSIZE_32BIT);
    dma_set_peripheral_size(VIDEO_DMA_WHITE, DMA_CHANNEL2, DMA_CCR_PSIZE_32BIT);

    // memory destination increment disable
    dma_disable_memory_increment_mode(VIDEO_DMA_WHITE, DMA_CHANNEL2);
    // source address increment disable
    dma_disable_peripheral_increment_mode(VIDEO_DMA_WHITE, DMA_CHANNEL2);

    // Location assigned to peripheral register will be target
    dma_set_read_from_memory(VIDEO_DMA_WHITE, DMA_CHANNEL2);

    // source and destination start addresses
    dma_set_peripheral_address(VIDEO_DMA_WHITE, DMA_CHANNEL2, (uint32_t)&(SPI_CR2(VIDEO_SPI_WHITE)));

    //dma write will trigger SPI dma
    video_spi_dma_cr_trigger[0] = SPI_CR2(VIDEO_SPI_WHITE) | SPI_CR2_TXDMAEN;
    dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL2, (uint32_t)&(video_spi_dma_cr_trigger[0]));

    // single word write
    dma_set_number_of_data(VIDEO_DMA_WHITE, DMA_CHANNEL2, 1);
    // very high prio
    dma_set_priority(VIDEO_DMA_WHITE, DMA_CHANNEL2, DMA_CCR_PL_VERY_HIGH);



    // start with clean init
    dma_channel_reset(VIDEO_DMA_BLACK, DMA_CHANNEL4);

    // source and destination size
    dma_set_memory_size(VIDEO_DMA_BLACK, DMA_CHANNEL4, DMA_CCR_MSIZE_32BIT);
    dma_set_peripheral_size(VIDEO_DMA_BLACK, DMA_CHANNEL4, DMA_CCR_PSIZE_32BIT);

    // memory destination increment disable
    dma_disable_memory_increment_mode(VIDEO_DMA_BLACK, DMA_CHANNEL4);
    // source address increment disable
    dma_disable_peripheral_increment_mode(VIDEO_DMA_BLACK, DMA_CHANNEL4);

    // Location assigned to peripheral register will be target
    dma_set_read_from_memory(VIDEO_DMA_BLACK, DMA_CHANNEL4);

    // source and destination start addresses
    dma_set_peripheral_address(VIDEO_DMA_BLACK, DMA_CHANNEL4, (uint32_t)&(SPI_CR2(VIDEO_SPI_BLACK)));

    //dma write will trigger SPI dma
    video_spi_dma_cr_trigger[1] = SPI_CR2(VIDEO_SPI_BLACK) | SPI_CR2_TXDMAEN;
    dma_set_memory_address(VIDEO_DMA_BLACK, DMA_CHANNEL4, (uint32_t)&(video_spi_dma_cr_trigger[1]));

    // single word write
    dma_set_number_of_data(VIDEO_DMA_BLACK, DMA_CHANNEL4, 1);

    // very high prio
    dma_set_priority(VIDEO_DMA_BLACK, DMA_CHANNEL4, DMA_CCR_PL_VERY_HIGH);
}



// BLACK DMA
void DMA1_CHANNEL4_5_IRQHandler(void) {
    // clear flags
    DMA_CLEAR_INTERRUPT_FLAGS_MULTI(VIDEO_DMA_BLACK,
                                    (DMA_TCIF) << DMA_FLAG_OFFSET(DMA_CHANNEL4) |
                                    (DMA_TCIF) << DMA_FLAG_OFFSET(DMA_CHANNEL5));

    // disable dma during config
    DMA_DISABLE_CHANNEL(VIDEO_DMA_BLACK, DMA_CHANNEL4);
    DMA_DISABLE_CHANNEL(VIDEO_DMA_BLACK, DMA_CHANNEL5);

    //clear pending dma transfer flag
    SPI_CR2(VIDEO_SPI_BLACK) &= ~SPI_CR2_TXDMAEN;


    // prepare to send tx trigger
    DMA_SET_NUMBER_OF_DATA(VIDEO_DMA_BLACK, DMA_CHANNEL4, 1);


    // prepare next page rendering:
    video_line.currently_rendering = 1 - video_line.currently_rendering;
    video_line.fill_request        = video_line.currently_rendering;

    // prepare to send dma spi data
    DMA_SET_MEMORY_ADDRES_NOCHECK(VIDEO_DMA_BLACK, DMA_CHANNEL5,
                                  (uint32_t)&video_line.buffer[BLACK][video_line.currently_rendering][0]);
    DMA_SET_NUMBER_OF_DATA(VIDEO_DMA_BLACK, DMA_CHANNEL5, VIDEO_BUFFER_WIDTH/2);

    // clear all dma if
    // NOT NECESSARY? move to define if enable is necc...
   //  dma_clear_interrupt_flags(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_TCIF);


    // disable timer match in order to clear pending timer triggers
    //TIMER_CLEAR_DMA_ON_COMPARE_EVENT(TIM1);
    TIMER_CLEAR_FLAG(TIM1, TIM_SR_CC1IF | TIM_SR_CC4IF);
    TIMER_DISABLE_IRQ(TIM1, TIM_DIER_CC1DE | TIM_DIER_CC4DE);
}

// WHITE DMA
void DMA1_CHANNEL2_3_IRQHandler(void) {
    // clear flags
    DMA_CLEAR_INTERRUPT_FLAGS_MULTI(VIDEO_DMA_BLACK,
                                    (DMA_TCIF) << DMA_FLAG_OFFSET(DMA_CHANNEL2) |
                                    (DMA_TCIF) << DMA_FLAG_OFFSET(DMA_CHANNEL3));

    // disable dma during config
    DMA_DISABLE_CHANNEL(VIDEO_DMA_WHITE, DMA_CHANNEL2);
    DMA_DISABLE_CHANNEL(VIDEO_DMA_WHITE, DMA_CHANNEL3);

    //clear pending dma transfer flag
    SPI_CR2(VIDEO_SPI_WHITE) &= ~SPI_CR2_TXDMAEN;

    // prepare to send tx trigger
    DMA_SET_NUMBER_OF_DATA(VIDEO_DMA_WHITE, DMA_CHANNEL2, 1);

    // prepare to send dma spi data
    DMA_SET_MEMORY_ADDRES_NOCHECK(VIDEO_DMA_WHITE, DMA_CHANNEL3,
                                  (uint32_t)&video_line.buffer[WHITE][video_line.currently_rendering][0]);
    DMA_SET_NUMBER_OF_DATA(VIDEO_DMA_WHITE, DMA_CHANNEL3, VIDEO_BUFFER_WIDTH/2);

    // clear all dma if
    // NOT NECESSARY? move to define if enable is necc...
   //  dma_clear_interrupt_flags(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_TCIF);
}
