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
#include "logo.h"

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
static void video_init_spi_single(uint32_t spi);
static void video_init_spi_dma(void);
static void video_init_pendsv(void);

static void video_dma_prepare();
static void video_dma_trigger(void);

static void video_set_dac_value_mv(uint16_t target);
static void video_set_dac_value_raw(uint16_t target);

#define VIDEO_BUFFER_WIDTH 70
volatile uint8_t video_buffer[2][VIDEO_BUFFER_WIDTH+1];
volatile uint8_t video_buffer_b[2][VIDEO_BUFFER_WIDTH+1];

volatile uint32_t video_dbg;
volatile uint32_t video_line;
volatile uint32_t video_field;
volatile uint16_t video_sync_last_compare_value;

volatile uint32_t video_spi_cr_trigger;
volatile uint32_t video_spi_cr_trigger_b;


volatile uint32_t video_buffer_fill_request;
volatile uint32_t video_buffer_page;
#define VIDEO_BUFFER_FILL_REQUEST_IDLE 3


// void TIM1_CC_IRQHandler(void) {
void TIM1_CC_IRQHandler(void) {
    if (timer_get_flag(TIM1, TIM_SR_CC4IF)) {
        timer_clear_flag(TIM1, TIM_SR_CC4IF);
        //timer_disable_irq(TIM1, TIM_DIER_CC2IE);
        //while(1) led_toggle();
       // led_toggle();
        debug("CC MATCH\n");
//        led_off();
    }
}

static const uint32_t video_cos_table[90] = {
    0x64, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
    0x63, 0x62, 0x62, 0x62, 0x61, 0x61, 0x61, 0x60,
    0x60, 0x5f, 0x5f, 0x5e, 0x5d, 0x5d, 0x5c, 0x5c,
    0x5b, 0x5a, 0x59, 0x59, 0x58, 0x57, 0x56, 0x55,
    0x54, 0x53, 0x52, 0x51, 0x50, 0x4f, 0x4e, 0x4d,
    0x4c, 0x4b, 0x4a, 0x49, 0x47, 0x46, 0x45, 0x44,
    0x42, 0x41, 0x40, 0x3e, 0x3d, 0x3c, 0x3a, 0x39,
    0x37, 0x36, 0x35, 0x33, 0x32, 0x30, 0x2e, 0x2d,
    0x2b, 0x2a, 0x28, 0x27, 0x25, 0x23, 0x22, 0x20,
    0x1e, 0x1d, 0x1b, 0x19, 0x18, 0x16, 0x14, 0x13,
    0x11, 0xf, 0xd, 0xc, 0xa, 0x8, 0x7, 0x5, 0x3,
    0x1
};


void video_init(void) {
    debug_function_call();

    // uint16_t tmp = 0;

    for(uint8_t i=0; i<VIDEO_BUFFER_WIDTH-1; i++) {
        video_buffer[0][i] = 0;
        video_buffer[1][i] = 0;

        video_buffer_b[0][i] = 0xFF;
        video_buffer_b[0][i] = 0xFF;
    }



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

    for(uint32_t i=0; i<VIDEO_BUFFER_WIDTH; i++){
        video_buffer[0][i] = 0xFF;
    }

    video_buffer[0][0] = 0x80;

    //video_buffer[0][VIDEO_BUFFER_WIDTH/2] = 0xAA;

    // this is the lastbyte that is transferred:
    video_buffer[0][VIDEO_BUFFER_WIDTH-3] = 0x80;


    /*for(uint32_t i=VIDEO_BUFFER_WIDTH-4; i<VIDEO_BUFFER_WIDTH-3; i++){
        video_buffer[0][i] = 0xFF;
    }*/

    video_buffer_fill_request = VIDEO_BUFFER_FILL_REQUEST_IDLE;

    led_off();


    video_dma_prepare();

uint32_t data = 0;
uint32_t logo_start_line = 625/2 - LOGO_HEIGHT/2;
uint32_t logo_end_line   = 625/2 + LOGO_HEIGHT/2;
uint32_t logo_offset_x   = VIDEO_BUFFER_WIDTH/2 - LOGO_WIDTH/8/2;
uint32_t logo_offset;
uint8_t *logo_ptr;
uint8_t *framebuffer_ptr;
uint8_t *video_buffer_ptr;

uint32_t ani_count = 0;
uint32_t ani_dir   = 0;

video_buffer_b[0][VIDEO_BUFFER_WIDTH/2] = 0;
video_buffer_b[1][VIDEO_BUFFER_WIDTH/2] = 0;

for (uint32_t i= 0; i<2; i++){
    for (uint32_t j= 0; j<2; j++){
video_buffer_b[i][j] = 0x38;

video_buffer[i][j] = 0x38;
    }
}

while(1){
    video_dma_prepare();
 //spi_enable_tx_dma(SPI2);
 //spi_enable_tx_dma(SPI1);
 delay_us(10*1000);
}

while(1){


    if (video_buffer_fill_request != VIDEO_BUFFER_FILL_REQUEST_IDLE) {
        if (video_line == 2) {
            ani_count+=4;
            if (ani_count >=360) {
                ani_count -= 360;
            }
        }

        uint32_t scale;
        if (ani_count < 90) {
            scale   = video_cos_table[ani_count];
            ani_dir = 0;
        }else if (ani_count < 180) {
            scale   = video_cos_table[180 - ani_count];
            ani_dir = 1;
        }else if (ani_count < 270) {
            scale   = video_cos_table[ani_count - 180];
            ani_dir = 1;
        }else{
            scale   = video_cos_table[360 - ani_count];
            ani_dir = 0;
        }

        // we have a new request, osd is rendering the other page now,
        // time to prepare the next page!
        //debug("filling page "); debug_put_uint8(video_buffer_fill_request); debug_put_newline();

        // calculate line number:
        uint32_t line    = video_line + 2;

        logo_start_line = 625/2 - (scale * LOGO_HEIGHT/2) / 100;
        logo_end_line   = 625/2 + (scale * LOGO_HEIGHT/2) / 100;

        // fetch correct buffer ptr
        video_buffer_ptr = &video_buffer[video_buffer_fill_request][0];

        // fill the next line of data now:
        if ((line > logo_start_line) && (line < logo_end_line)){
            logo_offset = (line - logo_start_line) * 100 / scale * (LOGO_WIDTH/8);
            if (ani_dir) {
                logo_ptr = &logo_data[logo_offset];
            } else {
                logo_ptr = &logo_data[LOGO_HEIGHT*LOGO_WIDTH/8 - logo_offset];
            }


            for (uint32_t i = 0; i < logo_offset_x; i++){
                *video_buffer_ptr++ = 0;
            }
            for(uint32_t i = 0; i < LOGO_WIDTH/8; i++){
                *video_buffer_ptr++ = *logo_ptr++;
            }
            for (uint32_t i =0; i < VIDEO_BUFFER_WIDTH; i++){
                *video_buffer_ptr++ = 0;
            }

        }else{
            // no image data region
            for(uint32_t x = 0; x < VIDEO_BUFFER_WIDTH-1; x++){
               *video_buffer_ptr++ = 0;
            }
        }

        // always make sure the last pixel is zero
        // THIS IS MANDATORY! otherwise the spi level stay high and ruins the image
        video_buffer[video_buffer_fill_request][VIDEO_BUFFER_WIDTH-1] &= 0xF7;


        // clear request
        video_buffer_fill_request = VIDEO_BUFFER_FILL_REQUEST_IDLE;
        //debug("ok\n");
    }

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
    video_init_spi_single(VIDEO_SPI_WHITE);
    video_init_spi_single(VIDEO_SPI_BLACK);
}

static void video_init_spi_single(uint32_t spi) {
    debug_function_call();

    // SPI NVIC
    // nvic_set_priority(NVIC_SPI2_IRQ, 3);
    // nvic_enable_irq(NVIC_SPI2_IRQ);

    // clean start
    spi_reset(spi);

    // set up spi
    // - master mode
    // - baud prescaler = apb_clk/2 = 24/2 = 12MHz!
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


void DMA1_CHANNEL4_5_IRQHandler(void) {
    // clear flag
    dma_clear_interrupt_flags(VIDEO_DMA_WHITE, DMA_CHANNEL4, DMA_TCIF);
    dma_clear_interrupt_flags(VIDEO_DMA_WHITE, DMA_CHANNEL5, DMA_TCIF);
    led_toggle();
}

void DMA1_CHANNEL2_3_IRQHandler(void) {
    // clear flag
    dma_clear_interrupt_flags(VIDEO_DMA_WHITE, DMA_CHANNEL2, DMA_TCIF);
    dma_clear_interrupt_flags(VIDEO_DMA_WHITE, DMA_CHANNEL3, DMA_TCIF);

    led_off();
}

static void video_init_spi_dma(void) {
    debug_function_call();

    // start disabled
    dma_disable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL2);
    dma_disable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL3);
    dma_disable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL4);
    dma_disable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL5);

    // TIM1_CH1 = DMA CH2
    // -> write SPI  SPI_CR2(spi) |= SPI_CR2_TXDMAEN
    // this will initiate SPI transfer using DMA CH3

    // TIM1_CH4 = DMA CH4
    // write SPI txdma
    // this will initiate SPI tx on DMA CH5

    // SPI CH3 -> WHITE pixel
    nvic_set_priority(NVIC_DMA1_CHANNEL2_3_IRQ, NVIC_PRIO_DMA1);
    nvic_enable_irq(NVIC_DMA1_CHANNEL2_3_IRQ); //
    // SPI CH5 -> BLACK pixel
    nvic_set_priority(NVIC_DMA1_CHANNEL4_5_IRQ, NVIC_PRIO_DMA1);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_5_IRQ); //


    /***********************************************************/

    // start with clean init
    dma_channel_reset(VIDEO_DMA_WHITE, DMA_CHANNEL3);

    // source and destination size
    dma_set_memory_size(VIDEO_DMA_WHITE, DMA_CHANNEL3, DMA_CCR_MSIZE_8BIT);
    dma_set_peripheral_size(VIDEO_DMA_WHITE, DMA_CHANNEL3, DMA_CCR_PSIZE_8BIT);

    // auto memory destination increment mode
    dma_enable_memory_increment_mode(VIDEO_DMA_WHITE, DMA_CHANNEL3);
    // source address increment disable
    dma_disable_peripheral_increment_mode(VIDEO_DMA_WHITE, DMA_CHANNEL3);

    // Location assigned to peripheral register will be target
    dma_set_read_from_memory(VIDEO_DMA_WHITE, DMA_CHANNEL3);

    // source and destination start addresses
    dma_set_peripheral_address(VIDEO_DMA_WHITE, DMA_CHANNEL3, (uint32_t)&(SPI_DR(VIDEO_SPI_WHITE)));
    dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL3, (uint32_t)&(video_buffer[0]));

    // write full len
    dma_set_number_of_data(VIDEO_DMA_WHITE, DMA_CHANNEL3, VIDEO_BUFFER_WIDTH);

    // very high DMA_CHANNEL3
    dma_set_priority(VIDEO_DMA_WHITE, DMA_CHANNEL3, DMA_CCR_PL_VERY_HIGH);

    // enable tx complete int
    // DO NOT USE INT dma_enable_transfer_complete_interrupt(VIDEO_DMA_WHITE, DMA_CHANNEL3);




    // start with clean init
    dma_channel_reset(VIDEO_DMA_WHITE, DMA_CHANNEL5);

    // source and destination size
    dma_set_memory_size(VIDEO_DMA_WHITE, DMA_CHANNEL5, DMA_CCR_MSIZE_8BIT);
    dma_set_peripheral_size(VIDEO_DMA_WHITE, DMA_CHANNEL5, DMA_CCR_PSIZE_8BIT);

    // auto memory destination increment mode
    dma_enable_memory_increment_mode(VIDEO_DMA_WHITE, DMA_CHANNEL5);
    // source address increment disable
    dma_disable_peripheral_increment_mode(VIDEO_DMA_WHITE, DMA_CHANNEL5);

    // Location assigned to peripheral register will be target
    dma_set_read_from_memory(VIDEO_DMA_WHITE, DMA_CHANNEL5);

    // source and destination start addresses
    dma_set_peripheral_address(VIDEO_DMA_WHITE, DMA_CHANNEL5, (uint32_t)&(SPI_DR(VIDEO_SPI_BLACK)));
    dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL5, (uint32_t)&(video_buffer_b[0]));

    // write full len
    dma_set_number_of_data(VIDEO_DMA_WHITE, DMA_CHANNEL5, VIDEO_BUFFER_WIDTH);

    // very high DMA_CHANNEL3
    dma_set_priority(VIDEO_DMA_WHITE, DMA_CHANNEL5, DMA_CCR_PL_VERY_HIGH);

    // enable tx complete int
    dma_enable_transfer_complete_interrupt(VIDEO_DMA_WHITE, DMA_CHANNEL5);


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
    video_spi_cr_trigger = SPI_CR2(VIDEO_SPI_WHITE) | SPI_CR2_TXDMAEN;
    dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL2, (uint32_t)&(video_spi_cr_trigger));

    // single word write
    dma_set_number_of_data(VIDEO_DMA_WHITE, DMA_CHANNEL2, 1);
    // very high prio
    dma_set_priority(VIDEO_DMA_WHITE, DMA_CHANNEL2, DMA_CCR_PL_VERY_HIGH);



    // start with clean init
    dma_channel_reset(VIDEO_DMA_WHITE, DMA_CHANNEL4);

    // source and destination size
    dma_set_memory_size(VIDEO_DMA_WHITE, DMA_CHANNEL4, DMA_CCR_MSIZE_32BIT);
    dma_set_peripheral_size(VIDEO_DMA_WHITE, DMA_CHANNEL4, DMA_CCR_PSIZE_32BIT);

    // memory destination increment disable
    dma_disable_memory_increment_mode(VIDEO_DMA_WHITE, DMA_CHANNEL4);
    // source address increment disable
    dma_disable_peripheral_increment_mode(VIDEO_DMA_WHITE, DMA_CHANNEL4);

    // Location assigned to peripheral register will be target
    dma_set_read_from_memory(VIDEO_DMA_WHITE, DMA_CHANNEL4);

    // source and destination start addresses
    dma_set_peripheral_address(VIDEO_DMA_WHITE, DMA_CHANNEL4, (uint32_t)&(SPI_CR2(VIDEO_SPI_BLACK)));

    //dma write will trigger SPI dma
    video_spi_cr_trigger_b = SPI_CR2(VIDEO_SPI_BLACK) | SPI_CR2_TXDMAEN;
    dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL4, (uint32_t)&(video_spi_cr_trigger_b));

    // single word write
    dma_set_number_of_data(VIDEO_DMA_WHITE, DMA_CHANNEL4, 1);
    // very high prio
    dma_set_priority(VIDEO_DMA_WHITE, DMA_CHANNEL4, DMA_CCR_PL_VERY_HIGH);
}

void video_dma_prepare() {
    // disable dma during config
    dma_disable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL2);
    dma_disable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL3);
    dma_disable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL4);
    dma_disable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL5);


    // prepare to send tx trigger
    /*video_spi_cr_trigger = SPI_CR2(SPI1) | SPI_CR2_TXDMAEN;
    dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL2, (uint32_t)&(video_spi_cr_trigger));
    dma_set_number_of_data(VIDEO_DMA_WHITE, DMA_CHANNEL2, 1);*/

    //video_spi_cr_trigger_b = SPI_CR2(SPI1) | SPI_CR2_TXDMAEN;
    //dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL4, (uint32_t)&(video_spi_cr_trigger_b));
    //dma_set_number_of_data(VIDEO_DMA_WHITE, DMA_CHANNEL4, 1);

    // prepare next page:
    video_buffer_fill_request = video_buffer_page;
    video_buffer_page         = 1 - video_buffer_page;

    // prepare to send dma spi data
    dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL3, (uint32_t)&(video_buffer[video_buffer_page]));
    dma_set_number_of_data(VIDEO_DMA_WHITE, DMA_CHANNEL3, 4);

    dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL5, (uint32_t)&(video_buffer_b[video_buffer_page]));
    dma_set_number_of_data(VIDEO_DMA_WHITE, DMA_CHANNEL5, 4);

    strange, why do i not need reset cr2 triogger?! why does some numbers work as bars, some txcount do not?!
                                                                                                            mutli trigger?


    // clear all dma if
    //dma_clear_interrupt_flags(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_TCIF);

    // clear pending dma request from timer
    //timer_disable_irq(TIM2, TIM_DIER_CC2DE);
    //timer_enable_irq(TIM2, TIM_DIER_CC2DE);

    // enable dma channel
    dma_enable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL3);
    dma_enable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL2);

    dma_enable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL5);
    dma_enable_channel(VIDEO_DMA_WHITE, DMA_CHANNEL4);
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
    rcc_periph_clock_enable(GPIO_RCC(VIDEO_GPIO_BLACK));

    // spi
    rcc_periph_clock_enable(VIDEO_SPI_WHITE_RCC);
    rcc_periph_clock_enable(VIDEO_SPI_BLACK_RCC);

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
    gpio_set(VIDEO_GPIO, GPIO7);

    // set spi to output
    // init sck (13, for dbg), MOSI (15)
    spi_gpios = GPIO15 | GPIO13;
    // set mode
    gpio_mode_setup(VIDEO_GPIO_BLACK, GPIO_MODE_AF, GPIO_PUPD_NONE, spi_gpios);
    gpio_set_output_options(VIDEO_GPIO_BLACK, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, spi_gpios);
    gpio_set(VIDEO_GPIO_BLACK, GPIO15);
}

static void video_init_comparator(void) {
    debug_function_call();

    // start disabled
    comp_disable(COMP1);

    // set comparator inputs
    // inp = PA1
    // inm = DAC_OUT_1 (PA4) -> INM4
    comp_select_input(COMP1, COMP_CSR_INSEL_INM4);

    // IC2 output
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

    //timer_enable_irq(TIM1, TIM_DIER_CC2IE);

    // Set the timers global mode to:
    // - use no divider
    // - alignment edge
    // - count direction up
    timer_set_mode(TIM1,
                   TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE,
                   TIM_CR1_DIR_UP);

    // input compare trigger
    // on channel IC2
    timer_ic_set_input(TIM1, TIM_IC2, TIM_IC_IN_TI1);
    timer_ic_set_polarity(TIM1, TIM_IC2, TIM_IC_BOTH);
    timer_ic_set_prescaler(TIM1, TIM_IC2, TIM_IC_PSC_OFF);
    timer_ic_set_filter(TIM1, TIM_IC2, TIM_IC_OFF);
    timer_ic_enable(TIM1, TIM_IC2);

    // set CC2 as output to internals
    //timer_ic_set_input(TIM1, TIM_IC2, TIM_CCMR1_CC2S_OUT);

    //OC2REF is set on compare match
    //timer_set_oc_mode(TIM1, TIM_OC2, TIM_OCM_PWM2);

    // set up oc2 interrupt
    nvic_set_priority(NVIC_TIM1_CC_IRQ, NVIC_PRIO_TIMER1);
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    timer_set_dma_on_compare_event(TIM1);
    timer_disable_oc_preload(TIM1, TIM_OC1);
    timer_disable_oc_preload(TIM1, TIM_OC4);

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

    // enable DMA trigger to ch1
    timer_enable_irq(TIM1, TIM_DIER_CC1DE | TIM_DIER_CC4DE);

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
    uint16_t current_compare_value = TIM_CCR2(TIM1);
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

           // led_on();
            //debug_put_uint16((current_compare_value + _US_TO_CLOCKS(10)) - TIM1_CNT ); debug_put_newline();
               video_dma_prepare();

            timer_set_oc_value(TIM1, TIM_OC1, current_compare_value + _US_TO_CLOCKS(14));
            timer_set_oc_value(TIM1, TIM_OC4, current_compare_value + _US_TO_CLOCKS(14));


            // TX: transfer buffer to slave
            //dma_set_memory_address(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, (uint32_t)video_buffer[0]);
            //dma_set_number_of_data(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, VIDEO_BUFFER_WIDTH-1);

            // clear pending dma request from timer
            //timer_enable_irq(TIM2, TIM_DIER_CC2DE);

            // enable dma channel
            //dma_enable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);

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

