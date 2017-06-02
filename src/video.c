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
#include "video_io.h"
#include "video_render.h"
#include "video_spi_dma.h"
#include "video_timer.h"
#include "debug.h"
#include "timeout.h"
#include "led.h"
#include "serial.h"

#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/comparator.h>

static void video_put_uint16(uint8_t *buffer, uint16_t val);
static void video_render_blank(uint16_t line);
static void video_detect_blank_level(void);
static void video_detect_pal_ntsc(void);
uint16_t video_char_buffer_write_ptr;
volatile uint8_t video_stick_data[4];

video_line_t video_line;
//volatile uint16_t video_buffer[2][2][VIDEO_BUFFER_WIDTH/2];
uint8_t video_char_buffer[VIDEO_CHAR_BUFFER_HEIGHT][VIDEO_CHAR_BUFFER_WIDTH];
//volatile uint32_t video_buffer_fill_request;
volatile uint32_t video_unprocessed_frame_count;
volatile uint32_t video_uart_overrun;
volatile uint32_t video_uart_checksum_err;
//volatile uint32_t video_line;
volatile uint32_t video_field;
volatile uint32_t video_buffer_page;
uint8_t video_armed_state;
uint8_t video_mode;


void video_init(void) {
    debug_function_call();

    video_armed_state = 0;
    video_mode = VIDEO_MODE_NTSC;

    // initialize line buffer
    video_line.currently_rendering = 0;
    video_line.active_line = 0;
    video_line.fill_request = VIDEO_BUFFER_FILL_REQUEST_IDLE;

    // clear all buffers
    VIDEO_CLEAR_BUFFER(BLACK, 0);
    VIDEO_CLEAR_BUFFER(BLACK, 1);
    VIDEO_CLEAR_BUFFER(WHITE, 0);
    VIDEO_CLEAR_BUFFER(WHITE, 1);

    // clear char buffer
    for (uint8_t y=0; y<VIDEO_CHAR_BUFFER_HEIGHT; y++) {
        for (uint8_t x=0; x<VIDEO_CHAR_BUFFER_WIDTH; x++) {
            video_char_buffer[y][x] = ' ';
        }
    }

    video_unprocessed_frame_count = 0;
    video_uart_checksum_err = 0;

    video_io_init();
    video_spi_dma_init();
    video_timer_init();

    // detect blank voltage level
    video_detect_blank_level();

    // enable interrupt routine
    video_timer_init_interrupt();

    // detect pal or ntsc
    video_detect_pal_ntsc();

    led_off();

    video_render_init();

}

static void video_detect_pal_ntsc(void) {
    debug_function_call();

    // detect ntsc/pal based on the number of lines
    // timeout >2 pal or nts frames
    uint16_t max_line = 0;

    timeout_set(1000/30 * 3);
    while(!timeout_timed_out()) {
        if (video_line.active_line > max_line) {
            max_line = video_line.active_line;
        }
    }

    debug("video: max_line = "); debug_put_uint16(max_line); debug_put_newline();
    if (max_line  > 600) {
        video_mode = VIDEO_MODE_PAL;
    } else {
        video_mode = VIDEO_MODE_NTSC;
    }
}

static void video_detect_blank_level(void) {
    debug_function_call();

    debug("video: waiting 500ms for cam to be ready\n");
    timeout_delay_ms(500);

    // first: find blank level. start with 0mV and search up to 300mV
    // this should not take too long, let's do this within < 500ms
    // -> use 10 frames
    // -> more than 4800 lines
    uint16_t sync_level = 0;
    uint16_t min_level  = 0;

    for(uint16_t mv = 0; mv < VIDEO_BLANK_LEVEL_DETECTION_MAX_MV; mv += 5) {
        debug("set_mv\n"); debug_flush();
        // set voltage
        video_io_set_dac_value_mv(mv);

        // try to count roughly 100 lines
        // pal: 15625  lines per second = ~6.4ms per 100 lines
        // 6ms counting interval should be good:
        timeout_set(6);
        uint16_t line_counter = 0;
        while(!timeout_timed_out()) {
            // count edges
            if (COMP_CSR(COMP1) & (1<<14)) {
                // rising edge
                while ((COMP_CSR(COMP1) & (1<<14)) && !(timeout_timed_out())) {
                    // wait for falling edge (or timeout)
                }
                line_counter++;
            }
        }

        // try to find min level:
        if ((min_level == 0) && (line_counter > 30)) {
            // take this as min level
            min_level = mv;
        } else if (line_counter > 120) {
            // this exceeds the expected linecount, looks like color burst
            sync_level = min_level + 0.4*(mv - min_level);
            break;
        }
        //debug("tc "); debug_put_uint16(mv); debug("="); debug_put_uint16(line_counter);debug_put_newline();
    }
    debug("video: sync level = "); debug_put_uint16(sync_level); debug_put_newline();

    if (sync_level == 0) {
        // bad detection -> fallback
        sync_level = 100;
    }

    video_io_set_dac_value_mv(sync_level);
}

void video_render_blank(uint16_t line) {
    // no data line, set to inactive
    VIDEO_CLEAR_BUFFER(BLACK, video_line.fill_request);
    VIDEO_CLEAR_BUFFER(WHITE, video_line.fill_request);

    // we have some time left, use it:
    switch (line) {
        default:
            //nothing to do
            break;
#if VIDEO_RENDER_DEBUG_DATA
        case(VIDEO_FIRST_ACTIVE_LINE-1):
            // show some statistics
            // update missing frames
            video_put_uint16((uint8_t*)&video_char_buffer[5+0][30], video_unprocessed_frame_count);
            video_char_buffer[5+0][29] = 'M';
            break;

        case(VIDEO_FIRST_ACTIVE_LINE-2):
            //uart overrun counter
            video_put_uint16((uint8_t*)&video_char_buffer[5+1][30], video_uart_overrun);
            video_char_buffer[5+1][29] = 'U';
            break;

        case(VIDEO_FIRST_ACTIVE_LINE-3):
            //uart checksum err counter
            video_put_uint16((uint8_t*)&video_char_buffer[5+2][30], video_uart_checksum_err);
            video_char_buffer[5+2][29] = 'C';
            break;


        case(VIDEO_FIRST_ACTIVE_LINE-4):
            // video mdoe
            if (video_mode == VIDEO_MODE_NTSC) {
                strcpy(&video_char_buffer[5+3][30], "NTSC");
            }else{
                strcpy(&video_char_buffer[5+3][30], "PAL");
            }
            break;
#endif
        case(VIDEO_FIRST_ACTIVE_LINE-5):
            break;
    }
}

void video_main_loop(void) {
    uint32_t page_to_fill;

    // endless loop
    while (1) {
        // store current page to render

        // is there a new line request?
        if (video_line.fill_request != VIDEO_BUFFER_FILL_REQUEST_IDLE) {
            #if VIDEO_DEBUG_ODD_EVEN
            led_set(video_line.active_line & 1);
            #endif


            page_to_fill = video_line.fill_request;

            // active or blank line?
            if ((video_line.active_line < VIDEO_FIRST_ACTIVE_LINE) || (video_line.active_line > VIDEO_LAST_ACTIVE_LINE)){
                // inactive lines, render lank line and do some processing
                video_render_blank(video_line.active_line);
            }else{
                //visible line -> render data
                uint16_t visible_line = video_line.active_line - VIDEO_FIRST_ACTIVE_LINE;

#if  VIDEO_RENDER_ADCVAL
                if (0) {
#else
                if (((video_armed_state & (1<<3)) == 0) &&
                     (visible_line >= VIDEO_START_LINE_ANIMATION) &&
                     (visible_line <= VIDEO_END_LINE_ANIMATION)
                   ) {
#endif
                    // never armed and inside ani window -> show animation
                    video_render_animation(page_to_fill, visible_line);
                } else {
                    video_render_text(page_to_fill, visible_line);

                    // render some more data when armed
                    if (video_armed_state & (1<<2)) {
                        // copter is armed
                        video_render_overlay_sticks(page_to_fill, visible_line);
                        video_render_pilot_logo(page_to_fill, visible_line);
                    }
                }
            }

            // make sure the last 4 bytes are always disabled
            // why four bytes? fifo ? todo: check this
            video_line.buffer[BLACK][page_to_fill][VIDEO_BUFFER_WIDTH/2-2] = 0;
            video_line.buffer[BLACK][page_to_fill][VIDEO_BUFFER_WIDTH/2-1] = 0;
            video_line.buffer[WHITE][page_to_fill][VIDEO_BUFFER_WIDTH/2-2] = 0;
            video_line.buffer[WHITE][page_to_fill][VIDEO_BUFFER_WIDTH/2-1] = 0;


#if VIDEO_RENDER_ADCVAL
            // 0 ... 4095 -> / 16 = 0...255
            static int16_t video_avg = 0;
            static int32_t video_avg_frame_tmp = 0;
            static int16_t video_avg_frame_cnt = 0;
            static int16_t video_avg_frame = 1;
            int16_t aval = adc_get_value();

            if (video_line.active_line == VIDEO_FIRST_ACTIVE_LINE) {
                video_avg_frame = video_avg_frame_tmp / video_avg_frame_cnt;
                video_avg_frame_tmp = 0;
                video_avg_frame_cnt = 0;
            }
            video_avg_frame_cnt++;

            // low pass filter: y[i] := y[i-1] + alpha * (x[i] - y[i-1])
            video_avg = video_avg  + (aval - video_avg)/2;
            video_avg_frame_tmp += video_avg;

            aval =  video_avg/ 8;

            if (aval > 500) aval = 500;

            uint16_t t = (1<<(15-(aval%16)));
            video_line.buffer[WHITE][page_to_fill][aval/16] = (t<<8) | (t>>8);

            aval = video_avg_frame / 8;
            t = (1<<(15-(aval%16)));
            video_line.buffer[WHITE][page_to_fill][aval/16] |= (t<<8) | (t>>8);

            if (video_avg > video_avg_frame) {
                video_line.buffer[WHITE][page_to_fill][VIDEO_BUFFER_WIDTH/2/2-1] |= 0x1800;
                video_line.buffer[BLACK][page_to_fill][VIDEO_BUFFER_WIDTH/2/2-1] |= 0x00FF;
            }
#endif

            // if the frame was sent in between, increment unprocessed frame counter
            if (video_line.fill_request != page_to_fill) {
                video_unprocessed_frame_count++;
            }

            // clear request
            video_line.fill_request = VIDEO_BUFFER_FILL_REQUEST_IDLE;

        }
        // process incoming data
        serial_process();
    }
}


// output an unsigned 16-bit number to uart
static void video_put_uint16(uint8_t *buffer, uint16_t val) {
    uint8_t tmp;
    uint8_t l = 0;
    uint32_t mul;
    // loop unrolling is better(no int16 arithmetic)
    for (mul = 10000; mul>0; mul = mul/10) {
        l = 0;
        tmp = '0';
        while (val >= mul) {
                val -= mul;
                tmp++;
                l = 1;
        }
        if ((l == 0) && (tmp == '0') && (mul!=1)) {
                *buffer++ = '0';
        } else {
                *buffer++ = tmp;
        }
    }
}

#if 0

#include "config.h"
#include "macros.h"
#include "debug.h"
#include "delay.h"
#include "clocksource.h"
#include "led.h"
#include "logo.h"
#include "font.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/comparator.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/syscfg.h>

#include <libopencm3/cm3/scb.h>

#include <libopencmsis/core_cm3.h>

#define VIDEO_DEBUG_DMA 0
#define VIDEO_DEBUG_DURATION_TEXTLINE 1
#define VIDEO_DEBUG_DURATION_ANIMATION 1

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

#define VIDEO_BUFFER_WIDTH (2*38) //66 // max should be ~68
//volatile uint8_t video_buffer[2][2][VIDEO_BUFFER_WIDTH+1];
volatile uint16_t video_buffer[2][2][VIDEO_BUFFER_WIDTH/2];

#define VIDEO_CHAR_BUFFER_WIDTH  35  // THIS SHALL NEVER EXCEED VIDEO_BUFFER_WIDTH/2-3 !
#define VIDEO_CHAR_BUFFER_HEIGHT 13
uint8_t video_char_buffer[VIDEO_CHAR_BUFFER_HEIGHT][VIDEO_CHAR_BUFFER_WIDTH];

volatile uint32_t video_dbg;
volatile uint32_t video_line;
volatile uint32_t video_field;
volatile uint16_t video_sync_last_compare_value;

volatile uint32_t video_spi_cr_trigger;
volatile uint32_t video_spi_cr_trigger_b;

volatile uint32_t video_unprocessed_frame_sent;
volatile uint32_t video_unprocessed_frame_count;

volatile uint32_t video_buffer_fill_request;
volatile uint32_t video_buffer_page;
#define VIDEO_BUFFER_FILL_REQUEST_IDLE 3

//NTSC
#define VIDEO_FIRST_ACTIVE_LINE 40
#define VIDEO_LAST_ACTIVE_LINE  516
#define VIDEO_CENTER_ACTIVE_LINE ((VIDEO_LAST_ACTIVE_LINE - VIDEO_FIRST_ACTIVE_LINE) / 2)

#define VIDEO_START_LINE_ANIMATION (VIDEO_CENTER_ACTIVE_LINE - LOGO_HEIGHT/2)
#define VIDEO_END_LINE_ANIMATION (VIDEO_CENTER_ACTIVE_LINE + LOGO_HEIGHT/2)

// void TIM1_CC_IRQHandler(void) {
void TIM1_CC_IRQHandler(void) {
    if (TIMER_GET_FLAG(TIM1, TIM_SR_CC4IF)) {
        TIMER_CLEAR_FLAG(TIM1, TIM_SR_CC4IF);
        if (VIDEO_DEBUG_DMA) led_toggle();
        if (VIDEO_DEBUG_DMA) led_toggle();
    }
}

static const uint32_t video_cos_table[90] = {
0x80,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,0x7f,
0x7e,0x7e,0x7e,0x7d,0x7d,0x7c,0x7c,0x7b,
0x7b,0x7a,0x79,0x79,0x78,0x77,0x76,0x75,
0x74,0x74,0x73,0x72,0x71,0x6f,0x6e,0x6d,
0x6c,0x6b,0x6a,0x68,0x67,0x66,0x64,0x63,
0x62,0x60,0x5f,0x5d,0x5c,0x5a,0x58,0x57,
0x55,0x54,0x52,0x50,0x4e,0x4d,0x4b,0x49,
0x47,0x45,0x43,0x41,0x40,0x3e,0x3c,0x3a,
0x38,0x36,0x34,0x32,0x30,0x2d,0x2b,0x29,
0x27,0x25,0x23,0x21,0x1f,0x1c,0x1a,0x18,
0x16,0x14,0x11,0xf,0xd,0xb,0x9,0x6,
0x4,0x2};

void video_render_text(uint16_t visible_line) {
    if (VIDEO_DEBUG_DURATION_TEXTLINE) led_on();
    uint32_t line    = visible_line / 2; // even and odd lines get the same data

    uint8_t text_row = (line / 18);
    uint8_t font_row = line % 18;

    // fill empty
    /*uint32_t *p = &video_buffer[0][video_buffer_fill_request][0];
    uint32_t *q = &video_buffer[1][video_buffer_fill_request][0];
    for(uint8_t i=0; i<VIDEO_BUFFER_WIDTH/4; i++){
        *p++ = 0;
        *q++ = 0;
    }*/
    //memset(&video_buffer[0][video_buffer_fill_request][0], 0, VIDEO_BUFFER_WIDTH);
    //memset(&video_buffer[1][video_buffer_fill_request][0], 0, VIDEO_BUFFER_WIDTH);

    if (text_row >= VIDEO_CHAR_BUFFER_HEIGHT){
        // no data
        return;
    }

//    for (uint8_t color = 0; color < 1; color++){

        uint8_t *video_buffer_ptr[2];
        video_buffer_ptr[0] = (uint8_t*) &video_buffer[0][video_buffer_fill_request][0];
        video_buffer_ptr[1] = (uint8_t*) &video_buffer[1][video_buffer_fill_request][0];

        // white data has to be shifted one byte with the first byte cleared
        *video_buffer_ptr[0]++ = 0x00;


        uint8_t *char_ptr = &video_char_buffer[text_row][0];
        uint8_t tmp=0;

        uint8_t *font_ptr_row = (uint8_t*)&font_data[0][font_row][0];

        for (uint32_t text_col = 0; text_col < VIDEO_CHAR_BUFFER_WIDTH; text_col++) {
            //use manual loop unrolling here, subtract char ptr and add etc

                uint32_t index = *char_ptr++; //video_char_buffer[text_row][text_col]; //(*char_ptr++);

                // fetch ptr to font data
                //uint16_t index = video_char_buffer[text_row][text_col]*3;
                uint8_t *font_ptr = font_ptr_row + index*2;

                *video_buffer_ptr[0]++ = *font_ptr++;
                *video_buffer_ptr[0]++ = *font_ptr++;

                // get black font data:
                font_ptr += sizeof(font_data[0])-2; //(uint8_t*)&font_data[1][font_row][index*2];

                *video_buffer_ptr[1]++ = (*font_ptr++);
                *video_buffer_ptr[1]++ = (*font_ptr++);

                font_ptr -= 4;
        }
  //  }

    if (VIDEO_DEBUG_DURATION_TEXTLINE) led_off();
}


uint32_t ani_count=0;
uint32_t ani_dir=0;
uint8_t div=0;

void video_render_ani(uint16_t visible_line) {
    if (VIDEO_DEBUG_DURATION_ANIMATION) led_on();

    uint32_t data = 0;
    uint32_t logo_start_line = VIDEO_CENTER_ACTIVE_LINE - LOGO_HEIGHT/2;
    uint32_t logo_end_line   = VIDEO_CENTER_ACTIVE_LINE + LOGO_HEIGHT/2;

    uint32_t logo_offset_x;
    if (LOGO_WIDTH <= VIDEO_BUFFER_WIDTH) {
        logo_offset_x = 0;
    }else{
        logo_offset_x = VIDEO_BUFFER_WIDTH/2 - LOGO_WIDTH/8/2;
    }
    uint32_t logo_offset;
    uint8_t *logo_ptr;
    uint8_t *framebuffer_ptr;
    uint8_t *video_buffer_ptr;
    uint8_t *video_buffer_end_ptr;



    if (visible_line == VIDEO_CENTER_ACTIVE_LINE) {
        ani_count+=4;
        if (ani_count >=360) {
            ani_count -= 360;
        }
    }
    //ani_count = 170;

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
    uint32_t line    = visible_line + 2;

    logo_start_line = VIDEO_CENTER_ACTIVE_LINE - (scale * LOGO_HEIGHT/2) / 128;
    logo_end_line   = VIDEO_CENTER_ACTIVE_LINE + (scale * LOGO_HEIGHT/2) / 128;

    logo_offset = (line - logo_start_line) * 128 / scale * (LOGO_WIDTH/8);

    if (!ani_dir) {
        // flip on neg rotation
        logo_offset = LOGO_HEIGHT*LOGO_WIDTH/8 - logo_offset;
    }

    uint32_t max_len = min((LOGO_WIDTH/8), (VIDEO_BUFFER_WIDTH - logo_offset_x));
#if 1

    // fill the next line of data now:
    for(uint8_t color = 0; color < 2; color++){

        //for(uint8_t i=0; i<VIDEO_BUFFER_WIDTH/2; i++){
//                video_buffer[col][video_buffer_fill_request][i] = 0x0000;
//            }

        // [0] = white, [1] = black data
        // fetch correct buffer ptr
        video_buffer_ptr     = (uint8_t*) &video_buffer[color][video_buffer_fill_request][0];
        video_buffer_end_ptr = video_buffer_ptr + VIDEO_BUFFER_WIDTH - 4; //(uint8_t*) &video_buffer[col][video_buffer_fill_request][VIDEO_BUFFER_WIDTH/2-2];

        logo_ptr = &logo_data[color][logo_offset];

        if (!color) {
            // white data has to be shifted one byte with the first byte cleared
            *video_buffer_ptr++ = 0x00;
        }

        if ((line > logo_start_line) && (line < logo_end_line)){

            /*for (uint32_t i = 0; i < logo_offset_x; i++){
                *video_buffer_ptr++ = 0x0;
            }*/

            memset(video_buffer_ptr, 0, logo_offset_x);
            video_buffer_ptr+=logo_offset_x;
            memcpy(video_buffer_ptr, logo_ptr, max_len);
            video_buffer_ptr+=max_len;

/*
            for(uint32_t i = 0; i < max_len; i++){
               *video_buffer_ptr++ = *logo_ptr++;
            }*/

            while(video_buffer_ptr < video_buffer_end_ptr){
                *video_buffer_ptr++ = 0x0;
            }

            //video_buffer[col][video_buffer_fill_request][40-1] = 0xfF;

        }else{
            // no image data region
            /*for(uint32_t x = 0; x < VIDEO_BUFFER_WIDTH; x++){
               *video_buffer_ptr++ = 0x0;
            }*/
            memset(&video_buffer[color][video_buffer_fill_request][0], 0x0000, VIDEO_BUFFER_WIDTH);

        }
    }
#endif
    if (VIDEO_DEBUG_DURATION_ANIMATION) led_off();
}

void video_init(void) {
    debug_function_call();

    // uint16_t tmp = 0;

    for(uint8_t col = 0; col < 2; col++) {
        for(uint8_t idx = 0; idx < 2; idx++) {
            for(uint8_t i=0; i<VIDEO_BUFFER_WIDTH/2; i++) {
                video_buffer[col][idx][i] = 0; //x3030;
            }
        }
    }

    video_init_rcc();
    video_init_gpio();

    //video_init_pendsv();

    video_init_dac();

    video_init_spi();
    video_init_spi_dma();

    video_set_dac_value_mv(180);

    video_init_timer();

    video_init_comparator();
    video_init_comparator_interrupt();


    /*for(uint32_t i=VIDEO_BUFFER_WIDTH-4; i<VIDEO_BUFFER_WIDTH-3; i++){
        video_buffer[0][i] = 0xFF;
    }*/

    video_buffer_fill_request = VIDEO_BUFFER_FILL_REQUEST_IDLE;

    led_off();


    for (uint8_t y=0; y<VIDEO_CHAR_BUFFER_HEIGHT; y++) {
        for (uint8_t x=0; x<VIDEO_CHAR_BUFFER_WIDTH; x++) {
            video_char_buffer[y][x] = ' ';
        }
    }

    for (uint8_t x=0; x<VIDEO_CHAR_BUFFER_WIDTH; x++) {
        video_char_buffer[VIDEO_CHAR_BUFFER_HEIGHT-1][x] = '0' + x;
    }



    video_unprocessed_frame_count = 0;

  //  video_dma_prepare();


/*while(1){

   //video_dma_prepare();


 //spi_enable_tx_dma(SPI2);
 //spi_enable_tx_dma(SPI1);
 delay_us(10*1000);
}*/


// TRICK: data is shifted one byte -> dma is triggered 1 byte offset -> no dual sram access requests
// white data: [byte0] [byte1]....
// blakc data: [DUMMY] [byte0] [byte1] ....
/*for(uint8_t idx = 0; idx < 2; idx++) {
        video_buffer[0][idx][0] = 0x8100;
        video_buffer[1][idx][0] = 0x0081;
}
while(1);
*/
    uint16_t l=0;
    uint32_t current_frame;

while(1){
    // show some stats:
    if (video_line == VIDEO_FIRST_ACTIVE_LINE-1){
            //led_on();
            video_put_uint16(&video_char_buffer[1][0], video_unprocessed_frame_count);
            //led_off();
    }

    if (video_buffer_fill_request != VIDEO_BUFFER_FILL_REQUEST_IDLE) {
        current_frame = video_buffer_fill_request;


#if 1



        if ((video_line < VIDEO_FIRST_ACTIVE_LINE) || (video_line > VIDEO_LAST_ACTIVE_LINE)){
            // no data line, set to unactive
            memset(&video_buffer[0][video_buffer_fill_request][0], 0, VIDEO_BUFFER_WIDTH);
            memset(&video_buffer[1][video_buffer_fill_request][0], 0, VIDEO_BUFFER_WIDTH);
        }else{
            //visible line number:
            uint16_t visible_line = video_line - VIDEO_FIRST_ACTIVE_LINE;

            if ((visible_line > VIDEO_START_LINE_ANIMATION) && (visible_line < VIDEO_END_LINE_ANIMATION)) {
                video_render_ani(visible_line);
            }else{
                video_render_text(visible_line);
            }
        }



        // make sure the last 4 bytes are always disabled
        // why four bytes? fifo ? todo: check this
        video_buffer[0][video_buffer_fill_request][VIDEO_BUFFER_WIDTH/2-2] = 0;
        video_buffer[0][video_buffer_fill_request][VIDEO_BUFFER_WIDTH/2-1] = 0;
        video_buffer[1][video_buffer_fill_request][VIDEO_BUFFER_WIDTH/2-2] = 0;
        video_buffer[1][video_buffer_fill_request][VIDEO_BUFFER_WIDTH/2-1] = 0;
#else
        if ((video_line < 23) || (video_line > 240*2)) {
            // no video allowed
            memset(&video_buffer[0][video_buffer_fill_request][0], 0x0000, VIDEO_BUFFER_WIDTH);
            memset(&video_buffer[1][video_buffer_fill_request][0], 0x0000, VIDEO_BUFFER_WIDTH);

        }else if ((video_line > 18*5*2) && (video_line < 18*6*2+200)){
            video_render_ani();
        }else{
            // render text
            video_render_text();
        }

        //if ((video_line > 10) && (video_line < 300)) video_buffer[0][video_buffer_fill_request][VIDEO_BUFFER_WIDTH/2-1] = 0x00FF;

    //video_buffer[0][video_buffer_fill_request][VIDEO_BUFFER_WIDTH/2-1] = 0x00FF;
#endif

        // if the frame was sent in between, increment unprocessed frame counter
        if (video_buffer_fill_request != current_frame) {
            video_unprocessed_frame_count++;
        }

        // clear request
        video_buffer_fill_request = VIDEO_BUFFER_FILL_REQUEST_IDLE;
        //debug("ok\n");
    }

};
    //timer_clear_flag(TIM1, TIM_SR_CC2IF);
    //timer_set_oc_value(TIM1, TIM_OC2, 0x2F00);
    //timer_enable_irq(TIM1, TIM_DIER_CC2DE | TIM_DIER_CC2IE);


    video_dma_prepare(0);

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
    //if (EXTI_EMR){
    //        debug("EXIT: 0x");
     //       debug_put_hex32(EXTI_EMR);
      //      debug_put_newline();
       //     EXTI_EMR = 0;
    //}
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

    // prepare to send dma spi data
    DMA_SET_MEMORY_ADDRES_NOCHECK(VIDEO_DMA_BLACK, DMA_CHANNEL5, &(video_buffer[1][video_buffer_page]));
    DMA_SET_NUMBER_OF_DATA(VIDEO_DMA_BLACK, DMA_CHANNEL5, VIDEO_BUFFER_WIDTH/2);

    // clear all dma if
    // NOT NECESSARY? move to define if enable is necc...
   //  dma_clear_interrupt_flags(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_TCIF);


    // disable timer match in order to clear pending timer triggers
    //TIMER_CLEAR_DMA_ON_COMPARE_EVENT(TIM1);
    TIMER_CLEAR_FLAG(TIM1, TIM_SR_CC1IF | TIM_SR_CC4IF);
    TIMER_DISABLE_IRQ(TIM1, TIM_DIER_CC1DE | TIM_DIER_CC4DE);

    // prepare next page:
    video_buffer_page         = 1 - video_buffer_page;
    video_buffer_fill_request = video_buffer_page;

    return;
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
    DMA_SET_MEMORY_ADDRES_NOCHECK(VIDEO_DMA_WHITE, DMA_CHANNEL3, &(video_buffer[0][video_buffer_page]));
    DMA_SET_NUMBER_OF_DATA(VIDEO_DMA_WHITE, DMA_CHANNEL3, VIDEO_BUFFER_WIDTH/2);

    // clear all dma if
    // NOT NECESSARY? move to define if enable is necc...
   //  dma_clear_interrupt_flags(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE, DMA_TCIF);

}

static void video_init_spi_dma(void) {
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


    // ***********************************************************

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
    dma_set_memory_address(VIDEO_DMA_WHITE, DMA_CHANNEL3, (uint32_t)&(video_buffer[0][0]));

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
    dma_set_memory_address(VIDEO_DMA_BLACK, DMA_CHANNEL5, (uint32_t)&(video_buffer[1][0]));

    // write full len
    dma_set_number_of_data(VIDEO_DMA_BLACK, DMA_CHANNEL5, VIDEO_BUFFER_WIDTH/2);

    // very high prio
    dma_set_priority(VIDEO_DMA_BLACK, DMA_CHANNEL5, DMA_CCR_PL_VERY_HIGH);

    // enable tx complete int
    dma_enable_transfer_complete_interrupt(VIDEO_DMA_BLACK, DMA_CHANNEL5);


    // ***********************************************************

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
    video_spi_cr_trigger_b = SPI_CR2(VIDEO_SPI_BLACK) | SPI_CR2_TXDMAEN;
    dma_set_memory_address(VIDEO_DMA_BLACK, DMA_CHANNEL4, (uint32_t)&(video_spi_cr_trigger_b));

    // single word write
    dma_set_number_of_data(VIDEO_DMA_BLACK, DMA_CHANNEL4, 1);

    // very high prio
    dma_set_priority(VIDEO_DMA_BLACK, DMA_CHANNEL4, DMA_CCR_PL_VERY_HIGH);
}

void video_dma_prepare() {


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

static void video_init_gpio(void) {
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
    //timer_enable_irq(TIM1, TIM_DIER_CC1DE | TIM_DIER_CC4DE);

    // DMA on compare event
    timer_set_dma_on_compare_event(TIM1);


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
            video_field = VIDEO_SECOND_FIELD;
        }
    } else  {
        // rising edge -> this was measuring the a sync part
        if (pulse_len < VIDEO_SYNC_SHORT_MAX) {
            // all short sync pulses are shortsyncs
            // new (half)frame -> init line counter
            // video start at sync 20 on even and odd, as we count different
            // init to the right values here:
            if (video_field == VIDEO_FIELD_ODD) {
                // odd
                video_line = 15;
            } else {
                // even
                video_line = 14;
            }


        } else if (pulse_len < VIDEO_SYNC_HSYNC_MAX) {
            // this is longer than a short sync and not a broad sync

          //  dma_disable_channel(VIDEO_DMA_WHITE, VIDEO_DMA_CHANNEL_WHITE);
            // video_dma_trigger();

            //debug_put_uint16((current_compare_value + _US_TO_CLOCKS(10)) - TIM1_CNT ); debug_put_newline();
            current_compare_value += _US_TO_CLOCKS(6+0);

            //uint32_t ccval = current_compare_value +100; // _US_TO_CLOCKS(15);
            TIM_CCR1(TIM1) = current_compare_value;
            TIM_CCR4(TIM1) = current_compare_value + 2*16; // correct for offset by different dma access time

            if (VIDEO_DEBUG_DMA) led_on();

            TIMER_ENABLE_IRQ(TIM1, TIM_DIER_CC1DE | TIM_DIER_CC4DE | TIM_DIER_CC4IE);

            if (VIDEO_DEBUG_DMA) TIMER_ENABLE_IRQ(TIM1, TIM_DIER_CC4IE);

            // enable dma channel, this was set up during the dma end of tx int
            DMA_ENABLE_CHANNEL(VIDEO_DMA_WHITE, DMA_CHANNEL2);
            DMA_ENABLE_CHANNEL(VIDEO_DMA_WHITE, DMA_CHANNEL3);
            DMA_ENABLE_CHANNEL(VIDEO_DMA_BLACK, DMA_CHANNEL4);
            DMA_ENABLE_CHANNEL(VIDEO_DMA_BLACK, DMA_CHANNEL5);

            if (VIDEO_DEBUG_DMA) led_off();

            // increment video field
            video_line += 2;

            // prepare for the next field
            video_field = VIDEO_FIRST_FIELD;
        } else {
            // this is a broad sync
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


// output an unsigned 16-bit number to uart
void video_put_uint16(uint8_t *buffer, uint16_t val) {
    uint8_t tmp;
    uint8_t l = 0;
    uint32_t mul;
    // loop unrolling is better(no int16 arithmetic)
    for (mul = 10000; mul>0; mul = mul/10) {
        l = 0;
        tmp = '0';
        while (val >= mul) {
                val -= mul;
                tmp++;
                l = 1;
        }
        if ((l == 0) && (tmp == '0') && (mul!=1)) {
                *buffer++ = '0';
        } else {
                *buffer++ = tmp;
        }
    }
}

#endif
