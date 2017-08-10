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
#include "macros.h"
#include "led.h"
#include "adc.h"
#include "serial.h"

#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/comparator.h>


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
uint16_t video_min_level;

volatile uint16_t video_stats_line_start;
volatile uint16_t video_stats_line_usage_min;
volatile uint16_t video_stats_line_usage_max;

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
        // set voltage
        video_io_set_dac_value_mv(mv);

        // try to count roughly 100 lines
        // pal: 15625  lines per second = ~6.4ms per 100 lines
        // 6ms counting interval should be good:
        timeout_set(6);
        uint16_t line_counter = 0;

        while(!timeout_timed_out()) {
            // count edges
            if (COMP_CSR(VIDEO_COMP) & (COMP2_CSR_COMPOUT)) {
                // rising edge
                while ((COMP_CSR(VIDEO_COMP) & (COMP2_CSR_COMPOUT)) && !(timeout_timed_out())) {
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
        debug("tc "); debug_put_uint16(mv); debug("="); debug_put_uint16(line_counter);debug_put_newline();
    }
    video_min_level = min_level;
    debug("video: min  level = "); debug_put_uint16(video_min_level); debug_put_newline();
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
                strcpy((char *)&video_char_buffer[5+3][30], "NTSC");
            }else{
                strcpy((char *)&video_char_buffer[5+3][30], "PAL");
            }
            break;

        case(VIDEO_FIRST_ACTIVE_LINE-5):
            // stats: line length
            {
            uint8_t *buf = &video_char_buffer[5+4][29];
            //video_put_uint8(buf+0, video_stats_line_usage_min);
            strcpy((char *)buf, "CPU");
            video_put_uint8(buf+3, video_stats_line_usage_max);
            //*(buf+3) = '-';

            // reset per frame statistics
            video_stats_line_usage_min = 0xFFFF;
            video_stats_line_usage_max = 0x0000;
            }
            break;

        case(VIDEO_FIRST_ACTIVE_LINE-6):
            // stats: line length
            {
            uint8_t *buf = &video_char_buffer[5+5][29];
            //video_put_uint8(buf+0, video_stats_line_usage_min);
            strcpy((char *)buf, "ARM");
            video_put_uint8(buf+3, video_armed_state);
            }
            break;



#endif
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

#if VIDEO_RENDER_BARS
                // render debug bars:
                video_render_grey_bars(page_to_fill, visible_line);

#elif VIDEO_RENDER_ADCVAL
                // render adc data
#else
                // normal ui
                //video_armed_state |= (1<<3);
                if (((video_armed_state & (1<<1)) == 0) &&
                     (visible_line >= VIDEO_START_LINE_ANIMATION) &&
                     (visible_line < VIDEO_END_LINE_ANIMATION)
                   ) {
                    // never armed and inside ani window -> show animation
                    video_render_animation(page_to_fill, visible_line);
                } else {

                    video_render_text(page_to_fill, visible_line);

                    // render some more data when armed
                    if (video_armed_state & (1<<0)) {
                        // copter is armed
                        video_render_overlay_sticks(page_to_fill, visible_line);
                        video_render_pilot_logo(page_to_fill, visible_line);
                        video_render_overlay_spectrum(page_to_fill, visible_line);
                    }

                }
#endif
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

            // do some statistics, calculate how much of the line we used for rendering:
            uint16_t ts_rendering_done = TIM_CNT(TIM1);

            // keep track of rendering time:
            uint16_t last_duration    = ts_rendering_done - video_stats_line_start;
            uint32_t usage = (100 * (uint32_t)last_duration) /  _US_TO_CLOCKS(VIDEO_LINE_LEN);

            video_stats_line_usage_min  = min(video_stats_line_usage_min, usage);
            video_stats_line_usage_max  = min(100, max(video_stats_line_usage_max, usage));

        }
        // process incoming data
        serial_process();
    }
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


// output an unsigned 8-bit number to uart
void video_put_uint8(uint8_t *buffer, uint8_t val) {
    uint8_t tmp;
    uint8_t l = 0;
    uint32_t mul;
    // loop unrolling is better(no int16 arithmetic)
    for (mul = 100; mul>0; mul = mul/10) {
        l = 0;
        tmp = '0';
        while (val >= mul) {
                val -= mul;
                tmp++;
                l = 1;
        }
        if ((l == 0) && (tmp == '0') && (mul!=1)) {
                *buffer++ = ' ';
        } else {
                *buffer++ = tmp;
        }
    }
}
