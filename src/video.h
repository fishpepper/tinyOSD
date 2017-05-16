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

#ifndef VIDEO_H_
#define VIDEO_H_

#define VIDEO_DEBUG_DMA 0
#define VIDEO_DEBUG_DURATION_TEXTLINE 0
#define VIDEO_DEBUG_DURATION_ANIMATION 0

#include <stdint.h>
#define VIDEO_START_LINE_ANIMATION (VIDEO_CENTER_ACTIVE_LINE - LOGO_HEIGHT/2)
#define VIDEO_END_LINE_ANIMATION (VIDEO_CENTER_ACTIVE_LINE + LOGO_HEIGHT/2)

#define VIDEO_RENDER_STICK_SIZE       96
#define VIDEO_RENDER_STICK_SIZE_X     96
#define VIDEO_RENDER_STICK_SIZE_Y    128
#define VIDEO_RENDER_STICK_POS_X   (3*8)
#define VIDEO_START_LINE_STICKS   (8*35)
#define VIDEO_END_LINE_STICKS  (VIDEO_START_LINE_STICKS + VIDEO_RENDER_STICK_SIZE)

void video_init(void);
void video_main_loop(void);

#define VIDEO_BUFFER_WIDTH (2*38) //66 // max should be ~68
#define VIDEO_CHAR_BUFFER_WIDTH  35  // THIS SHALL NEVER EXCEED VIDEO_BUFFER_WIDTH/2-3 !
#define VIDEO_CHAR_BUFFER_HEIGHT 13

#define VIDEO_BUFFER_FILL_REQUEST_IDLE 3

#define WHITE 0
#define BLACK 1

//NTSC
#define VIDEO_FIRST_ACTIVE_LINE 40
#define VIDEO_LAST_ACTIVE_LINE  516
#define VIDEO_CENTER_ACTIVE_LINE ((VIDEO_LAST_ACTIVE_LINE - VIDEO_FIRST_ACTIVE_LINE) / 2)

extern uint8_t video_char_buffer[VIDEO_CHAR_BUFFER_HEIGHT][VIDEO_CHAR_BUFFER_WIDTH];
extern uint16_t video_char_buffer_write_ptr;

#define VIDEO_CLEAR_BUFFER(__col, __idx) { memset((void *)&video_line.buffer[__col][__idx][0], 0, (VIDEO_BUFFER_WIDTH/2)*2); }
#define VIDEO_SET_BUFFER(__col, __idx) { memset((void *)&video_line.buffer[__col][__idx][0], 0xFFFF, (VIDEO_BUFFER_WIDTH/2)*2); }

extern volatile uint8_t video_stick_data[4];

typedef struct {
    volatile uint16_t buffer[2][2][VIDEO_BUFFER_WIDTH/2];

    // current line on screen
    volatile uint32_t active_line;

    // page currently beeing drawn
    volatile uint32_t currently_rendering;

    // pending fill requests?
    volatile uint32_t fill_request;
} video_line_t;

extern video_line_t video_line;

//extern volatile uint16_t video_buffer[2][2][VIDEO_BUFFER_WIDTH/2];
//extern volatile uint32_t video_buffer_fill_request;
extern volatile uint32_t video_unprocessed_frame_count;
extern volatile uint32_t video_uart_overrun;
extern volatile uint32_t video_uart_checksum_err;
//extern volatile uint32_t video_line;
extern volatile uint32_t video_field;
//extern volatile uint32_t video_buffer_page;

#define TIMER_CLOCKS_PER_US                      (48000000 / 1000000)
#define _US_TO_CLOCKS(__us)                      ((uint32_t)((__us) * TIMER_CLOCKS_PER_US))

#define VIDEO_LINE_LEN            63.556  // us
#define VIDEO_SYNC_SHORT           2.000  // us
#define VIDEO_SYNC_HSYNC           4.700  // us
#define VIDEO_FIELD_ODD            1
#define VIDEO_FIELD_EVEN           (1-VIDEO_FIELD_ODD)
#define VIDEO_FIRST_FIELD          VIDEO_FIELD_ODD     // ODD (NTSC)
#define VIDEO_SECOND_FIELD         (1-(VIDEO_FIRST_FIELD))

// timing for high level (from shortest to longest period)
// (1) [HI] BROAD SYNC: t ~ VIDEO_SYNC_HSYNC
// (2) [HI] VSYNC+DATA: t ~ (VIDEO_LINE_LEN / 2) - VIDEO_SYNC_HSYNC
// (3) [HI] SHORT SYNC: t ~ (VIDEO_LINE_LEN / 2) - VIDEO_SYNC_SHORT
// (4) [HI] VIDEO DATA: t ~ VIDEO_LINE_LEN - VIDEO_SYNC_HSYNC
//
#define VIDEO_SYNC_HI_BROAD     (VIDEO_SYNC_HSYNC)
#define VIDEO_SYNC_HI_VSYNC     ((VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_HSYNC)
#define VIDEO_SYNC_HI_SHORT     ((VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_SHORT)
#define VIDEO_SYNC_HI_DATA      (VIDEO_LINE_LEN)
//
// -> valid vsync = .... (1)---[xxx(2)xxx]---(3)------(4)
//
#define VIDEO_SYNC_VSYNC_MIN        _US_TO_CLOCKS(VIDEO_SYNC_HI_VSYNC - (VIDEO_SYNC_HI_VSYNC - VIDEO_SYNC_HI_BROAD)/2.0)
#define VIDEO_SYNC_VSYNC_MAX        _US_TO_CLOCKS(VIDEO_SYNC_HI_VSYNC + (VIDEO_SYNC_HI_SHORT - VIDEO_SYNC_HI_VSYNC)/2.0)

// timing for low level (from shortest to longest period)
// (1) [LO] SHORT SYNC: t ~ 2.0us
// (2) [LO] HSYNC     : t ~ 4.7us
// (3) [LO] BROAD     : t ~ (VIDEO_LINE_LEN / 2) - VIDEO_SYNC_HSYNC
//
//
// short sync =  (1)xxx]---(2)------(3)
//
#define VIDEO_SYNC_SHORT_MIN    _US_TO_CLOCKS(0)
#define VIDEO_SYNC_SHORT_MAX    _US_TO_CLOCKS(VIDEO_SYNC_SHORT +  (VIDEO_SYNC_HSYNC - VIDEO_SYNC_SHORT)/2.0)
//
// hsync      =  (1)---[xxx(2)xxx]---(3)
//
#define VIDEO_SYNC_HSYNC_MIN    _US_TO_CLOCKS(VIDEO_SYNC_HSYNC - (VIDEO_SYNC_HSYNC - VIDEO_SYNC_SHORT)/2.0)
#define VIDEO_SYNC_HSYNC_MAX    _US_TO_CLOCKS(VIDEO_SYNC_HSYNC + (VIDEO_SYNC_LO_BROAD - VIDEO_SYNC_HSYNC)/2.0)
//
// broad      = (1)------(2)---[xxx(3)]
//
#define VIDEO_SYNC_LO_BROAD       (VIDEO_LINE_LEN / 2.0) - VIDEO_SYNC_HSYNC



#endif  // VIDEO_H_
