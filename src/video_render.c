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

#include "video_render.h"
#include "video.h"
#include "config.h"
#include "macros.h"
#include "debug.h"
#include "delay.h"
#include "clocksource.h"
#include "led.h"
#include "logo.h"
#include "font.h"

#include <string.h>

RUN_FROM_RAM static void video_render_text_row(uint8_t text_row, uint8_t font_row);

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




void video_render_init(void) {
    video_stick_data[0] = VIDEO_RENDER_STICK_SIZE_X;    // A
    video_stick_data[1] = VIDEO_RENDER_STICK_SIZE_Y/2;  // E
    video_stick_data[2] = VIDEO_RENDER_STICK_SIZE_Y/2;  // T
    video_stick_data[3] = VIDEO_RENDER_STICK_SIZE_X;    // R
}

void video_render_overlay_sticks(uint16_t visible_line) {
    uint16_t py = visible_line - VIDEO_START_LINE_STICKS;

    // check if this is inside the stick ui region
    if (py > VIDEO_RENDER_STICK_SIZE_Y){
        // outside, abort
        return;
    }

    uint16_t page_to_render = video_line.fill_request;

    if ((py == 0) || (py == VIDEO_RENDER_STICK_SIZE_Y)) {
        // render top and bottom of stick rect
        uint8_t *buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][VIDEO_RENDER_STICK_POS_X/8/2];
        //uint8_t *buf_b = (uint8_t *)&video_line.buffer[BLACK][page_to_render][VIDEO_RENDER_STICK_POS_X/8/2];

        // white is shiftet 1 byte
        // plus shift for non 16bit aligned start positions:
        buf_w += 1 +  ((VIDEO_RENDER_STICK_POS_X/8) & 1);

        for (uint8_t i=VIDEO_RENDER_STICK_POS_X/8; i<((VIDEO_RENDER_STICK_POS_X + VIDEO_RENDER_STICK_SIZE_X)/8); i++){
            *buf_w++ = 0xFF;
            //*buf_b++ = 0xFF;
        }

    } else if (py == video_stick_data[2]) {
            // y value matches, render point at valid x pos
            uint8_t *buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][0];
            // add valid point
            uint8_t xr = VIDEO_RENDER_STICK_POS_X/8 + video_stick_data[3]/8;
            uint8_t xm = video_stick_data[3] & 0x07;  // = %8
            uint32_t v32w = 0b110110000000000000 >> xm;
            //uint32_t v32b = 0b001000000000000000 >> xm;
            buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][0];
            buf_w += xr + 1 - 1;

            *buf_w++ |= (v32w >> 16) & 0xFF;
            *buf_w++ |= (v32w >> 8) & 0xFF;
            *buf_w++ |= v32w & 0xFF;

    } else if ((py == video_stick_data[2]-1) || (py == video_stick_data[2]+1)){
            // y +/- 1 matches, render point at valid x pos
            uint8_t xr = VIDEO_RENDER_STICK_POS_X/8 + video_stick_data[3]/8;
            uint8_t xm = video_stick_data[3] & 0x07;  // = %8
            uint32_t v32w = 0b00000011100000000000000 >> xm;

            uint8_t *buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][0];
            buf_w += xr + 1 - 1;

            *buf_w++ |= (v32w >> 16) & 0xFF;
            *buf_w++ |= (v32w >> 8) & 0xFF;
            *buf_w++ |= v32w & 0xFF;
    }

    // add border
    uint8_t *buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][VIDEO_RENDER_STICK_POS_X/8/2];
    //uint8_t *buf_b = (uint8_t *)&video_line.buffer[BLACK][page_to_render][VIDEO_RENDER_STICK_POS_X/8/2];

    // white is shiftet 1 byte
    // plus shift for non 16bit aligned start positions:
    buf_w += 1 +  ((VIDEO_RENDER_STICK_POS_X/8) & 1);

    // render border
    *buf_w = (*buf_w) | 0x80;
    //*buf_b = (*buf_b) | 0x80;

    buf_w += VIDEO_RENDER_STICK_SIZE_X/8 - 1;
    //buf_b += VIDEO_RENDER_STICK_SIZE_X/8 - 1;

    *buf_w = (*buf_w) | 0x01;
    //*buf_b = (*buf_b) | 0x01;

/*
    // overlay stick position
    py = py - (uint16_t)video_stick_buffer_offset;
    if (py < 8) {
        // overlay stick pos
        uint8_t *buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][VIDEO_RENDER_STICK_POS_X/8/2];
        uint8_t *buf_b = (uint8_t *)&video_line.buffer[BLACK][page_to_render][VIDEO_RENDER_STICK_POS_X/8/2];

        // white is shiftet 1 byte
        buf_w++;

        uint8_t *stick_buf_w = (uint8_t *)&video_stick_buffer[WHITE][py][0];
        uint8_t *stick_buf_b = (uint8_t *)&video_stick_buffer[BLACK][py][0];

        for(uint8_t x=0; x<VIDEO_RENDER_STICK_SIZE_X/8; x++){
            *buf_w++ |= *stick_buf_w++;
            *buf_b++ |= *stick_buf_b++;
        }
    }
    */
}

void video_render_sticks(uint16_t visible_line) {
    uint16_t page_to_render = video_line.fill_request;

    uint16_t py = visible_line - VIDEO_START_LINE_STICKS;
    // start with empty buffer
    VIDEO_CLEAR_BUFFER(BLACK, page_to_render);
    VIDEO_CLEAR_BUFFER(WHITE, page_to_render);

    if (py > VIDEO_RENDER_STICK_SIZE){
        // outside active region, send empty line
        return;
    }

    // render left stick
    if ((py == 0) || (py == VIDEO_RENDER_STICK_SIZE-1)) {
        // render top and bottom of stick rect
        uint8_t *buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][0];
        uint8_t *buf_b = (uint8_t *)&video_line.buffer[BLACK][page_to_render][0];

        // white is shiftet 1 byte
        buf_w += VIDEO_RENDER_STICK_POS_X/8 + 1;
        buf_b += VIDEO_RENDER_STICK_POS_X/8;

        for (uint8_t i=VIDEO_RENDER_STICK_POS_X/8; i<((VIDEO_RENDER_STICK_POS_X + VIDEO_RENDER_STICK_SIZE)/8); i++){
            *buf_w++ = 0xFF;
            *buf_b++ = 0xFF;
        }
    } else if ((py == video_stick_data[2]-1) || (py == video_stick_data[2]+1)){
        // y +/- 1 matches, render point at valid x pos
        uint8_t xr = VIDEO_RENDER_STICK_POS_X/8 + video_stick_data[3]/8;
        uint8_t xm = video_stick_data[3] & 0x07;  // = %8
        uint32_t v32w = 0b00000011100000000000000 >> xm;

        uint8_t *buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][0];
        buf_w += xr + 1 - 1;

        *buf_w++ = (v32w >> 16) & 0xFF;
        *buf_w++ = (v32w >> 8) & 0xFF;
        *buf_w++ = v32w & 0xFF;

        // add border
        buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][0];
        buf_w += VIDEO_RENDER_STICK_POS_X/8 + 1;
        *buf_w = (*buf_w) | 0x80;
        buf_w += VIDEO_RENDER_STICK_SIZE/8 - 1;
        *buf_w = (*buf_w) | 0x01;
    //b[1][STICK_POS_X/8] |= 0x80
    //b[1][STICK_POS_X/8+STICK_SIZE/8-1] |= 0x01

    } else if (py == video_stick_data[2]) {
        // y value matches, render point at valid x pos
        uint8_t *buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][0];
        buf_w += VIDEO_RENDER_STICK_POS_X/8 + 1;
        *buf_w = (*buf_w) | 0x80;
        buf_w += VIDEO_RENDER_STICK_SIZE/8 - 1;
        *buf_w = (*buf_w) | 0x01;
        //b[0][STICK_POS_X/8+STICK_SIZE/8-1] = 0x01
        //b[1][STICK_POS_X/8+STICK_SIZE/8-1] = 0x01

        // add valid point
        uint8_t xr = VIDEO_RENDER_STICK_POS_X/8 + video_stick_data[3]/8;
        uint8_t xm = video_stick_data[3] & 0x07;  // = %8
        uint32_t v32w = 0b110110000000000000 >> xm;
        //uint32_t v32b = 0b001000000000000000 >> xm;
        buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][0];
        buf_w += xr + 1 - 1;

        *buf_w++ = (v32w >> 16) & 0xFF;
        *buf_w++ = (v32w >> 8) & 0xFF;
        *buf_w++ = v32w & 0xFF;

  /*
       # [00000000][00000011][11000000]
       val8 = val32 & 0xFF
       b[0][xr+1] |=  val8
       #b[1][xr+1] &= ~val8
       val8 = (val32 >> 8) & 0xFF
       b[0][xr+0] |=  val8
       b[1][xr+0] |=  (val32b >> 8)&0xFF
       #b[1][xr+0] &= ~val8
       val8 = (val32 >> 16) & 0xFF
       b[0][xr-1] |=  val8
       #b[1][xr-1] &= ~val8
*/

    } else {
        // render border only
        // y value matches, render point at valid x pos
        uint8_t *buf_w = (uint8_t *)&video_line.buffer[WHITE][page_to_render][0];
        buf_w += VIDEO_RENDER_STICK_POS_X/8 + 1;
        *buf_w = (*buf_w) | 0x80;
        buf_w += VIDEO_RENDER_STICK_SIZE/8 - 1;
        *buf_w = (*buf_w) | 0x01;
    }

}


void video_render_text(uint16_t visible_line) {
    if (VIDEO_DEBUG_DURATION_TEXTLINE) led_on();

    // even and odd lines render the same data in text mode:
    visible_line = visible_line / 2;

    uint8_t text_row = visible_line / 18;
    uint8_t font_row = visible_line % 18;

    if (text_row < VIDEO_CHAR_BUFFER_HEIGHT){
        // render text
        // run code from ram, faster as there are no waitstates
        video_render_text_row(text_row, font_row);
    } else {
        // text row is bigger than visible frame, send empty line
        VIDEO_CLEAR_BUFFER(BLACK, video_line.fill_request);
        VIDEO_CLEAR_BUFFER(WHITE, video_line.fill_request);
    }

    if (VIDEO_DEBUG_DURATION_TEXTLINE) led_off();
}


RUN_FROM_RAM static void video_render_text_row(uint8_t text_row, uint8_t font_row) {

    #if 1

    uint8_t page_to_fill = video_line.fill_request;

    uint8_t *video_buffer_ptr[2];
    video_buffer_ptr[0] = (uint8_t*) &video_line.buffer[WHITE][page_to_fill][0];
    video_buffer_ptr[1] = (uint8_t*) &video_line.buffer[BLACK][page_to_fill][0];

    // white data has to be shifted one byte with the first byte cleared
    *video_buffer_ptr[0]++ = 0x00;


    uint8_t *char_ptr = &video_char_buffer[text_row][0];

    uint8_t *font_ptr_row = (uint8_t*)&font_data[0][font_row][0];
    uint16_t *font_ptr_row16 = (uint16_t*)&font_data[1][font_row][0];

    uint16_t *b16 = (uint16_t*)&video_line.buffer[BLACK][page_to_fill][0];

    for (uint32_t text_col = 0; text_col < VIDEO_CHAR_BUFFER_WIDTH; text_col++) {
        //use manual loop unrolling here, subtract char ptr and add etc

            uint32_t index = *char_ptr++; //video_char_buffer[text_row][text_col]; //(*char_ptr++);

            // fetch ptr to font data
            //uint16_t index = video_char_buffer[text_row][text_col]*3;
            uint8_t *font_ptr = font_ptr_row + index*2;

            *video_buffer_ptr[0]++ = *font_ptr++;
            *video_buffer_ptr[0]++ = *font_ptr++;

            // get black font data:
            //font_ptr += sizeof(font_data[0])-2; //(uint8_t*)&font_data[1][font_row][index*2];

            uint16_t *fp16 = font_ptr_row16 + index;
            *b16++ = *fp16;
        }
    #else
    // fill white buffer
    uint8_t *line_buf_ptr = (uint8_t*) video_line_buffer.white_write_ptr;

    // white data has to be shifted one byte with the first byte cleared
    *line_buf_ptr++ = 0x00;

    // fetch text ptr
    uint8_t *char_ptr;
    uint8_t *font_ptr_head = (uint8_t*)&font_data[0][font_row][0];
    uint8_t *font_ptr; // = (uint8_t*)&font_data[0][font_row][0];

    // black: we can use 16bit aligned copy here
    uint16_t *font_ptr16;
    uint16_t *font_ptr_head16 = (uint16_t*)&font_data[1][font_row][0];
    uint16_t *line_buf_ptr16 = (uint16_t*) video_line_buffer.black_write_ptr;

    // white: process all chars (loop unrolled!):
    char_ptr = &video_char_buffer[text_row][0];
    UNROLL_LOOP(VIDEO_CHAR_BUFFER_WIDTH, {\
        font_ptr        = font_ptr_head + 2*(*char_ptr);\
        *line_buf_ptr++ = *font_ptr++;\
        *line_buf_ptr++ = *font_ptr;\

        font_ptr16        = font_ptr_head16 + *char_ptr++;\
        *line_buf_ptr16++ = *font_ptr16++;\
    });

    #endif
}

static uint32_t video_render_ani_count;
//static uint32_t video_render_ani_dir;

// this renders in ~23.2us (~64us available)
void video_render_animation(uint16_t visible_line) {
    if (VIDEO_DEBUG_DURATION_ANIMATION) led_on();

    //uint32_t data = 0;
    uint32_t logo_start_line = VIDEO_CENTER_ACTIVE_LINE - LOGO_HEIGHT/2;
    uint32_t logo_end_line   = VIDEO_CENTER_ACTIVE_LINE + LOGO_HEIGHT/2;

    uint32_t logo_offset;
    uint16_t *logo_ptr;
    uint16_t *video_buffer_ptr;
    uint8_t ani_dir;


    if (visible_line == VIDEO_CENTER_ACTIVE_LINE) {
        video_render_ani_count+=4;
        if (video_render_ani_count >=360) {
            video_render_ani_count -= 360;
        }
    }

    uint32_t scale;
    if (video_render_ani_count < 90) {
        scale   = video_cos_table[video_render_ani_count];
        ani_dir = 0;
    }else if (video_render_ani_count < 180) {
        scale   = video_cos_table[180 - video_render_ani_count];
        ani_dir = 1;
    }else if (video_render_ani_count < 270) {
        scale   = video_cos_table[video_render_ani_count - 180];
        ani_dir = 1;
    }else{
        scale   = video_cos_table[360 - video_render_ani_count];
        ani_dir = 0;
    }

    // calculate line number:
    uint32_t line    = visible_line + 2;

    logo_start_line = VIDEO_CENTER_ACTIVE_LINE - (scale * LOGO_HEIGHT/2) / 128;
    logo_end_line   = VIDEO_CENTER_ACTIVE_LINE + (scale * LOGO_HEIGHT/2) / 128;

    logo_offset = (line - logo_start_line) * 128 / scale * (LOGO_WIDTH/8);

    if (!ani_dir) {
        // flip on neg rotation
        logo_offset = LOGO_HEIGHT*LOGO_WIDTH/8 - logo_offset;
    }

    // fill the next line of data now:
    if ((line > logo_start_line) && (line < logo_end_line)){
        // black and white:
        for(uint8_t color = 0; color < 2; color++){
            // fetch source and dest ptr
            video_buffer_ptr = (uint16_t*) &video_line.buffer[color][video_line.fill_request][0];
            logo_ptr         = (uint16_t*)&logo_data16[color][logo_offset/2];

            uint16_t halfwords_todo = LOGO_WIDTH/8/2;
            while (halfwords_todo--) {
                *video_buffer_ptr++ = *logo_ptr++;
            }
        }
    }else{
        // no image data region
        VIDEO_CLEAR_BUFFER(BLACK, video_line.fill_request);
        VIDEO_CLEAR_BUFFER(WHITE, video_line.fill_request);
    }
    if (VIDEO_DEBUG_DURATION_ANIMATION) led_off();
}

#if 0
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


volatile uint32_t video_dbg;




volatile uint32_t video_spi_cr_trigger_b;

volatile uint32_t video_unprocessed_frame_sent;


#endif
