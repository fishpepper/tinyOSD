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

#include <stdint.h>

void video_init(void);


#define TIMER_CLOCKS_PER_US                      (48000000 / 1000000)
#define _US_TO_CLOCKS(__us)                      ((uint32_t)((__us) * TIMER_CLOCKS_PER_US))

#define VIDEO_LINE_LEN            63.556  // us
#define VIDEO_SYNC_SHORT           2.000  // us
#define VIDEO_SYNC_HSYNC           4.700  // us
#define VIDEO_FIRST_FIELD          1      // ODD (NTSC)

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
