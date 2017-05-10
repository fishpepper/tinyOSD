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

#ifndef VIDEO_RENDER_H_
#define VIDEO_RENDER_H_

#include <stdint.h>
#include "logo.h"

#define VIDEO_START_LINE_ANIMATION (VIDEO_CENTER_ACTIVE_LINE - LOGO_HEIGHT/2)
#define VIDEO_END_LINE_ANIMATION (VIDEO_CENTER_ACTIVE_LINE + LOGO_HEIGHT/2)

void video_render_init(void);
void video_render_animation(uint16_t visible_line);
void video_render_text(uint16_t visible_line);



#endif  // VIDEO_RENDER_H_
