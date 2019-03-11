# tinyOSD

My approach to a custom, fully graphic OSD for analog video without 
using special hardware.

Some time ago I got annoyed by the available OSD solutions and I 
started to look for alternatives. Nowadays small micro controllers 
are fast enough and include fancy features like DMA so that the OSD 
generation can be handled in software. 
I selected a STM32F3 due to the availability in small packages and 
the necessary features I needed and got working. 

[![tinyOSD youtube video](https://img.youtube.com/vi/USWiuCVAzIQ/0.jpg)](https://www.youtube.com/watch?v=USWiuCVAzIQ)

The result:
* Fully opensource
* Very high update rates
* Full graphic overlays and animations
* Custom, nice font (can be changed)
* Currently showing 35 chars in 13 lines
* "Multi-color": up to eight levels of gray on the screen
* Configurable global brightness and "blackness" setting

In addition I also release the hardware design of a reference 
implementation that also includes a RTC6705 vide transmitter chip:
see https://github.com/fishpepper/tinyFINITY

Please refer to my blogpost for more details:
https://fishpepper.de/2019/03/11/tinyosd-tinyfinity-a-tiny-opensource-video-tx-with-full-graphic-osd


