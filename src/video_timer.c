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

#include "video_timer.h"
#include "video_spi_dma.h"
#include "video.h"
#include "config.h"
#include "macros.h"
#include "debug.h"
#include "led.h"
#include "clocksource.h"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/comparator.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/dma.h>


#include <libopencmsis/core_cm3.h>
#include <libopencm3/cm3/nvic.h>

static void video_timer_init_tim1(void);
static void video_timer_init_comparator(void);

static volatile uint16_t video_sync_last_compare_value;

void video_timer_init(void) {
    video_timer_init_tim1();
    video_timer_init_comparator();
    //video_timer_init_comparator_interrupt();
}

static void video_timer_init_tim1(void) {
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

    // enable master mode TRGO = CH3 OC (used as adc trigger)
    timer_set_master_mode(TIM1, TIM_CR2_MMS_COMPARE_OC3REF);
    //OC3REF is set on compare match
    timer_set_oc_mode(TIM1, TIM_OC3, TIM_OCM_PWM2);



    // input compare trigger
    // on channel IC2
    timer_ic_set_input(TIM1, TIM_IC2, TIM_IC_IN_TI1);
    timer_ic_set_polarity(TIM1, TIM_IC2, TIM_IC_BOTH);
    timer_ic_set_prescaler(TIM1, TIM_IC2, TIM_IC_PSC_OFF);
    timer_ic_set_filter(TIM1, TIM_IC2, TIM_IC_OFF);
    timer_ic_enable(TIM1, TIM_IC2);

    // set CC2 as output to internals
    //timer_ic_set_input(TIM1, TIM_IC2, TIM_CCMR1_CC2S_OUT);


    // set up oc2 interrupt
    nvic_set_priority(NVIC_TIM1_CC_IRQ, NVIC_PRIO_TIMER1);
    nvic_enable_irq(NVIC_TIM1_CC_IRQ);

    timer_set_dma_on_compare_event(TIM1);
    timer_disable_oc_preload(TIM1, TIM_OC1);
    timer_disable_oc_preload(TIM1, TIM_OC3);
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

static void video_timer_init_comparator(void) {
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

void video_timer_init_interrupt(void) {
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
    //video_dbg = pulse_len;

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
                video_line.active_line = 15;
            } else {
                // even
                video_line.active_line = 14;
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

            // let the adc sample the middle of each line:
            TIM_CCR3(TIM1) = current_compare_value + _US_TO_CLOCKS(19);

            if (VIDEO_DEBUG_DMA) led_on();

            // prepare next page rendering:
            video_line.currently_rendering = 1 - video_line.currently_rendering;
            video_line.fill_request        = video_line.currently_rendering;

            TIMER_ENABLE_IRQ(TIM1, TIM_DIER_CC1DE | TIM_DIER_CC4DE);

            if (VIDEO_DEBUG_DMA) TIMER_ENABLE_IRQ(TIM1, TIM_DIER_CC4IE);

            //TIMER_ENABLE_IRQ(TIM1, TIM_DIER_CC3IE);

            // enable dma channel, this was set up during the dma end of tx int
            DMA_ENABLE_CHANNEL(VIDEO_DMA_WHITE, DMA_CHANNEL2);
            DMA_ENABLE_CHANNEL(VIDEO_DMA_WHITE, DMA_CHANNEL3);
            DMA_ENABLE_CHANNEL(VIDEO_DMA_BLACK, DMA_CHANNEL4);
            DMA_ENABLE_CHANNEL(VIDEO_DMA_BLACK, DMA_CHANNEL5);

            if (VIDEO_DEBUG_DMA) led_off();

            // increment video field
            video_line.active_line += 2;

            // prepare for the next field
            video_field = VIDEO_FIRST_FIELD;
        } else {
            // this is a broad sync
        }
    }

    // store current value for period measurements
    video_sync_last_compare_value = current_compare_value;
}

// void TIM1_CC_IRQHandler(void) {
void TIM1_CC_IRQHandler(void) {
    if (TIMER_GET_FLAG(TIM1, TIM_SR_CC4IF)) {
        TIMER_CLEAR_FLAG(TIM1, TIM_SR_CC4IF);
        if (VIDEO_DEBUG_DMA) led_toggle();
        if (VIDEO_DEBUG_DMA) led_toggle();
    }
    /*if (TIMER_GET_FLAG(TIM1, TIM_SR_CC3IF)) {
        TIMER_CLEAR_FLAG(TIM1, TIM_SR_CC3IF);
        led_toggle();
        led_toggle();
    }*/
}

