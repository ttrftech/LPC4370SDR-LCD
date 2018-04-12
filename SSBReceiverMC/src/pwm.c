/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <lpc43xx.h>
#include <lpc43xx_sct.h>
#include <lpc43xx_scu.h>
#include <lpc43xx_gpdma.h>
#include "receiver.h"

#define PWM_SETCOUNT(x) (x - 1)                 // set count value
#define PWM_GETCOUNT(x) (x + 1)                 // get count value

#define PWM_EVENT_MAX (CONFIG_SCT_nEV/2 - 1)  // Max PWM channels
#define PWM_CONFIG    SCT_CONFIG_16BIT_COUNTER // default config
#define PWM_CTRL      &LPC_SCT->CTRL_L        // control register
#define PWM_HALT      SCT_CTRL_HALT_L         // halt counter
#define PWM_CLEAR     SCT_CTRL_CLRCTR_L       // clock clear
#define PWM_PRE(x)    SCT_CTRL_PRE_L(x)       // clock prescale
#define PWM_EVT_MASK  (1 << 12)               // event control mask
#define PWM_LIMIT     &LPC_SCT->LIMIT_L       // limit register
#define PWM_MATCH(x)  &LPC_SCT->MATCH[x].L    // match register
#define PWM_MR(x)     &LPC_SCT->MATCHREL[x].L // 16-bit match reload register

#define PWM_MR0         PWM_MR(0)               // MR register 0 is for period


static unsigned int pwm_clock_mhz;

static void _pwmout_dev_init() {
    uint32_t i;

    // set SCT clock and config
    LPC_CCU1->CLK_M4_SCT_CFG = (1 << 0); // enable SCT clock in CCU1
    LPC_SCT->CONFIG |= PWM_CONFIG; // set config options
    *PWM_CTRL |= PWM_HALT; // set HALT bit to stop counter
    // clear counter and set prescaler for desired freq
    //*PWM_CTRL |= PWM_CLEAR | PWM_PRE(SystemCoreClock / PWM_FREQ_BASE - 1);
    //pwm_clock_mhz = PWM_FREQ_BASE / 1000000;
    *PWM_CTRL |= PWM_CLEAR | 0; /* PWM_PRE is zero */
    pwm_clock_mhz = SystemCoreClock;

    // configure SCT events
    for (i = 0; i < PWM_EVENT_MAX; i++) {
        *PWM_MATCH(i) = 0; // match register
        *PWM_MR(i) = 0; // match reload register
        LPC_SCT->EVENT[i].STATE = 0xFFFFFFFF; // event happens in all states
        LPC_SCT->EVENT[i].CTRL  = (i << 0) | PWM_EVT_MASK; // match condition only
    }
    *PWM_LIMIT = (1 << 0) ; // set event 0 as limit
    // initialize period to 20ms: standard for servos, and fine for e.g. brightness control
    //*PWM_MR0 = PWM_SETCOUNT((uint32_t)((5 * PWM_FREQ_BASE) / 1000000));
    *PWM_MR0 = PWM_SETCOUNT(pwm_clock_mhz / 48000); // 48kHz

    // initialize SCT outputs
    for (i = 0; i < CONFIG_SCT_nOU; i++) {
        LPC_SCT->OUT[i].SET = (1 << 0); // event 0 will set SCTOUT_xx
        LPC_SCT->OUT[i].CLR = 0; // set clear event when duty cycle
    }
    LPC_SCT->OUTPUT = 0; // default outputs to clear

    *PWM_CTRL &= ~PWM_HALT; // clear HALT bit to start counter
}

static uint8_t event = 0;

void pwmout_init(pwmout_t* obj, int pwm) {
	// CTOUT_6 at P6_5 — SCT output 6
    scu_pinmux(0x6, 5, MD_PLN_FAST, FUNC1);
    if (event == 0)
    	_pwmout_dev_init();
    obj->mr = ++event;
    obj->pwm = pwm;
}

void pwmout_write(pwmout_t* obj, int v) {
    if (v > 0) {
        // set new match register value and enable SCT output
        *PWM_MR(obj->mr) = PWM_SETCOUNT(v);
        LPC_SCT->OUT[obj->pwm].CLR = (1 << obj->mr);  // on event will clear PWM_XX
    } else {
        // set match to zero and disable SCT output
        *PWM_MR(obj->mr) = 0;
        LPC_SCT->OUT[obj->pwm].CLR = 0;
    }
}


#define DMA_LLI_NUM    2
static GPDMA_LLI_Type pwmdma_lli[DMA_LLI_NUM];

static pwmout_t pwmdma;

void pwmout_setupdma(uint16_t *buffer, int count)
{
  int i;
  uint32_t blocksize;

  LPC_CREG->DMAMUX &= ~(0x3 << 0);
  LPC_CREG->DMAMUX |= 0x1 << 0;  /* peripheral 0 SCT CTOUT_2 (0x1) */

  blocksize = count / DMA_LLI_NUM;

  for (i = 0; i < DMA_LLI_NUM; i++) {
	  pwmdma_lli[i].SrcAddr = (uint32_t)buffer;
	  pwmdma_lli[i].DstAddr = (uint32_t)PWM_MR(1);
	  pwmdma_lli[i].NextLLI = (uint32_t)(&pwmdma_lli[(i+1) % DMA_LLI_NUM]);
	  pwmdma_lli[i].Control = (blocksize << 0) |      // Transfersize (does not matter when flow control is handled by peripheral)
                           (0x0 << 12)  |          // Source Burst Size
                           (0x0 << 15)  |          // Destination Burst Size
                           (0x1 << 18)  |          // Source width // 16 bit width
                           (0x1 << 21)  |          // Destination width // 16 bits
                           (0x0 << 24)  |          // Source AHB master 0 / 1
                           (0x1 << 25)  |          // Dest AHB master 0 / 1
                           (0x1 << 26)  |          // Source increment(LAST Sample)
                           (0x0 << 27)  |          // Destination increment
                           (0x0UL << 31);          // Terminal count interrupt disabled
	  buffer += blocksize;
  }

  LPC_GPDMA->C1SRCADDR = pwmdma_lli[0].SrcAddr;
  LPC_GPDMA->C1DESTADDR = pwmdma_lli[0].DstAddr;
  LPC_GPDMA->C1CONTROL = pwmdma_lli[0].Control;
  LPC_GPDMA->C1LLI     = pwmdma_lli[0].NextLLI;   // must be pointing to the second LLI as the first is used when initializing
  LPC_GPDMA->C1CONFIG  =  (0x1)        |          // Enable bit
                          (0x0 << 1)   | 		  // SRCPERIPHERAL - memory
                          (0x0 << 6)   |          // Destination peripheral - SCT CTOUT_2
                          (0x1 << 11)  |          // Flow control - memory to peripheral  - DMA control
                          (0x0 << 14)  |          // Int error mask
                          (0x0 << 15);            // ITC - term count error mask

  pwmout_init(&pwmdma, 2);
  pwmout_write(&pwmdma, 4096);
#if 0
  // clear CTOUT_2 on 4096 (CTOUT_2 is used to trigger DMA transfer)
  *PWM_MR(1) = PWM_SETCOUNT(4096);
  LPC_SCT->OUT[2].CLR = 1 << 1;
#endif
}
