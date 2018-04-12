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

#ifndef __RECEIVER_H__
#define __RECEIVER_H__

#include <arm_math.h>
#include <lpc43xx_gpio.h>

#define AUDIO_RATE			16000
#define FIR_DECIMATION_RATIO	5
#define IF_RATE				80000
#define CIC_DECIMATION_RATIO	150
#define ADC_RATE			12000000

#define CAPTUREBUFFER_SIZE	60000
#define CAPTUREBUFFER0		((uint8_t*)0x20000000)
#define CAPTUREBUFFER1		((uint8_t*)0x20008000)
#define CAPTUREBUFFER_SIZEHALF	30000

#define NCO_TABLE_SIZE		3000
#define NCO_SAMPLES			3000
#define NCO_SIN_TABLE		((int16_t*)0x1008C000)
#define NCO_COS_TABLE		((int16_t*)0x1008E000)
#define NCO_AMPL			(SHRT_MAX/128)

/*  30000 / 150 = 200 */
#define FIR_BUFFER_SIZE		200
#define FIR_STATE_SIZE		64*2

#define I_FIR_STATE			((q15_t*)0x10080000)
#define I_FIR_BUFFER		((q15_t*)0x10080080)
#define Q_FIR_STATE			((q15_t*)0x10081000)
#define Q_FIR_BUFFER		((q15_t*)0x10081080)

/* 200 / 10 = 20 */
#define IIR_BUFFER_SIZE		20

#define IIR_I_BUFFER 		((q15_t*)0x10082000)
#define IIR_Q_BUFFER 		((q15_t*)0x10082100)

#define AUD_I_BUFFER 		((q15_t*)0x10082200)
#define AUD_Q_BUFFER 		((q15_t*)0x10082300)

#define AUD_BUFFER 			((q15_t*)0x10082400)

typedef enum {
	MOD_LSB,
	MOD_USB,
	MOD_MAX
} modulation_t;

typedef struct {
	enum { FREQ, GAIN, MOD, AGCMODE, RFGAIN, SPDISP, TESTP, DEBUGMODE, MODE_MAX } mode;
	int gain;
	int channel;
	uint32_t freq;
	modulation_t modulation;
	int digit; /* 0~5 */
	enum { AGC_MANUAL, AGC_SLOW, AGC_MID, AGC_FAST } agcmode;
	int rfgain;
	float32_t ncoampl;
	enum { SPDISP_CAP0, SPDISP_CAP, SPDISP_CIC, SPDISP_FIR, SPDISP_IIR, SPDISP_AUD, SPDISP_MODE_MAX } spdispmode;
	int tp;
	int debugmode;
} uistat_t;

#define UISTAT ((uistat_t*)0x10083f00)


typedef struct {
	uint32_t sample_freq;
	int16_t offset;
	int16_t stride;
	int16_t overgain;

	int16_t origin;
	int16_t tickstep;
	int16_t tickbase;
	int16_t tickunit;
	const char *unitname;
} spectrumdisplay_param_t;

// when event sent with SEV from M4 core, filled following data
typedef struct {
	q31_t *buffer;
	uint32_t buffer_rest;
	uint8_t update_flag;
	uint8_t ui_update_flag;
	spectrumdisplay_param_t p;
} spectrumdisplay_info_t;

#define FLAG_SPDISP 	(1<<0)
#define FLAG_UI 		(1<<1)

#define SPDISPINFO ((spectrumdisplay_info_t*)0x10083f80)

// r:2048 c:1024 samples (8192 byte with q31_t)
#define SPDISP_BUFFER_SIZE	8192
#define SPDISP_BUFFER	 	((q31_t*)0x10084000)


#define DEMOD_GAINBITS		0	/* 0 ~ 10 */

#define AUDIO_BUFFER 		((q15_t*)0x1008A000)
//#define AUDIO_BUFFER_SIZE	0x2000
#define AUDIO_BUFFER_SIZE	0x400
#define AUDIO_TEST_BUFFER 	((q15_t*)0x1008C000)

// dsp.c
extern void DMA_IRQHandler(void);
extern void nco_set_frequency(int32_t freq, float32_t ampl);
extern void generate_test_tone(int freq);
extern void dsp_init();
extern void set_modulation(modulation_t modulation);

extern void update_adc_dc_offset(void);
extern void audio_set_gain(int gain);

#define AUDIO_GAIN_MAX 29
#define AUDIO_GAIN_MIN -7
#define AUDIO_GAIN_REF 7

// ui.c
extern void ui_init();
extern void ui_process();

// clkcfg.h
extern void setup_systemclock();
extern void setup_pll0audio(uint32_t msel, uint32_t nsel, uint32_t psel);
extern void setup_i2s_clock(LPC_I2Sn_Type *I2Sx, uint32_t Freq, uint8_t TRMode);


extern volatile int32_t capture_count;
extern int32_t agc_gain;
extern int32_t agc_multiplier;
extern int32_t agc_rshift;
extern int32_t ovf_flags;
extern int16_t agc_slowness;

extern int16_t spdisp_fetch_mode;
void spdisp_fetch_start();


typedef struct {
	uint16_t write_current;
	uint16_t write_total;
	uint16_t read_total;
	uint16_t read_current;
	uint16_t rebuffer_count;
} audio_state_t;

typedef struct {
	q15_t *dest;
	int16_t *nco_base;
	int32_t dest_idx;
	int32_t s0;
	int32_t s1;
	int32_t d0;
	int32_t d1;
} cic_state_t;

extern cic_state_t cic_i;
extern cic_state_t cic_q;

typedef struct {
	int32_t index;
	uint32_t average;
	uint32_t max;
} am_demod_state_t;

extern am_demod_state_t am_demod_state;


// pwm.c
typedef enum {
	PWM_6 = 6
} PWMName;

struct pwmout_s {
    PWMName pwm;
    uint8_t mr;
};

typedef struct pwmout_s pwmout_t;

void pwmout_init(pwmout_t* obj, int pwm);
void pwmout_write(pwmout_t* obj, int value);
void pwmout_setupdma(uint16_t *buffer, int count);



#define LED_INIT()	     (LPC_GPIO_PORT->DIR[0] |= (1UL << 8))
#define LED_ON()		 (LPC_GPIO_PORT->SET[0] |= (1UL << 8))
#define LED_OFF()		 (LPC_GPIO_PORT->CLR[0] = (1UL << 8))
#define LED_TOGGLE()	 (LPC_GPIO_PORT->NOT[0] = (1UL << 8))

#define ROTLED_INIT()	     (LPC_GPIO_PORT->DIR[1] |= (1UL << 3)|(1UL << 4))
#define ROTLED_RED()		 (LPC_GPIO_PORT->SET[1] |= (1UL << 3))
#define ROTLED_GREEN()		 (LPC_GPIO_PORT->SET[1] |= (1UL << 4))
#define ROTLED_OFF()		 (LPC_GPIO_PORT->CLR[1] = (1UL << 3)|(1UL << 4))

#define TESTPOINT_INIT() \
do {scu_pinmux(0x6, 11, PUP_DISABLE | PDN_DISABLE | SLEWRATE_SLOW | FILTER_ENABLE, FUNC0); \
	LPC_GPIO_PORT->DIR[3] |= (1UL << 7); \
	LPC_GPIO_PORT->SET[3] |= (1UL << 7); } while(0)
#define TESTPOINT_ON() 	(LPC_GPIO_PORT->SET[3] |= (1UL << 7))
#define TESTPOINT_OFF()	(LPC_GPIO_PORT->CLR[3] = (1UL << 7))
#define TESTPOINT_TOGGLE()	(LPC_GPIO_PORT->NOT[3] = (1UL << 7))
#define TESTPOINT_SPIKE()	TESTPOINT_TOGGLE();TESTPOINT_TOGGLE()

#define DMA_HALT()	(LPC_GPDMA->C0CONFIG |= (1 << 18))
#define DMA_RUN()	(LPC_GPDMA->C0CONFIG &= ~(1 << 18))

#endif /* __RECEIVER_H__ */
