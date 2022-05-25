/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <drivers/drv_hrt.h>
#include <drivers/device/device.h>

#include <px4_platform_common/module.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/freq_input.h>

#if HRT_TIMER == FREQIN_TIMER
#error cannot share timer between HRT and FREQIN
#endif

#if !defined(GPIO_FREQ_IN) || !defined(FREQIN_TIMER) || !defined(FREQIN_TIMER_CHANNEL)
#error FREQIN defines are needed in board_config.h for this board
#endif

/* Get the timer defines */
#define INPUT_TIMER 		FREQIN_TIMER
#include "timer_registers.h"
#define FREQIN_TIMER_BASE	TIMER_BASE
#define FREQIN_TIMER_CLOCK	TIMER_CLOCK
#define FREQIN_TIMER_POWER_REG	TIMER_CLOCK_POWER_REG
#define FREQIN_TIMER_POWER_BIT	TIMER_CLOCK_POWER_BIT
#define FREQIN_TIMER_VECTOR	TIMER_IRQ_REG

/*
 * HRT clock must be at least 1MHz
 */
#if FREQIN_TIMER_CLOCK <= 1000000
# error FREQIN_TIMER_CLOCK must be greater than 1MHz
#endif

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(FREQIN_TIMER_BASE + _reg))

#define rCR1		REG(STM32_GTIM_CR1_OFFSET)
#define rCR2		REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR		REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER		REG(STM32_GTIM_DIER_OFFSET)
#define rSR		REG(STM32_GTIM_SR_OFFSET)
#define rEGR		REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1		REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2		REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER		REG(STM32_GTIM_CCER_OFFSET)
#define rCNT		REG(STM32_GTIM_CNT_OFFSET)
#define rPSC		REG(STM32_GTIM_PSC_OFFSET)
#define rARR		REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1		REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2		REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3		REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4		REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR		REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR		REG(STM32_GTIM_DMAR_OFFSET)

/*
 * Specific registers and bits used by HRT sub-functions
 */
#if FREQIN_TIMER_CHANNEL == 1
#define rCCR_FREQIN_A		rCCR1			/* compare register for FREQIN */
#define DIER_FREQIN_A		(GTIM_DIER_CC1IE) 	/* interrupt enable for FREQIN */
#define SR_INT_FREQIN_A		GTIM_SR_CC1IF		/* interrupt status for FREQIN */
#define rCCR_FREQIN_B		rCCR2 			/* compare register for FREQIN */
#define SR_INT_FREQIN_B		GTIM_SR_CC2IF		/* interrupt status for FREQIN */
#define CCMR1_FREQIN		((0x02 << GTIM_CCMR1_CC2S_SHIFT) | (0x01 << GTIM_CCMR1_CC1S_SHIFT))
#define CCMR2_FREQIN		0
#define CCER_FREQIN		(GTIM_CCER_CC2P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
#define SR_OVF_FREQIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
#define SMCR_FREQIN_1		(0x05 << GTIM_SMCR_TS_SHIFT)
#define SMCR_FREQIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_FREQIN_1)
#elif FREQIN_TIMER_CHANNEL == 2
#define rCCR_FREQIN_A		rCCR2			/* compare register for FREQIN */
#define DIER_FREQIN_A		(GTIM_DIER_CC2IE)	/* interrupt enable for FREQIN */
#define SR_INT_FREQIN_A		GTIM_SR_CC2IF		/* interrupt status for FREQIN */
#define rCCR_FREQIN_B		rCCR1			/* compare register for FREQIN */
#define DIER_FREQIN_B		GTIM_DIER_CC1IE		/* interrupt enable for FREQIN */
#define SR_INT_FREQIN_B		GTIM_SR_CC1IF		/* interrupt status for FREQIN */
#define CCMR1_FREQIN		((0x01 << GTIM_CCMR1_CC2S_SHIFT) | (0x02 << GTIM_CCMR1_CC1S_SHIFT))
#define CCMR2_FREQIN		0
#define CCER_FREQIN		(GTIM_CCER_CC1P | GTIM_CCER_CC1E | GTIM_CCER_CC2E)
#define SR_OVF_FREQIN		(GTIM_SR_CC1OF | GTIM_SR_CC2OF)
#define SMCR_FREQIN_1		(0x06 << GTIM_SMCR_TS_SHIFT)
#define SMCR_FREQIN_2		((0x04 << GTIM_SMCR_SMS_SHIFT) | SMCR_FREQIN_1)
#else
#error FREQIN_TIMER_CHANNEL must be either 1 and 2.
#endif

class FREQIN : public ModuleBase<FREQIN>
{
public:
	void start();
	void publish(uint16_t status, uint32_t period, uint32_t pulse_width, uint32_t frequency);
	void print_info(void);

	static int freqin_tim_isr(int irq, void *context, void *arg);

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int task_spawn(int argc, char *argv[]);


private:
	void timer_init(void);

	uint32_t _error_count {};
	uint32_t _pulses_captured {};
	uint32_t _last_period {};
	uint32_t _last_width {};
	uint32_t _last_freq {};

	bool _timer_started {};

	freq_input_s _freq {};

	uORB::PublicationData<freq_input_s> _freq_input_pub{ORB_ID(freq_input)};

};
