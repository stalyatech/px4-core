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

/* Get the timer defines */
#define INPUT_TIMER 			FREQIN_TIMER
#include "timer_registers.h"
#define FREQIN_TIMER_BASE		TIMER_BASE
#define FREQIN_TIMER_CLOCK		TIMER_CLOCK
#define FREQIN_TIMER_POWER_REG	TIMER_CLOCK_POWER_REG
#define FREQIN_TIMER_POWER_BIT	TIMER_CLOCK_POWER_BIT
#define FREQIN_TIMER_VECTOR		TIMER_IRQ_REG

/*
 * FREQIN clock must be at least 1MHz
 */
#if FREQIN_TIMER_CLOCK <= 1000000
# error FREQIN_TIMER_CLOCK must be greater than 1MHz
#endif

/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(FREQIN_TIMER_BASE + _reg))

#define rCR1		REG(STM32_ATIM_CR1_OFFSET)
#define rCR2		REG(STM32_ATIM_CR2_OFFSET)
#define rSMCR		REG(STM32_ATIM_SMCR_OFFSET)
#define rDIER		REG(STM32_ATIM_DIER_OFFSET)
#define rSR			REG(STM32_ATIM_SR_OFFSET)
#define rEGR		REG(STM32_ATIM_EGR_OFFSET)
#define rCCMR1		REG(STM32_ATIM_CCMR1_OFFSET)
#define rCCMR2		REG(STM32_ATIM_CCMR2_OFFSET)
#define rCCMR3		REG(STM32_ATIM_CCMR3_OFFSET)
#define rCCER		REG(STM32_ATIM_CCER_OFFSET)
#define rCNT		REG(STM32_ATIM_CNT_OFFSET)
#define rPSC		REG(STM32_ATIM_PSC_OFFSET)
#define rARR		REG(STM32_ATIM_ARR_OFFSET)
#define rRCR		REG(STM32_ATIM_RCR_OFFSET)
#define rCCR1		REG(STM32_ATIM_CCR1_OFFSET)
#define rCCR2		REG(STM32_ATIM_CCR2_OFFSET)
#define rCCR3		REG(STM32_ATIM_CCR3_OFFSET)
#define rCCR4		REG(STM32_ATIM_CCR4_OFFSET)
#define rBDTR		REG(STM32_ATIM_BDTR_OFFSET)
#define rDCR		REG(STM32_ATIM_DCR_OFFSET)
#define rDMAR		REG(STM32_ATIM_DMAR_OFFSET)


/* frequency measurement data structure */
typedef struct
{
    float perValue;
	float frequency;
    uint32_t capNumber;
    uint32_t capValue[2];
    uint32_t captured;
    uint32_t error;
} freq_meas_t;

class FREQIN : public ModuleBase<FREQIN>
{
public:
	void start();
	void publish(uint8_t channel);
	void print_info(void);

	static int freqin_tim_isr(int irq, void *context, void *arg);

	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int task_spawn(int argc, char *argv[]);


private:
	void timer_init(void);
	void reset(void);

	static constexpr uint32_t MAX_CHANNEL{3};

	freq_meas_t  _meas[MAX_CHANNEL] {0};
	freq_input_s _freq {};

	uORB::PublicationData<freq_input_s> _freq_input_pub{ORB_ID(freq_input)};

};
