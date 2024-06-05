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

#include "freq_input.h"

using namespace time_literals;

FREQIN::FREQIN() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_freq_status_pub.advertise();
}

FREQIN::~FREQIN()
{

}

bool FREQIN::init()
{
	start();

	return true;
}

void FREQIN::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	/* calculate all periods */
	for (uint8_t i=0; i<MAX_CHANNEL; i++) {

		if (_meas[i].captured > 0) {

			/* clear the counters */
			_meas[i].captured = 0;
		} else if (++_meas[i].timeout >= MAX_TMOCOUNT) {

			/* reset the channel values */
			reset(i);
		}
	}
}

int FREQIN::task_spawn(int argc, char *argv[])
{
	FREQIN *instance = new FREQIN();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	} else {
		PX4_ERR("driver allocation failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}//task_spawn

void FREQIN::start()
{
	/* stat-up delay !!! */
	sleep(2);

	/* NOTE: must first publish here, first publication cannot be in interrupt context */
	_freq_status_pub.update();

	/* initialize the timer isr for measuring pulse widths. Publishing is done inside the isr. */
	timer_init();

	/* Start the scheduler */
	ScheduleOnInterval(100_ms); // 10 Hz
}//start

void FREQIN::timer_init(void)
{
	/* run with interrupts disabled in case the timer is already
	 * setup. We don't want it firing while we are doing the setup */
	irqstate_t flags = px4_enter_critical_section();

	/* configure input pin(s) */
	#ifdef GPIO_FREQ_IN1
	px4_arch_configgpio(GPIO_FREQ_IN1);
	#endif
	#ifdef GPIO_FREQ_IN2
	px4_arch_configgpio(GPIO_FREQ_IN2);
	#endif
	#ifdef GPIO_FREQ_IN3
	px4_arch_configgpio(GPIO_FREQ_IN3);
	#endif

	/* claim our interrupt vector */
	irq_attach(FREQIN_TIMER_VECTOR, FREQIN::freqin_tim_isr, NULL);

	/* Clear no bits, set timer enable bit.*/
	modifyreg32(FREQIN_TIMER_POWER_REG, 0, FREQIN_TIMER_POWER_BIT);

	/* disable and configure the timer */
	rCR1  = 0;
	rCR2  = 0;
	rSMCR = 0;
	rDIER = 0;
	#ifdef FREQIN_TIMER_CHANNEL1
	rDIER |= ATIM_DIER_CC1IE;
	#endif
	#ifdef FREQIN_TIMER_CHANNEL2
	rDIER |= ATIM_DIER_CC2IE;
	#endif
	#ifdef FREQIN_TIMER_CHANNEL3
	rDIER |= ATIM_DIER_CC3IE;
	#endif
	rCCER  = 0;		/* unlock CCMR* registers */
	rCCMR1 = ((0x01 << ATIM_CCMR1_CC1S_SHIFT) | (0x01 << ATIM_CCMR1_CC2S_SHIFT));
	rCCMR2 = ((0x01 << ATIM_CCMR2_CC3S_SHIFT));
	#ifdef FREQIN_TIMER_CHANNEL1
	rCCER |= ATIM_CCER_CC1E;
	#endif
	#ifdef FREQIN_TIMER_CHANNEL2
	rCCER |= ATIM_CCER_CC2E;
	#endif
	#ifdef FREQIN_TIMER_CHANNEL3
	rCCER |= ATIM_CCER_CC3E;
	#endif
	rDCR   = 0;

	/* for simplicity scale by the clock in 100KHz. This gives us
	 * readings in x10uS which is typically what is needed
	 * for a FREQ input driver */
	uint32_t prescaler = FREQIN_TIMER_CLOCK / 100000UL;

	/*
	 * define the clock speed. We want the highest possible clock
	 * speed that avoids overflows.
	 */
	rPSC = prescaler - 1;

	/* run the full span of the counter. All timers can handle
	 * uint16 */
	rARR = UINT16_MAX;

	/* generate an update event; reloads the counter, all registers */
	rEGR = ATIM_EGR_UG;

	/* enable the timer */
	rCR1 = ATIM_CR1_CEN;

	/* clear the data */
	memset(_meas, 0, sizeof(_meas));

	px4_leave_critical_section(flags);

	/* enable interrupts */
	up_enable_irq(FREQIN_TIMER_VECTOR);
}//timer_init

int FREQIN::freqin_tim_isr(int irq, void *context, void *arg)
{
	/* get the timer status */
	uint16_t status = rSR;
	uint32_t period = 0;

	/* ack the interrupts we just read */
	rSR = 0;

	/* get the object instance */
	auto obj = get_instance();
	if (obj == nullptr) {
		return PX4_ERROR;
	}

	/* get the variable instance */
	freq_meas_t *meas = &(obj->_meas[0]);

#ifdef FREQIN_TIMER_CHANNEL1
	/* channel 1 capture */
	if (status & ATIM_SR_CC1IF) {

	  	/* get the Input Capture value */
	  	meas[0].capValue[meas[0].capNumber++ & 1] = (uint16_t)rCCR1;
	}

	/* channel 1 error */
	if (status & ATIM_SR_CC1OF) {

	  	/* increment the error counter */
	  	meas[0].error++;
	}
#endif /* FREQIN_TIMER_CHANNEL1 */


#ifdef FREQIN_TIMER_CHANNEL2
	/* channel 2 capture */
	if (status & ATIM_SR_CC2IF) {

	  	/* get the Input Capture value */
	  	meas[1].capValue[meas[1].capNumber++ & 1] = (uint16_t)rCCR2;
	}

	/* channel 2 error */
	if (status & ATIM_SR_CC2OF) {

	  	/* increment the error counter */
	  	meas[1].error++;
	}
#endif /* FREQIN_TIMER_CHANNEL2 */

#ifdef FREQIN_TIMER_CHANNEL3
	/* channel 3 capture */
	if (status & ATIM_SR_CC3IF) {

	  	/* get the Input Capture value */
	  	meas[2].capValue[meas[2].capNumber++ & 1] = (uint16_t)rCCR3;
	}

	/* channel 2 error */
	if (status & ATIM_SR_CC2OF) {

	  	/* increment the error counter */
	  	meas[2].error++;
	}
#endif /* FREQIN_TIMER_CHANNEL3 */

	/* calculate all periods */
	for (uint8_t i=0; i<MAX_CHANNEL; i++) {

	  	if (meas[i].capNumber >= 2) {

			/* Compute the period length and frequency */
			if (meas[i].capValue[0] < meas[i].capValue[1]) {
				period = meas[i].capValue[1] - meas[i].capValue[0];
			} else {
				period = (0x10000 - meas[i].capValue[0]) + meas[i].capValue[1] + 1;
			}

			meas[i].perValue  = period * 10.0f;														// uS
			meas[i].frequency = (meas[i].perValue > 0.0f) ? (1000000.0f / meas[i].perValue) : (-1);	// Hz
			meas[i].timeout   = 0;
			meas[i].capNumber = 0;
			meas[i].capValue[0] = 0;
			meas[i].capValue[1] = 0;
			meas[i].captured++;

			/* pulish the values */
			obj->publish(i);
	  	}
	}

	return PX4_OK;
}//freqin_tim_isr

void FREQIN::publish(uint8_t channel)
{
	_freq_stat.channel 	   = channel + 1;
	_freq_stat.timestamp   = hrt_absolute_time();
	_freq_stat.error_count = _meas[channel].error;
	_freq_stat.period 	   = _meas[channel].perValue;
	_freq_stat.pulse_width = _meas[channel].perValue/2;
	_freq_stat.frequency   = _meas[channel].frequency;
	_freq_status_pub.publish(_freq_stat);
}//publish

void FREQIN::print_info(uint8_t channel)
{
	/* print the channel variables */
	if (channel < MAX_CHANNEL) {
		PX4_INFO("chan=%u count=%u error=%u period=%uuS freq=%uHz",
			static_cast<unsigned>(channel),
			static_cast<unsigned>(_meas[channel].captured),
			static_cast<unsigned>(_meas[channel].error),
			static_cast<unsigned>(_meas[channel].perValue),
			static_cast<unsigned>(_meas[channel].frequency));
	}
}//print_info

void FREQIN::print_info(void)
{
	for (uint8_t i=0; i<MAX_CHANNEL; i++) {
		/* print the channel variables */
		print_info(i);
	}
}//print_info

void FREQIN::reset(uint8_t channel)
{
	/* reset the channel variables */
	if (channel < MAX_CHANNEL) {
		_meas[channel].perValue = 0;
		_meas[channel].capNumber = 0;
		_meas[channel].capValue[0] = 0;
		_meas[channel].capValue[1] = 0;
		_meas[channel].captured = 0;
		_meas[channel].error = 0;
		_meas[channel].timeout = 0;
		_meas[channel].frequency = 0;
	}
}//reset

void FREQIN::reset(void)
{
	for (uint8_t i=0; i<MAX_CHANNEL; i++) {
		/* reset the channel variables */
		reset(i);

		/* print the channel variables */
		print_info(i);
	}
}//reset

int FREQIN::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Measures the FREQ input on FRQ1, FRQ2, FRQ3 via a timer capture ISR and publishes via the uORB 'freq_input` message.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("freq_input", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("values", "prints FREQ capture info.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "resets FREQ capture info.");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}//print_usage

int FREQIN::custom_command(int argc, char *argv[])
{
	const char *input = argv[0];
	auto *obj = get_instance();

	if (!is_running() || !obj) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	if (!strcmp(input, "values")) {
		obj->print_info();
		return PX4_OK;
	}

	if (!strcmp(input, "reset")) {
		obj->reset();
		return PX4_OK;
	}

	print_usage();
	return PX4_ERROR;
}//custom_command

extern "C" __EXPORT int freq_input_main(int argc, char *argv[])
{
	return FREQIN::main(argc, argv);
}//freq_input_main
