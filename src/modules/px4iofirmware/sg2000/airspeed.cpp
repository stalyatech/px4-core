/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file airspeed.cpp
 *
 * MS4525DO airspeed sensor on the TPU-v2 I2C bus.  The IO only samples and
 * validates the raw counts; the FMU px4io driver reads PX4IO_PAGE_AIRSPEED,
 * converts them and publishes differential_pressure.
 */

#include <px4_platform_common/px4_config.h>

#include <pthread.h>
#include <sched.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>

#include <drivers/drv_hrt.h>

#include "px4io.h"

#if defined(PX4IO_AIRSPEED_I2C_ADDR)

namespace
{

constexpr uint32_t I2C_FREQUENCY = 100000;
constexpr uint8_t CMD_READ_MR = 0x00;		// start a measurement

// status bits [7:6] of the first data byte
constexpr uint8_t STATUS_NORMAL = 0;
constexpr uint8_t STATUS_STALE = 2;
constexpr uint8_t STATUS_FAULT = 3;

// The NuttX tick is 10 ms, so these sleeps are at least one tick each and
// a full cycle (measure, wait, fetch, wait) takes about 20 ms.
constexpr useconds_t CONVERSION_WAIT_US = 5000;
constexpr useconds_t CYCLE_WAIT_US = 10000;
constexpr useconds_t ABSENT_WAIT_US = 500000;

constexpr unsigned RESET_AFTER_ERRORS = 10;

struct i2c_master_s *_i2c {nullptr};
pthread_t _thread{};

uint16_t _page[PX4IO_P_AIRSPEED_COUNT] {};
hrt_abstime _sample_time{0};

int i2c_write(const uint8_t *data, size_t len)
{
	struct i2c_msg_s msg {};
	msg.frequency = I2C_FREQUENCY;
	msg.addr = PX4IO_AIRSPEED_I2C_ADDR;
	msg.flags = 0;
	msg.buffer = const_cast<uint8_t *>(data);
	msg.length = len;
	return I2C_TRANSFER(_i2c, &msg, 1);
}

int i2c_read(uint8_t *data, size_t len)
{
	struct i2c_msg_s msg {};
	msg.frequency = I2C_FREQUENCY;
	msg.addr = PX4IO_AIRSPEED_I2C_ADDR;
	msg.flags = I2C_M_READ;
	msg.buffer = data;
	msg.length = len;
	return I2C_TRANSFER(_i2c, &msg, 1);
}

void count_error()
{
	irqstate_t flags = enter_critical_section();
	_page[PX4IO_P_AIRSPEED_ERRORS]++;
	leave_critical_section(flags);
}

void set_present(bool present)
{
	irqstate_t flags = enter_critical_section();

	if (present) {
		_page[PX4IO_P_AIRSPEED_STATUS] |= PX4IO_P_AIRSPEED_STATUS_PRESENT;

	} else {
		_page[PX4IO_P_AIRSPEED_STATUS] &= ~(PX4IO_P_AIRSPEED_STATUS_PRESENT | PX4IO_P_AIRSPEED_STATUS_VALID);
	}

	leave_critical_section(flags);
}

/*
 * One measurement as the PX4 MS4525DO driver does it: request a conversion,
 * then fetch twice.  The first fetch must report normal status and the
 * second stale status with the same data, otherwise the sample is dropped.
 */
bool sample()
{
	const uint8_t cmd = CMD_READ_MR;

	if (i2c_write(&cmd, 1) < 0) {
		return false;
	}

	const hrt_abstime sample_time = hrt_absolute_time();
	set_present(true);

	usleep(CONVERSION_WAIT_US);

	uint8_t data_1[4] {};
	uint8_t data_2[4] {};

	if (i2c_read(data_1, sizeof(data_1)) < 0 || i2c_read(data_2, sizeof(data_2)) < 0) {
		return false;
	}

	const uint8_t status_1 = data_1[0] >> 6;
	const uint8_t status_2 = data_2[0] >> 6;

	if (status_1 == STATUS_FAULT || status_2 == STATUS_FAULT) {
		return false;
	}

	const uint16_t bridge_1 = ((data_1[0] & 0x3f) << 8) | data_1[1];
	const uint16_t bridge_2 = ((data_2[0] & 0x3f) << 8) | data_2[1];
	const uint16_t temperature_1 = ((data_1[2] << 8) | (data_1[3] & 0xe0)) >> 5;
	const uint16_t temperature_2 = ((data_2[2] << 8) | (data_2[3] & 0xe0)) >> 5;

	if (status_1 != STATUS_NORMAL || status_2 != STATUS_STALE
	    || bridge_1 != bridge_2 || temperature_1 != temperature_2) {
		// no fresh conversion yet, not a fault
		return true;
	}

	irqstate_t flags = enter_critical_section();
	_page[PX4IO_P_AIRSPEED_BRIDGE] = bridge_1;
	_page[PX4IO_P_AIRSPEED_TEMP] = temperature_1;
	_page[PX4IO_P_AIRSPEED_SEQ]++;
	_page[PX4IO_P_AIRSPEED_STATUS] |= PX4IO_P_AIRSPEED_STATUS_VALID;
	_sample_time = sample_time;
	leave_critical_section(flags);

	return true;
}

void *airspeed_worker(void *arg)
{
	(void)arg;
	unsigned errors_in_row = 0;

	while (true) {
		if (sample()) {
			errors_in_row = 0;
			usleep(CYCLE_WAIT_US);
			continue;
		}

		count_error();

		if (++errors_in_row < RESET_AFTER_ERRORS) {
			usleep(CYCLE_WAIT_US);
			continue;
		}

		// not there or the bus is stuck: reset the controller and retry slowly
		set_present(false);
		errors_in_row = 0;
#ifdef CONFIG_I2C_RESET
		I2C_RESET(_i2c);
#endif
		usleep(ABSENT_WAIT_US);
	}

	return nullptr;
}

} // namespace

void airspeed_snapshot(uint16_t page[PX4IO_P_AIRSPEED_COUNT])
{
	irqstate_t flags = enter_critical_section();
	memcpy(page, _page, sizeof(_page));
	const hrt_abstime sample_time = _sample_time;
	leave_critical_section(flags);

	uint16_t age = 0xffff;

	if (sample_time != 0) {
		const hrt_abstime elapsed_ms = hrt_elapsed_time(&sample_time) / 1000;

		if (elapsed_ms < 0xffff) {
			age = static_cast<uint16_t>(elapsed_ms);
		}
	}

	page[PX4IO_P_AIRSPEED_AGE] = age;
}

void airspeed_init(void)
{
	_i2c = tpu_v2_airspeed_i2c();

	if (_i2c == nullptr) {
		syslog(LOG_ERR, "px4io_sg2000: airspeed I2C bus init failed\n");
		return;
	}

	/* The I2C driver polls, so a transfer keeps the CPU busy for about 1 ms.
	 * Run above the main loop, which never blocks and only yields to equal
	 * priorities, and below the FMU rx_worker so link traffic always wins.
	 */
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setstacksize(&attr, 2048);

	struct sched_param sp;
	sp.sched_priority = sched_get_priority_max(SCHED_RR) - 10;
	(void)pthread_attr_setschedpolicy(&attr, SCHED_RR);
	(void)pthread_attr_setschedparam(&attr, &sp);
	(void)pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

	const int ret = pthread_create(&_thread, &attr, airspeed_worker, nullptr);
	pthread_attr_destroy(&attr);

	if (ret != 0) {
		syslog(LOG_ERR, "px4io_sg2000: failed to start airspeed thread (%d)\n", ret);
	}
}

#endif /* PX4IO_AIRSPEED_I2C_ADDR */
