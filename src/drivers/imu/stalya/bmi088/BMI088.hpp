/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include "BMI088_Accelerometer.hpp"
#include "BMI088_Gyroscope.hpp"

using namespace device;
using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb) { return (msb << 8u) | lsb; }

Device *BMI088_SPI_interface(uint8_t device_type, const char *name, const int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency, bool dummy_read);
Device *BMI088_I2C_interface(uint8_t device_type, const char *name, const int bus, const uint16_t address, const uint32_t frequency);

class BMI088 : public I2CSPIDriver<BMI088>
{
public:
	BMI088(Device *interface_accel, Device *interface_gyro, I2CSPIBusOption bus_option, int bus, enum Rotation rotation,
		int i2c_address = 0, spi_drdy_gpio_t drdy_gpio = 0);
	~BMI088() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void print_status();

	bool Reset();

	int  Init();

	void RunImpl();

private:
	Bosch::BMI088::Accelerometer::BMI088_Accelerometer *_accel{nullptr};
	Bosch::BMI088::Gyroscope::BMI088_Gyroscope *_gyro{nullptr};

	struct sensor_data_t {
		int16_t x;
		int16_t y;
		int16_t z;
	};

	sensor_data_t	_accel_data{0};
	sensor_data_t	_gyro_data{0};

	uint32_t    	_drdy_gpio{0};

	perf_counter_t	_sample_perf{nullptr};
	perf_counter_t	_bad_transfer_perf{nullptr};
	perf_counter_t	_bad_register_perf{nullptr};
	perf_counter_t	_good_transfer_perf{nullptr};

	hrt_abstime 	_temperature_update_timestamp{0};
	hrt_abstime 	_last_config_check_timestamp{0};

	enum class STATE : uint8_t {
		INIT,
		READ,
		FIFO_READ,
	};

	STATE _state{STATE::INIT};

	void exit_and_cleanup() override;

	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();
	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
};
