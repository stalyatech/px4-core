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

#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include "Bosch_BMI088_Accelerometer_Registers.hpp"

using namespace device;

namespace Bosch::BMI088::Accelerometer
{

class BMI088_Accelerometer
{
public:
	BMI088_Accelerometer(Device *interface, enum Rotation rotation);
	~BMI088_Accelerometer() {};

	int  Probe();
	int  Reset();
	int  Check();
	int  ReadData(int16_t *accel);
	int  ReadTemperature(float *temp);
	bool Configure();
	int  ConfigureSyncMode();

	inline void Update(const hrt_abstime &timestamp_sample, float x, float y, float z) {
		_px4_accel.update(timestamp_sample, x, y, z);
	}

	inline void SetErrorCount(uint32_t error_count) {
		_px4_accel.set_error_count(error_count);
	}

	inline void SetTemperature(float temperature) {
		_px4_accel.set_temperature(temperature);
	}

private:
	Device 	        *_interface{nullptr};
	PX4Accelerometer _px4_accel;

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{10};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                        | Set bits, Clear bits
		{ Register::ACC_PWR_CONF,          0, ACC_PWR_CONF_BIT::acc_pwr_save },
		{ Register::ACC_PWR_CTRL,          ACC_PWR_CTRL_BIT::acc_enable, 0 },
		{ Register::ACC_CONF,              ACC_CONF_BIT::acc_bwp_Normal | ACC_CONF_BIT::acc_odr_400, Bit2 | Bit0 },
		{ Register::ACC_RANGE,             ACC_RANGE_BIT::acc_range_12g, Bit0 },
		{ Register::INT1_IO_CONF,          INT1_IO_CONF_BIT::int1_in | INT1_IO_CONF_BIT::int1_lvl, 0 },
		{ Register::INT2_IO_CONF,          INT2_IO_CONF_BIT::int2_out | INT2_IO_CONF_BIT::int2_lvl, 0 },
		{ Register::INT1_INT2_MAP_DATA,    INT1_INT2_MAP_DATA_BIT::int1_drdy | INT1_INT2_MAP_DATA_BIT::int2_drdy, 0},
	};

	void 	ConfigureAccel();
	uint8_t RegisterRead(Register reg);
	int 	RegisterWrite(Register reg, const uint8_t value);
	void 	RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);
	bool 	RegisterCheck(const register_config_t &reg_cfg);
	int 	StreamRead(Register reg, void *data, unsigned count);
	int 	StreamWrite(Register reg, const void *data, unsigned count);
	int 	FeatureWrite(Register reg, const void *data, unsigned count);
	int 	BurstWrite(const void *burst_data, unsigned index, int burst_len);
};

} // namespace Bosch::BMI088::Accelerometer
