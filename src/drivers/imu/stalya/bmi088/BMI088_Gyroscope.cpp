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

#include "BMI088_Gyroscope.hpp"

using namespace time_literals;

namespace Bosch::BMI088::Gyroscope
{

/*******************************************************************************
* Function Name  : Constructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
BMI088_Gyroscope::BMI088_Gyroscope(Device *interface, enum Rotation rotation) :
	_interface(interface),
	_px4_gyro(interface->get_device_id(), rotation)
{
	/* Initialize the SPI/I2C interface first */
	_interface->init();

	/* set the device type */
	_interface->set_device_type(DRV_GYR_DEVTYPE_BMI088);
	_px4_gyro.set_device_type(DRV_GYR_DEVTYPE_BMI088);
}


/*******************************************************************************
* Function Name  : Probe
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Gyroscope::Probe()
{
	const uint8_t chipid = RegisterRead(Register::GYRO_CHIP_ID);

	if (chipid != ID) {
		PX4_ERR("unexpected GYRO_CHIP_ID 0x%02x", chipid);
		return PX4_ERROR;
	}

	return PX4_OK;
}//Probe


/*******************************************************************************
* Function Name  : Reset
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Gyroscope::Reset()
{
        /* Reset gyro device */
        if (RegisterWrite(Register::GYRO_SOFTRESET, 0xB6) == PX4_OK) {

		/* Delay 30 ms after reset value is written to its register */
		px4_usleep(30000);

		return PX4_OK;
	}//if

	return PX4_ERROR;
}//Reset


/*******************************************************************************
* Function Name  : Check
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Gyroscope::Check()
{
	// check configuration register
	if (RegisterCheck(_register_cfg[_checked_register])) {
		_checked_register = (_checked_register + 1) % size_register_cfg;

		return PX4_OK;
	}

	return PX4_ERROR;
}//BMI088_Gyroscope


/*******************************************************************************
* Function Name  : ReadData
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Gyroscope::ReadData(int16_t *gyro)
{
	uint8_t data[6];
	uint8_t lsb, msb;
	int16_t msblsb;

	/* Read gyro sensor data */
	if (StreamRead(Register::RATE_X_LSB, &data[0], 6) == 6) {

		/* sensor's frame is +x forward, +y left, +z up
		   publish right handed with x forward, y right, z down */

		lsb = data[0];
		msb = data[1];
		msblsb = (msb << 8) | lsb;
		gyro[0] = msblsb;  					/* Data in X axis */

		lsb = data[2];
		msb = data[3];
		msblsb = (msb << 8) | lsb;
		gyro[1] = (msblsb == INT16_MIN) ? INT16_MAX : -msblsb; 	/* Data in Y axis */

		lsb = data[4];
		msb = data[5];
		msblsb = (msb << 8) | lsb;
		gyro[2] = (msblsb == INT16_MIN) ? INT16_MAX : -msblsb; 	/* Data in Z axis */

		return PX4_OK;
	}

	return PX4_ERROR;
}//ReadData


/*******************************************************************************
* Function Name  : ReadFIFO
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Gyroscope::ReadFIFO(const hrt_abstime &timestamp_sample, sensor_gyro_fifo_s &gyro)
{
	/* prepare the gyroscope data */
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;
	gyro.dt = FIFO_SAMPLE_DT;

	 /* Read FIFO Status */
	uint8_t n_frames = RegisterRead(Register::FIFO_STATUS) & 0x7F;

	int n_frames_to_read = 6;

	/* don't read more than 6 frames at a time */
	if (n_frames > n_frames_to_read) {
		n_frames = n_frames_to_read;
	}

	if (n_frames == 0) {
		return PX4_ERROR;
	}

	int frame_size = 6 * n_frames;
	uint8_t data[frame_size];

	/* Read FIFO data */
	if (StreamRead(Register::FIFO_DATA, data, frame_size) == frame_size) {

		for (uint8_t i = 0; i < n_frames; i++) {
			const uint8_t *d = &data[i * 6];
			int16_t xyz[3] {
				int16_t(uint16_t(d[0] | d[1] << 8)),
				int16_t(uint16_t(d[2] | d[3] << 8)),
				int16_t(uint16_t(d[4] | d[5] << 8))
			};

			gyro.x[i] = xyz[0];
			gyro.y[i] = (xyz[1] == INT16_MIN) ? INT16_MAX : -xyz[1];
			gyro.z[i] = (xyz[2] == INT16_MIN) ? INT16_MAX : -xyz[2];
			gyro.samples++;
		}

		return PX4_OK;
	}

	return PX4_ERROR;
}//ReadFIFO


/*******************************************************************************
* Function Name  : Configure
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool BMI088_Gyroscope::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
		px4_usleep(1000);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	ConfigureGyro();

	return success;
}//Configure


/*******************************************************************************
* Function Name  : ConfigureGyro
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BMI088_Gyroscope::ConfigureGyro()
{
	const uint8_t GYRO_RANGE = RegisterRead(Register::GYRO_RANGE) & (Bit3 | Bit2 | Bit1 | Bit0);

	switch (GYRO_RANGE) {
	case gyro_range_2000_dps:
		_px4_gyro.set_scale(math::radians(1.f / 16.384f));
		_px4_gyro.set_range(math::radians(2000.f));
		break;

	case gyro_range_1000_dps:
		_px4_gyro.set_scale(math::radians(1.f / 32.768f));
		_px4_gyro.set_range(math::radians(1000.f));
		break;

	case gyro_range_500_dps:
		_px4_gyro.set_scale(math::radians(1.f / 65.536f));
		_px4_gyro.set_range(math::radians(500.f));
		break;

	case gyro_range_250_dps:
		_px4_gyro.set_scale(math::radians(1.f / 131.072f));
		_px4_gyro.set_range(math::radians(250.f));
		break;

	case gyro_range_125_dps:
		_px4_gyro.set_scale(math::radians(1.f / 262.144f));
		_px4_gyro.set_range(math::radians(125.f));
		break;
	}
}//ConfigureGyro


/*******************************************************************************
* Function Name  : RegisterRead
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t BMI088_Gyroscope::RegisterRead(Register reg)
{
	return _interface->read_reg(static_cast<unsigned>(reg));
}//RegisterRead


/*******************************************************************************
* Function Name  : RegisterWrite
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Gyroscope::RegisterWrite(Register reg, const uint8_t value)
{
	return _interface->write_reg(static_cast<unsigned>(reg), value);
}//RegisterWrite


/*******************************************************************************
* Function Name  : RegisterSetAndClearBits
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BMI088_Gyroscope::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}//RegisterSetAndClearBits


/*******************************************************************************
* Function Name  : RegisterCheck
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool BMI088_Gyroscope::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}//RegisterCheck


/*******************************************************************************
* Function Name  : StreamRead
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Gyroscope::StreamRead(Register reg, void *data, unsigned count)
{
	return _interface->read(static_cast<unsigned>(reg), data, count);
}//StreamRead


/*******************************************************************************
* Function Name  : StreamWrite
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Gyroscope::StreamWrite(Register reg, const void *data, unsigned count)
{
	return _interface->write(static_cast<unsigned>(reg), (void*)data, count);
}//StreamWrite

} // namespace Bosch::BMI088::Gyroscope
