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

#include "BMI088_Accelerometer.hpp"

#include <ecl/geo/geo.h> // CONSTANTS_ONE_G

using namespace time_literals;

namespace Bosch::BMI088::Accelerometer
{

/*******************************************************************************
* Function Name  : Constructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
BMI088_Accelerometer::BMI088_Accelerometer(Device *interface, enum Rotation rotation) :
	_interface(interface),
	_px4_accel(interface->get_device_id(), rotation)
{
	/* Initialize the SPI/I2C interface first */
	_interface->init();

	/* set the device type */
	_interface->set_device_type(DRV_ACC_DEVTYPE_BMI088);
	_px4_accel.set_device_type(DRV_ACC_DEVTYPE_BMI088);
}


/*******************************************************************************
* Function Name  : Probe
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Accelerometer::Probe()
{
	const uint8_t chipid = RegisterRead(Register::ACC_CHIP_ID);

	if (chipid != ID) {
		PX4_ERR("unexpected ACC_CHIP_ID 0x%02x", chipid);
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
int BMI088_Accelerometer::Reset()
{
        /* Reset accel device */
        if (RegisterWrite(Register::ACC_SOFTRESET, 0xB6) == PX4_OK) {

		/* Delay 10 ms after reset value is written to its register */
		px4_usleep(10000);

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
int BMI088_Accelerometer::Check()
{
	// check configuration register
	if (RegisterCheck(_register_cfg[_checked_register])) {
		_checked_register = (_checked_register + 1) % size_register_cfg;

		return PX4_OK;
	}

	return PX4_ERROR;
}//Check


/*******************************************************************************
* Function Name  : ReadData
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Accelerometer::ReadData(int16_t *accel)
{
	uint8_t data[6];
	uint8_t lsb, msb;
	int16_t msblsb;

	 /* Read accel x,y sensor data */
	if (StreamRead(Register::ACC_GP_0, &data[0], 4) == 4) {

		/* Read accel sensor data */
		if (StreamRead(Register::ACC_GP_4, &data[4], 2) == 2) {

			/* sensor's frame is +x forward, +y left, +z up
			   publish right handed with x forward, y right, z down */

			lsb = data[0];
			msb = data[1];
			msblsb = (msb << 8) | lsb;
			accel[0] = msblsb;  					/* Data in X axis */

			lsb = data[2];
			msb = data[3];
			msblsb = (msb << 8) | lsb;
			accel[1] = (msblsb == INT16_MIN) ? INT16_MAX : -msblsb;	/* Data in Y axis */

			lsb = data[4];
			msb = data[5];
			msblsb = (msb << 8) | lsb;
			accel[2] = (msblsb == INT16_MIN) ? INT16_MAX : -msblsb;	/* Data in Z axis */

			return PX4_OK;
		}
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
int BMI088_Accelerometer::ReadFIFO(const hrt_abstime &timestamp_sample, sensor_accel_fifo_s &accel)
{
	uint16_t fifo_fill_level;

	/* prepare the acceleration data */
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT;

	 /* Read FIFO length */
	if (StreamRead(Register::FIFO_LENGTH_0, &fifo_fill_level, 2) == 2) {

		/* Length check */
		if (fifo_fill_level & 0x8000) {
			return PX4_ERROR;
		}

		int n_frames_to_read = 6;

		/* don't read more than 6 frames at a time */
		if (fifo_fill_level > n_frames_to_read * 7) {
			fifo_fill_level = n_frames_to_read * 7;
		}

		if (fifo_fill_level == 0) {
			return PX4_ERROR;
		}

		uint8_t data[fifo_fill_level];

		/* Read FIFO data */
		if (StreamRead(Register::FIFO_DATA, data, fifo_fill_level) == fifo_fill_level) {

			const uint8_t *p = &data[0];

			while (fifo_fill_level >= 7)
			{
				uint8_t frame_len = 2;

				switch (p[0] & 0xFC)
				{
					/* acceleration sensor data frame */
					case 0x84:
					{
						frame_len = 7;
						const uint8_t *d = p + 1;
						int16_t xyz[3] {
							int16_t(uint16_t(d[0] | (d[1] << 8))),
							int16_t(uint16_t(d[2] | (d[3] << 8))),
							int16_t(uint16_t(d[4] | (d[5] << 8)))
						};

						const int16_t tX[3] = {1, 0, 0};
						const int16_t tY[3] = {0, -1, 0};
						const int16_t tZ[3] = {0, 0, -1};

						float x = 0;
						float y = 0;
						float z = 0;

						x = xyz[0] * tX[0] + xyz[1] * tX[1] + xyz[2] * tX[2];
						y = xyz[0] * tY[0] + xyz[1] * tY[1] + xyz[2] * tY[2];
						z = xyz[0] * tZ[0] + xyz[1] * tZ[1] + xyz[2] * tZ[2];

						accel.x[accel.samples] = x;
						accel.y[accel.samples] = y;
						accel.z[accel.samples] = z;
						accel.samples++;
						break;
					}

					/* skip frame */
					case 0x40:
					{
						frame_len = 2;
						break;
					}

					/* sensortime frame */
					case 0x44:
					{
						frame_len = 4;
						break;
					}

					/* fifo config frame */
					case 0x48:
					{
						frame_len = 2;
						break;
					}

					/* sample drop frame */
					case 0x50:
					{
						frame_len = 2;
						break;
					}
				}//switch

				p += frame_len;
				fifo_fill_level -= frame_len;
			}//while

			return PX4_OK;
		}//if
	}//if

	return PX4_ERROR;
}//ReadFIFO


/*******************************************************************************
* Function Name  : ReadTemperature
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Accelerometer::ReadTemperature(float *temp)
{
	// stored in an 11-bit value in 2’s complement format
	uint8_t temperature_buf[2] {};

	if (StreamRead(Register::TEMP_MSB, temperature_buf, 2) == 2) {

		const uint8_t TEMP_MSB = temperature_buf[0];
		const uint8_t TEMP_LSB = temperature_buf[1];

		// Datasheet 5.3.7: Register 0x22 – 0x23: Temperature sensor data
		uint16_t Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32);
		int16_t Temp_int11 = 0;

		if (Temp_uint11 > 1023) {
			Temp_int11 = Temp_uint11 - 2048;
		} else {
			Temp_int11 = Temp_uint11;
		}

		// Temp_int11 * 0.125°C/LSB + 23°C
		float temperature = (Temp_int11 * 0.125f) + 23.f;

		if (PX4_ISFINITE(temperature)) {
			*temp = temperature;

			return PX4_OK;
		}
	}

	return PX4_ERROR;
}//ReadTemperature


/*******************************************************************************
* Function Name  : Configure
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool BMI088_Accelerometer::Configure()
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

	ConfigureAccel();

	return success;
}//Configure


/*******************************************************************************
* Function Name  : ConfigureAccel
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BMI088_Accelerometer::ConfigureAccel()
{
	const uint8_t ACC_RANGE = RegisterRead(Register::ACC_RANGE) & (Bit1 | Bit0);

	switch (ACC_RANGE) {
	case acc_range_3g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(3.f * CONSTANTS_ONE_G);
		break;

	case acc_range_6g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(6.f * CONSTANTS_ONE_G);
		break;

	case acc_range_12g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(12.f * CONSTANTS_ONE_G);
		break;

	case acc_range_24g:
		_px4_accel.set_scale(CONSTANTS_ONE_G * (powf(2, ACC_RANGE + 1) * 1.5f) / 32768.f);
		_px4_accel.set_range(24.f * CONSTANTS_ONE_G);
		break;
	}
}//ConfigureAccel


/*******************************************************************************
* Function Name  : ConfigureSyncMode
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Accelerometer::ConfigureSyncMode()
{
	/*! Config stream data buffer address will be assigned */
	const uint8_t *config_file_ptr = bmi08x_config_file;
	uint8_t current_acc_pwr_ctrl = 0;
	uint16_t index = 0;
	uint8_t reg_data = 0;

	/* deactivate accel, otherwise post processing can not be enabled safely */
	current_acc_pwr_ctrl = RegisterRead(Register::ACC_PWR_CTRL);
	if (RegisterWrite(Register::ACC_PWR_CTRL, 0) != PX4_OK) {
		return PX4_ERROR;
	}

	/* delay required to switch power modes */
	px4_usleep(5000);

	/* Disable config loading */
	if (RegisterWrite(Register::INIT_CTRL, 0) != PX4_OK) {
		return PX4_ERROR;
	}

	/* Transfer the config file */
	for (index = 0; index < CONFIG_STREAM_SIZE; index += 32) {
		/* Write the config stream */
		if (BurstWrite(config_file_ptr + index, index, 32) != PX4_OK) {
			return PX4_ERROR;
		}
	}

	/* Enable config loading and FIFO mode */
	if (RegisterWrite(Register::INIT_CTRL, 1) != PX4_OK) {
		return PX4_ERROR;
	}

	/* Wait till ASIC is initialized. Refer the data-sheet for more information */
	px4_usleep(150000);

	/* Check for config initialization status (1 = OK) */
	reg_data = RegisterRead(Register::INTERNAL_STAT);
	if (reg_data != 1) {
		return PX4_ERROR;
	} else {
		/* reactivate accel */
		if (RegisterWrite(Register::ACC_PWR_CTRL, current_acc_pwr_ctrl) != PX4_OK) {
			return PX4_ERROR;
		}

		/* delay required to switch power modes */
		px4_usleep(5000);
	}

        /* Enable data synchronization */
	uint16_t sync_data = DataSyncMode::sync_2000_Hz;

	return FeatureWrite(Register::DATA_SYNC_ADR, &sync_data, sizeof(sync_data));
}//ConfigureSyncMode


/*******************************************************************************
* Function Name  : RegisterRead
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint8_t BMI088_Accelerometer::RegisterRead(Register reg)
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
int BMI088_Accelerometer::RegisterWrite(Register reg, const uint8_t value)
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
void BMI088_Accelerometer::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
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
bool BMI088_Accelerometer::RegisterCheck(const register_config_t &reg_cfg)
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
int BMI088_Accelerometer::StreamRead(Register reg, void *data, unsigned count)
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
int BMI088_Accelerometer::StreamWrite(Register reg, const void *data, unsigned count)
{
	return _interface->write(static_cast<unsigned>(reg), (void*)data, count);
}//StreamWrite


/*******************************************************************************
* Function Name  : FeatureWrite
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Accelerometer::FeatureWrite(Register reg, const void *data, unsigned count)
{
	uint8_t   reg_addr = static_cast<uint8_t>(reg);
	uint16_t *reg_data = (uint16_t*)data;
	uint16_t  read_length = (reg_addr * 2) + (count * 2);
	uint8_t  *feature_data = new uint8_t(read_length);
	int ret = PX4_ERROR;

        /* Read feature space up to the given feature position */
	if (StreamRead(Register::FEATURE_CFG, &feature_data[0], read_length) == read_length)
        {
            /* Apply the given feature config. */
            for (unsigned i = 0; i < count; ++i) {
                /* Be careful: the feature config space is 16bit aligned! */
                feature_data[(reg_addr * 2) + (i * 2)] = reg_data[i] & 0xFF;
                feature_data[(reg_addr * 2) + (i * 2) + 1] = reg_data[i] >> 8;
            }

            /* Write back updated feature space */
	    ret = StreamWrite(Register::FEATURE_CFG, &feature_data[0], read_length) == read_length ? PX4_OK : PX4_ERROR;
        }//if

    	delete [] feature_data;

	return ret;
}//FeatureWrite


/*******************************************************************************
* Function Name  : BurstWrite
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088_Accelerometer::BurstWrite(const void *burst_data, unsigned index, int burst_len)
{
	uint8_t asic_msb = (uint8_t)((index / 2) >> 4);
	uint8_t asic_lsb = ((index / 2) & 0x0F);

	/* Write to feature config register */
	if (RegisterWrite(Register::RESERVED_5B, asic_lsb) == PX4_OK) {

		/* Write to feature config register */
		if (RegisterWrite(Register::RESERVED_5C, asic_msb) == PX4_OK) {

			/* Write to feature config registers */
			return StreamWrite(Register::FEATURE_CFG, burst_data, burst_len) == burst_len ? PX4_OK : PX4_ERROR;
		}
	}

	return PX4_ERROR;
}//BurstWrite

} // namespace Bosch::BMI088::Accelerometer
