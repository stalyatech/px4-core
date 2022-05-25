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

#include <lib/drivers/device/i2c.h>

#define BMI088_REG_MASK	(0x00FF)
#define BMI088_REG(r) 	((r) & BMI088_REG_MASK)

class BMI088_I2C : public device::I2C
{
public:
	/**
	 * @ Constructor
	 *
	 * @param device_type	The device type (see drv_sensor.h)
	 * @param name		Driver name
	 * @param bus		I2C bus on which the device lives
	 * @param address	I2C bus address, or zero if set_address will be used
	 * @param frequency	I2C bus frequency for the device (currently not used)
	 */
	BMI088_I2C(uint8_t device_type, const char *name, const int bus, const uint16_t address, const uint32_t frequency, uint32_t stream_length = 64);

	~BMI088_I2C()
	{
		if (_stream_buffer) {
			delete [] _stream_buffer;
		}
	};

	/**
	 * Initialise the driver and make it ready for use.
	 *
	 * @return	OK if the driver initialized OK, negative errno otherwise;
	 */
	virtual int	init() override;

	/**
	 * Read directly from the device.
	 *
	 * The actual size of each unit quantity is device-specific.
	 *
	 * @param reg	The register address at which to start reading
	 * @param data	The buffer into which the read values should be placed.
	 * @param count	The number of items to read.
	 * @return	The number of items read on success, negative errno otherwise.
	 */
	virtual int read(unsigned reg, void *data, unsigned count) override;

	/**
	 * Write directly to the device.
	 *
	 * The actual size of each unit quantity is device-specific.
	 *
	 * @param reg	The register address at which to start writing.
	 * @param data	The buffer from which values should be read.
	 * @param count	The number of items to write.
	 * @return	The number of items written on success, negative errno otherwise.
	 */
	virtual int write(unsigned reg, void *data, unsigned count) override;

	/**
	 * Read a register from the device.
	 *
	 * @param	The register to read.
	 * @return	The value that was read.
	 */
	virtual uint8_t read_reg(unsigned reg) override;

	/**
	 * Write a register to the device.
	 *
	 * @param reg	The register to write.
	 * @param value	The new value to write.
	 * @return	OK on success, negative errno otherwise.
	 */
	virtual int write_reg(unsigned reg, uint8_t value) override;

private:
	uint8_t *_stream_buffer{nullptr};
	uint32_t _stream_length{0};
};

BMI088_I2C::BMI088_I2C(uint8_t device_type, const char *name, const int bus, const uint16_t address, const uint32_t frequency, uint32_t stream_length) :
	I2C(device_type, name, bus, address, frequency),
	_stream_length(stream_length)
{
	if (_stream_length) {
		_stream_buffer = new uint8_t[_stream_length];
	}
}


/**
 * Initialise the driver and make it ready for use.
 *
 * @return	OK if the driver initialized OK, negative errno otherwise;
 */
int BMI088_I2C::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_DEBUG("I2C::init failed (%i)", ret);
	}

	return ret;
}//init


/**
 * Read directly from the device.
 *
 * The actual size of each unit quantity is device-specific.
 *
 * @param reg	The register address at which to start reading
 * @param data	The buffer into which the read values should be placed.
 * @param count	The number of items to read.
 * @return	The number of items read on success, negative errno otherwise.
 */
int BMI088_I2C::read(unsigned reg, void *data, unsigned count)
{
	uint8_t cmd[1];

	/* mask the register */
	cmd[0] = BMI088_REG(reg);

	/* transfer data */
	return transfer(cmd, 1, (uint8_t*)data, count) == PX4_OK ? count : 0;
}//read


/**
 * Write directly to the device.
 *
 * The actual size of each unit quantity is device-specific.
 *
 * @param reg	The register address at which to start writing.
 * @param data	The buffer from which values should be read.
 * @param count	The number of items to write.
 * @return	The number of items written on success, negative errno otherwise.
 */
int BMI088_I2C::write(unsigned reg, void *data, unsigned count)
{
	/* mask the register */
	_stream_buffer[0] = BMI088_REG(reg);
	memcpy(&_stream_buffer[1], data, count);

	/* transfer data */
	return transfer(_stream_buffer, count + 1, 0, 0) == PX4_OK ? count : 0;
}//write


/**
 * Read a register from the device.
 *
 * @param	The register to read.
 * @return	The value that was read.
 */
uint8_t BMI088_I2C::read_reg(unsigned reg)
{
	uint8_t cmd[1];
	uint8_t data[1];

	/* mask the register */
	cmd[0] = BMI088_REG(reg);

	/* transfer data */
	transfer(cmd, 1, data, 1);

	/* return with data */
	return data[0];
}//read_reg


/**
 * Write a register to the device.
 *
 * @param reg	The register to write.
 * @param value	The new value to write.
 * @return	OK on success, negative errno otherwise.
 */
int BMI088_I2C::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[2];

	/* mask the register */
	cmd[0] = BMI088_REG(reg);
	cmd[1] = value;

	/* transfer data */
	return transfer(cmd, 2, nullptr, 0);
}//write_reg


device::Device *BMI088_I2C_interface(uint8_t device_type, const char *name, const int bus, const uint16_t address, const uint32_t frequency)
{
	return new BMI088_I2C(device_type, name, bus, address, frequency);
}
