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

#include <lib/drivers/device/spi.h>

#define BMI088_REG_MASK	(0x00FF)
#define BMI088_REG(r) 	((r) & BMI088_REG_MASK)

/* SPI protocol address bits */
#define DIR_READ(a)	((a) | 0x80)
#define DIR_WRITE(a)    ((a) & 0x7f)

class BMI088_SPI : public device::SPI
{
public:
	/**
	 * Constructor
	 *
	 * @param device_type	The device type (see drv_sensor.h)
	 * @param name		Driver name
	 * @param bus		SPI bus on which the device lives
	 * @param device	Device handle (used by SPI_SELECT)
	 * @param mode		SPI clock/data mode
	 * @param frequency	SPI clock frequency
	 */
	BMI088_SPI(uint8_t device_type, const char *name, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency, bool dummy_read = false, uint32_t stream_length = 64);

	~BMI088_SPI()
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
	bool _dummy_read{false};
	uint32_t _stream_length{0};
	uint8_t *_stream_buffer{nullptr};
};

BMI088_SPI::BMI088_SPI(uint8_t device_type, const char *name, const int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency, bool dummy_read, uint32_t stream_length) :
	SPI(device_type, name, bus, device, mode, frequency),
	_dummy_read(dummy_read),
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
int BMI088_SPI::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		PX4_DEBUG("SPI init failed");
		return -EIO;
	}

	return PX4_OK;
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
int BMI088_SPI::read(unsigned reg, void *data, unsigned count)
{
	int ret;

	/* mask the register */
	_stream_buffer[0] = DIR_READ(BMI088_REG(reg));
	_stream_buffer[1] = 0;

	/* transfer data */
	if (_dummy_read) {
		if ((ret = transfer(_stream_buffer, _stream_buffer, count + 2)) == PX4_OK) {
			memcpy(data, &_stream_buffer[2], count);
		}
	} else {
		if ((ret = transfer(_stream_buffer, _stream_buffer, count + 1)) == PX4_OK) {
			memcpy(data, &_stream_buffer[1], count);
		}
	}

	return (ret == PX4_OK) ? count : 0;
}

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
int BMI088_SPI::write(unsigned reg, void *data, unsigned count)
{
	/* mask the register */
	_stream_buffer[0] = DIR_WRITE(BMI088_REG(reg));
	memcpy(&_stream_buffer[1], data, count);

	/* transfer data */
	return transfer(_stream_buffer, nullptr, count + 1) == PX4_OK ? count : 0;
}

/**
 * Read a register from the device.
 *
 * @param	The register to read.
 * @return	The value that was read.
 */
uint8_t BMI088_SPI::read_reg(unsigned reg)
{
	uint8_t cmd[3] {};

	/* mask the register */
	cmd[0] = DIR_READ(BMI088_REG(reg));

	/* transfer data */
	if (_dummy_read) {
		if (transfer(cmd, cmd, 3) == PX4_OK) {
			/* return with data */
			return cmd[2];
		}
	} else {
		if (transfer(cmd, cmd, 2) == PX4_OK) {
			/* return with data */
			return cmd[1];
		}
	}

	/* return with null */
	return 0;
}

/**
 * Write a register to the device.
 *
 * @param reg	The register to write.
 * @param value	The new value to write.
 * @return	OK on success, negative errno otherwise.
 */
int BMI088_SPI::write_reg(unsigned reg, uint8_t value)
{
	uint8_t cmd[2];

	/* mask the register */
	cmd[0] = DIR_WRITE(BMI088_REG(reg));
	cmd[1] = value;

	/* transfer data */
	return transfer(cmd, nullptr, 2);
}

device::Device *BMI088_SPI_interface(uint8_t device_type, const char *name, int bus, uint32_t device, enum spi_mode_e mode, uint32_t frequency, bool dummy_read)
{
	return new BMI088_SPI(device_type, name, bus, device, mode, frequency, dummy_read);
}
