/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "ICM20689.hpp"
#include <drivers/device/spi.h>

using namespace InvenSense_ICM20689;

class ICM20689_SPI : public device::SPI
{
public:
	ICM20689_SPI(int bus, int bus_frequency, uint32_t devid, spi_mode_e spi_mode);
	virtual ~ICM20689_SPI() = default;

	virtual int init();
	virtual int read(unsigned address, void *data, unsigned count);
	virtual int write(unsigned address, void *data, unsigned count);

private:
	uint8_t _buffer[FIFO::SIZE+4];
};


/*******************************************************************************
* Function Name  : ICM20689_SPI_interface
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
device::Device *ICM20689_SPI_interface(int bus, int bus_frequency, uint32_t devid, spi_mode_e spi_mode)
{
	return new ICM20689_SPI(bus, bus_frequency, devid, spi_mode);
}//ICM20689_SPI_interface


/*******************************************************************************
* Function Name  : Constructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
ICM20689_SPI::ICM20689_SPI(int bus, int bus_frequency, uint32_t devid, spi_mode_e spi_mode) :
	SPI(DRV_IMU_DEVTYPE_ICM20689, MODULE_NAME, bus, devid, spi_mode, bus_frequency)
{
}


/*******************************************************************************
* Function Name  : init
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int ICM20689_SPI::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	/* read WHO_AM_I value */
	uint8_t data = 0;

	if (read(static_cast<unsigned int>(Register::WHO_AM_I), &data, 1) != PX4_OK) {
		DEVICE_DEBUG("ICM20689 read_reg fail");
	}

	if (data != WHOAMI) {
		DEVICE_DEBUG("ICM20689 bad ID: %02x", data);
		return -EIO;
	}

	return PX4_OK;
}//init


/*******************************************************************************
* Function Name  : read
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int ICM20689_SPI::read(unsigned address, void *data, unsigned count)
{
	int ret;

	/* buffer bound check */
	if (sizeof(_buffer) < (count + 4)) {
		return -EIO;
	}

	/* mask the register */
	_buffer[0] = static_cast<uint8_t>(address | DIR_READ);
	_buffer[1] = 0;

	/* transfer data */
	if ((ret = transfer(_buffer, _buffer, count + 1)) == PX4_OK) {
		memcpy(data, &_buffer[1], count);
	}

	return ret;
}//read


/*******************************************************************************
* Function Name  : write
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int ICM20689_SPI::write(unsigned address, void *data, unsigned count)
{
	/* buffer bound check */
	if (sizeof(_buffer) < (count + 4)) {
		return -EIO;
	}

	/* mask the register */
	_buffer[0] = address;
	memcpy(&_buffer[1], data, count);

	/* transfer data */
	return transfer(_buffer, nullptr, count + 1);
}//write
