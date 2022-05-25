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
#include <drivers/device/i2c.h>

using namespace InvenSense_ICM20689;

class ICM20689_I2C : public device::I2C
{
public:
	ICM20689_I2C(int bus, int bus_frequency, int bus_address);
	virtual ~ICM20689_I2C() = default;

	virtual int	init();
	virtual int read(unsigned address, void *data, unsigned count);
	virtual int write(unsigned address, void *data, unsigned count);

protected:
	virtual int probe();
};


/*******************************************************************************
* Function Name  : ICM20689_I2C_interface
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
device::Device *ICM20689_I2C_interface(int bus, int bus_frequency, int bus_address)
{
	return new ICM20689_I2C(bus, bus_frequency, bus_address);
}//ICM20689_I2C_interface


/*******************************************************************************
* Function Name  : Constructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
ICM20689_I2C::ICM20689_I2C(int bus, int bus_frequency, int bus_address) :
	I2C(DRV_IMU_DEVTYPE_ICM20689, MODULE_NAME, bus, bus_address, bus_frequency)
{
}


/*******************************************************************************
* Function Name  : init
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int ICM20689_I2C::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_DEBUG("I2C::init failed (%i)", ret);
	}

	return ret;
}//init


/*******************************************************************************
* Function Name  : probe
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int ICM20689_I2C::probe()
{
	uint8_t data = 0;

	_retries = 10;
	if (read(static_cast<unsigned int>(Register::WHO_AM_I), &data, 1) != PX4_OK) {
		DEVICE_DEBUG("read_reg fail");
		return -EIO;
	}

	_retries = 2;
	if (data != WHOAMI) {
		DEVICE_DEBUG("ICM20689 bad ID: %02x", data);
		return -EIO;
	}

	return PX4_OK;
}//probe


/*******************************************************************************
* Function Name  : read
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int ICM20689_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}//read


/*******************************************************************************
* Function Name  : write
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int ICM20689_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}//write
