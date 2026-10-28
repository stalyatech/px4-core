/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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

/**
 * @file pca9557.cpp
 *
 * Driver for the PCA9557 I2C I/O expander module
 *
 * @author volvox <volvox@stalya.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <drivers/device/device.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_ioe.h>

using namespace time_literals;

/* I2C Device Address 8 bit format */
#define IOE_DEV_ADDR    (0x18)

/* Device Register Map */
enum
{
    IOE_REG_INPUT,
    IOE_REG_OUTPUT,
    IOE_REG_INVERS,
    IOE_REG_CONFIG,
};

/* PCA9557 I/O Mapper */
class PCA9557Dev : public cdev::CDev
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	PCA9557Dev() : cdev::CDev(IOE_DEVICE_PATH) { };

	/**
	 * Destructor.
	 *
	 * Wait for worker thread to terminate.
	 */
	virtual ~PCA9557Dev() override = default;
};

/* PCA9557 I2C Driver */
class PCA9557 : public device::I2C, public I2CSPIDriver<PCA9557>, public PCA9557Dev
{
public:
	PCA9557(const I2CSPIDriverConfig &config);
	~PCA9557() override = default;

	static void print_usage();

	int  init() override;
	void print_status() override;
	int  reset();

	void RunImpl();

protected:
	void custom_method(const BusCLIArguments &cli) override;

private:

	uint32_t _outputs;
	uint32_t _inputs;

	/* Times the direction register was found reset and written again */
	uint32_t _restores{0};

	/* Pin direction register value: all eight pins are outputs */
	static constexpr uint8_t CONFIG_ALL_OUTPUTS{0x00};

	/* Read one register */
	int read_reg(uint8_t reg, uint8_t &value);

	/* Write outputs and directions, then read the directions back */
	int apply();

	int probe() override;

	/* Configure the I/O ports */
	int config();

	/* Write to the output register */
	int write(uint32_t outputs);

	/* Read from the input register */
	int read(uint32_t& inputs);

	/* Perform an ioctl operation on the device */
	virtual int	ioctl(file *filp, int cmd, unsigned long arg) override;
};


/*******************************************************************************
* Function Name  : Constructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
PCA9557::PCA9557(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_outputs(0),
	_inputs(0)
{
#if defined(IOE_RST_GPIO_PIN)
	// Configure reset pin
	px4_arch_configgpio(IOE_RST_GPIO_PIN);
#endif
}


/*******************************************************************************
* Function Name  : probe
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int PCA9557::probe()
{
	return PX4_OK;
}//probe


/*******************************************************************************
* Function Name  : init
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int PCA9557::init()
{
	int ret;

	ret = PCA9557Dev::init();
	if (ret != OK) {
		return ret;
	}

	ret = I2C::init();
	if (ret != OK) {
		return ret;
	}

	ret = reset();
	if (ret != OK) {
		return ret;
	}

	//---------------------------------------------------------------
	// Default output values
	//---------------------------------------------------------------
	// IOP_PWR  = 1
	// USB_PWR  = 0
	// PER_PWR  = 1
	// IMU_HEAT = 0
	// IMU_PWR  = 1
	// GPS_PWR  = 0
	// RTCM_SEL = 0
	// XBE_PWR  = 1
	//---------------------------------------------------------------
	_outputs = 0xA9;
	ret = apply();
	if (ret != OK) {
		return ret;
	}

	// The expander loses its registers if its reset line is pulsed by
	// anything else (e.g. a wrong board revision): check it once a second.
	ScheduleOnInterval(1_s);

	return ret;
}//init


/*******************************************************************************
* Function Name  : apply
* Description    : Write the outputs first so the pins come up at the right
*                  level, then the directions, and read the directions back
* Input          : None
* Output         : None
* Return         : PX4_OK or the transfer error
*******************************************************************************/
int PCA9557::apply()
{
	int ret = write(_outputs);

	if (ret == PX4_OK) {
		ret = config();
	}

	if (ret == PX4_OK) {
		uint8_t dir = 0xff;
		ret = read_reg(IOE_REG_CONFIG, dir);

		if (ret == PX4_OK && dir != CONFIG_ALL_OUTPUTS) {
			PX4_ERR("direction register reads 0x%02x after writing it", dir);
			ret = PX4_ERROR;
		}
	}

	return ret;
}//apply


/*******************************************************************************
* Function Name  : read_reg
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int PCA9557::read_reg(uint8_t reg, uint8_t &value)
{
	return transfer(&reg, 1, &value, 1);
}//read_reg


/*******************************************************************************
* Function Name  : reset
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int PCA9557::reset(void)
{
#if defined(IOE_RST_GPIO_PIN)
	px4_arch_gpiowrite(IOE_RST_GPIO_PIN, false);
	px4_usleep(10000);
	px4_arch_gpiowrite(IOE_RST_GPIO_PIN, true);
	px4_usleep(1000);
#endif

	return PX4_OK;
}//reset


/*******************************************************************************
* Function Name  : config
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int PCA9557::config()
{
	/* configure the I/O pins*/
	uint8_t cmd[2] = { IOE_REG_CONFIG, CONFIG_ALL_OUTPUTS };

	/* send config word to the device  */
	int ret = transfer(cmd, 2, nullptr, 0);

	if (PX4_OK != ret) {
		PX4_DEBUG("config : i2c::transfer returned %d", ret);
	}

	return ret;
}//config


/*******************************************************************************
* Function Name  : write
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int PCA9557::read(uint32_t& inputs)
{
	uint8_t data;

	/* read the input register */
	uint8_t cmd[1] = { IOE_REG_INPUT };

	/* send config word to the device  */
	int ret = transfer(cmd, 1, &data, 1);

	if (PX4_OK != ret) {
		PX4_DEBUG("read : i2c::transfer returned %d", ret);
	} else {
		inputs = data;
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
int PCA9557::write(uint32_t outputs)
{
	/* configure the I/O pins*/
	uint8_t cmd[2] = { IOE_REG_OUTPUT, (uint8_t)(outputs&0xff) };

	/* send config word to the device  */
	int ret = transfer(cmd, 2, nullptr, 0);

	if (PX4_OK != ret) {
		PX4_DEBUG("write : i2c::transfer returned %d", ret);
	}

	return ret;
}//write


/*******************************************************************************
* Function Name  : ioctl
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int PCA9557::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	PCA9557Dev::lock();

	switch (cmd)
	{
		case IOE_SET_OUTPUTS:
			_outputs = ((uint32_t)arg);
			if ((ret = write(_outputs)) == PX4_OK) {

			}
			break;

		case IOE_GET_INPUTS:
			if ((ret = read(_inputs)) == PX4_OK) {
				*((uint32_t *)arg) = _inputs;
			}
			break;

		case IOE_SET_PIN:
			_outputs |= (1 << ((uint32_t)arg));
			if ((ret = write(_outputs)) == PX4_OK) {

			}
			break;

		case IOE_RST_PIN:
			_outputs &= ~(1 << ((uint32_t)arg));
			if ((ret = write(_outputs)) == PX4_OK) {

			}
			break;

		default:
			ret = -ENOTTY;
			break;
	}//switch

	PCA9557Dev::unlock();

	return ret;
}//ioctl


/*******************************************************************************
* Function Name  : print_status
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PCA9557::print_status()
{
	I2CSPIDriverBase::print_status();

	uint32_t inputs = 0;
	PCA9557Dev::lock();
	const int ret = read(inputs);
	PCA9557Dev::unlock();

	PX4_INFO("output register 0x%02x, restored %u times", static_cast<unsigned>(_outputs & 0xff),
		 static_cast<unsigned>(_restores));

	if (ret == PX4_OK) {
		PX4_INFO("pin levels      0x%02x", static_cast<unsigned>(inputs & 0xff));

	} else {
		PX4_ERR("input read failed (%d)", ret);
	}

	// Registers as the chip holds them
	static const char *const names[3] {"output", "polarity", "config"};

	for (uint8_t reg = IOE_REG_OUTPUT; reg <= IOE_REG_CONFIG; reg++) {
		uint8_t value = 0;
		PCA9557Dev::lock();
		const int r = read_reg(reg, value);
		PCA9557Dev::unlock();

		if (r == PX4_OK) {
			PX4_INFO("chip %-8s  0x%02x", names[reg - IOE_REG_OUTPUT], value);

		} else {
			PX4_ERR("chip %s read failed (%d)", names[reg - IOE_REG_OUTPUT], r);
		}
	}
}//print_status


/*******************************************************************************
* Function Name  : RunImpl
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PCA9557::RunImpl()
{
	PCA9557Dev::lock();

	uint8_t dir = 0;

	if (read_reg(IOE_REG_CONFIG, dir) == PX4_OK && dir != CONFIG_ALL_OUTPUTS) {
		_restores++;

		if (_restores == 1) {
			PX4_WARN("expander was reset (directions 0x%02x), restoring outputs", dir);
		}

		apply();
	}

	PCA9557Dev::unlock();
}//RunImpl


/*******************************************************************************
* Function Name  : print_usage
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PCA9557::print_usage()
{
	PRINT_MODULE_USAGE_NAME("pca9557", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(IOE_DEV_ADDR);
	PRINT_MODULE_USAGE_COMMAND("reset");
	PRINT_MODULE_USAGE_COMMAND_DESCR("restore", "Write the pin directions and outputs again");
	PRINT_MODULE_USAGE_COMMAND_DESCR("pin", "Set one output");
	PRINT_MODULE_USAGE_PARAM_INT('p', -1, 0, 7, "Output pin", false);
	PRINT_MODULE_USAGE_PARAM_INT('v', -1, 0, 1, "Output level", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}//print_usage


/*******************************************************************************
* Function Name  : custom_method
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PCA9557::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
		case 0: reset(); break;

		case 1:
		case 2:
			PCA9557Dev::lock();

			if (cli.custom1 == 1) {
				_outputs |= (1 << cli.custom2);

			} else {
				_outputs &= ~(1 << cli.custom2);
			}

			if (write(_outputs) != PX4_OK) {
				PX4_ERR("write failed");
			}

			PCA9557Dev::unlock();
			break;

		case 3:
			PCA9557Dev::lock();

			if (apply() != PX4_OK) {
				PX4_ERR("restore failed");
			}

			PCA9557Dev::unlock();
			break;
	}
}//custom_method


/*******************************************************************************
* Function Name  : pca9557_main
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
extern "C" int pca9557_main(int argc, char *argv[])
{
	using ThisDriver = PCA9557;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = IOE_DEV_ADDR;

	int ch;
	int pin = -1;
	int value = -1;

	while ((ch = cli.getOpt(argc, argv, "p:v:")) != EOF) {
		switch (ch) {
		case 'p':
			pin = atoi(cli.optArg());
			break;

		case 'v':
			value = atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_IOE_DEVTYPE_PCA9557);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "reset")) {
		cli.custom1 = 0;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "restore")) {
		cli.custom1 = 3;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "pin")) {
		if (pin < 0 || pin > 7 || (value != 0 && value != 1)) {
			PX4_ERR("usage: pca9557 -p <0-7> -v <0|1> pin");
			return -1;
		}

		cli.custom1 = value ? 1 : 2;
		cli.custom2 = pin;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}//pca9557_main
