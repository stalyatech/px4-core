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
	PCA9557(I2CSPIBusOption bus_option, int bus, int bus_frequency);
	~PCA9557() override = default;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
										 int runtime_instance);
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
PCA9557::PCA9557(I2CSPIBusOption bus_option, int bus, int bus_frequency) :
	I2C(DRV_IOE_DEVTYPE_PCA9557, MODULE_NAME, bus, IOE_DEV_ADDR, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),
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

	ret = config();
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
	ret = write(_outputs);
	if (ret != OK) {
		return ret;
	}

	return ret;
}//init


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
	uint8_t cmd[2] = { IOE_REG_CONFIG, 0x00 };

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
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}//print_usage


/*******************************************************************************
* Function Name  : instantiate
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
I2CSPIDriverBase *PCA9557::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
									   int runtime_instance)
{
	PCA9557 *instance = new PCA9557(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency);

	if (!instance) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (OK != instance->init()) {
		delete instance;
		return nullptr;
	}

	return instance;
}//instantiate


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

	const char *verb = cli.parseDefaultArguments(argc, argv);
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

	ThisDriver::print_usage();
	return -1;
}//pca9557_main
