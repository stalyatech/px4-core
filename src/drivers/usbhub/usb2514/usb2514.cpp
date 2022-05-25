/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file usb2514.h
 *
 * Header for a USB hub connected via SMBus (I2C).
 * Designed for USB2514
 *
 * @author Tayfun Karan <tmkaran@hotmail.com.com>
 */

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <drivers/drv_ioe.h>

#include "usb2514.h"

/*
*********************************************************************************************************
*                                            LOCAL DEFINITIONS
*********************************************************************************************************
*/

#define DRV_HUB_DEVTYPE_SMBUS	(0xfe)

#define VBUS_READ_CNT   (10)

#define HUB_SMB_ADDR    (0x2C)
#define HUB_BLOCK_SIZE  (32)

#define HUB_MFR_LEN_OFS	(0x13)
#define HUB_PRD_LEN_OFS	(0x14)
#define HUB_SER_LEN_OFS	(0x15)

#define HUB_MFR_STR_OFS	(0x16)
#define HUB_PRD_STR_OFS	(0x54)
#define HUB_SER_STR_OFS	(0x92)

#define CRC32_POLY	(0x04C11DB7)

// System USB devices
#define USBHUB_CFG_MASK     ( (1<<static_cast<int>(USBHUB_PORT_TYPE::F9P_PORT))  |  \
                              (1<<static_cast<int>(USBHUB_PORT_TYPE::F9H_PORT))  |  \
                              (1<<static_cast<int>(USBHUB_PORT_TYPE::XBE_PORT)) )


/*
*********************************************************************************************************
*                                             LOCAL VARIABLES
*********************************************************************************************************
*/

const uint32_t divider[12] =
{   10,
    100,
	1000,		// K
	10000,
	100000,
	1000000,	// M
	10000000,
	100000000,
	1000000000 };	// B


extern "C" __EXPORT int usb2514_main(int argc, char *argv[]);


/*******************************************************************************
* Function Name  : Constructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
USB2514::USB2514(const I2CSPIDriverConfig &config, SMBus *interface) :
	I2CSPIDriver(config),
	_interface(interface)
{
#if defined(USBHUB_PWR_IOE_PIN) || defined(USBHUB_RST_IOE_PIN)
	_ioe_fd = px4_open(IOE_DEVICE_PATH, O_RDWR);

	if (_ioe_fd < 0) {
		PX4_ERR("Unable to open I/O expander device path");
		return;
	}
#endif /* USBHUB_PWR_IOE_PIN || USBHUB_RST_IOE_PIN */

#if defined(USBHUB_PWR_GPIO_PIN)
	// Configure power pin
	px4_arch_configgpio(USBHUB_PWR_GPIO_PIN);
#endif

#if defined(USBHUB_RST_GPIO_PIN)
	// Configure reset pin
	px4_arch_configgpio(USBHUB_RST_GPIO_PIN);
#endif

	_interface->init();
}

/*******************************************************************************
* Function Name  : Destructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
USB2514::~USB2514()
{
#if defined(USBHUB_PWR_IOE_PIN) || defined(USBHUB_RST_IOE_PIN)
	if (_ioe_fd != -1) {
		px4_close(_ioe_fd);
	}
#endif /* USBHUB_PWR_IOE_PIN || USBHUB_RST_IOE_PIN */

	if (_interface != nullptr) {
		delete _interface;
	}
}


/*******************************************************************************
* Function Name  : exit_and_cleanup
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::exit_and_cleanup()
{
	PowerOff();
	I2CSPIDriverBase::exit_and_cleanup();
}//exit_and_cleanup


/*******************************************************************************
* Function Name  : RunImpl
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::RunImpl()
{
}//RunImpl


/*******************************************************************************
* Function Name  : Configure
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool USB2514::Configure()
{
	uint8_t  block[HUB_BLOCK_SIZE]={0};
	uint32_t crc1 = 0xFFFFFFFF;
	uint32_t crc2 = 0xFFFFFFFF;
	uint8_t  i, addr, *cfg;
	char serial[32]={0};

	//
	// Power on the USB hub while Reset asserted
	//
	Reset(true);
	PowerOn();
	px4_usleep(10000);

	//
	// Release the reset of USB hub
	//
	Reset(false);
	px4_usleep(100000);

	//
	// Get the USB serial number as string
	//
	UIntToStr(serial, 10, 0xCAFEBABE);

	//
	// Update the HUB configuration
	//
	InsertManufacturer("Stalya");
	InsertProduct("TRAP");
	InsertSerial(serial);

	//
	// Update the HUB configuration
	//
	for (i=0; i<8; i++)
	{
		// update the register address
		addr = i * HUB_BLOCK_SIZE;

		// get the configuration part
		cfg = &_hub_config[addr];

		// write configuration data
		if (_interface->block_write(addr, cfg, HUB_BLOCK_SIZE, false) != 0) {
			// return with error
			return false;
		}//if

		// calculate the crc value of configuration part
		crc1 = Checksum(cfg, HUB_BLOCK_SIZE/4, crc1);
	}//for

	//
	// Read back the HUB configuration
	//
	for (i=0; i<8; i++)
	{
		// update the register address
		addr = i * HUB_BLOCK_SIZE;

		// read the configuration data
		if (_interface->block_read(addr, block, HUB_BLOCK_SIZE, false) != 0) {
			// return with error
			return false;
		}//if

		// calculate the crc value of configuration part
		crc2 = Checksum(block, HUB_BLOCK_SIZE/4, crc2);
	}//for

	//
	// Check for HUB configuration validity
	//
	if (crc1 == crc2)
	{
		//
		// Configures the hub ports
		//
		ConfigureChip(USBHUB_CFG_MASK);

		//
		// Attach the USB up-stream port
		//
		_attached = Attach();
		return _attached;
	}//if

	return false;
}//Configure


/*******************************************************************************
* Function Name  : print_usage
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::print_usage()
{
	PRINT_MODULE_USAGE_NAME("usb2514", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(HUB_SMB_ADDR);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}//print_usage


/*******************************************************************************
* Function Name  : instantiate
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
I2CSPIDriverBase *USB2514::instantiate(const I2CSPIDriverConfig &config, int runtime_instance)
{
	SMBus *interface = new SMBus(DRV_USB_DEVTYPE_HUB, config.bus, config.i2c_address);
	if (interface == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}
	USB2514 *instance = new USB2514(config, interface);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	instance->Configure();

	return instance;
}//instantiate


/*******************************************************************************
* Function Name  : custom_method
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::custom_method(const BusCLIArguments &cli)
{
}//custom_method


/*******************************************************************************
* Function Name  : Reset
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::Reset(bool stat)
{
#if defined(USBHUB_RST_GPIO_PIN)
	px4_arch_gpiowrite(USBHUB_RST_GPIO_PIN, !stat);
#elif defined(USBHUB_RST_IOE_PIN)
   	if (_ioe_fd >= 0) {
		px4_ioctl(_ioe_fd, IOE_RST_PIN, USBHUB_RST_IOE_PIN);
	}
#endif
}//Reset


/*******************************************************************************
* Function Name  : PowerOn
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::PowerOn()
{
#if defined(USBHUB_PWR_GPIO_PIN)
	px4_arch_gpiowrite(USBHUB_PWR_GPIO_PIN, true);
#elif defined(USBHUB_PWR_IOE_PIN)
   	if (_ioe_fd >= 0) {
		px4_ioctl(_ioe_fd, IOE_SET_PIN, USBHUB_PWR_IOE_PIN);
	}
#endif
}//PowerOn


/*******************************************************************************
* Function Name  : PowerOff
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::PowerOff()
{
#if defined(USBHUB_PWR_GPIO_PIN)
	px4_arch_gpiowrite(USBHUB_PWR_GPIO_PIN, false);
#elif defined(USBHUB_PWR_IOE_PIN)
   	if (_ioe_fd >= 0) {
		px4_ioctl(_ioe_fd, IOE_RST_PIN, USBHUB_PWR_IOE_PIN);
	}
#endif
}//PowerOff


/*******************************************************************************
* Function Name  : Attach
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool USB2514::Attach()
{
	// prepare the SMBus data
	uint8_t cmd = 0x01;

	// write the configuration data
	if (_interface->block_write(0xFF, &cmd, 1, false) != 0) {
		// return with error
		return false;
	}//if

	return true;
}//Attach


/*******************************************************************************
* Function Name  : ConfigurePort
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool USB2514::ConfigurePort(uint8_t port, bool stat)
{
	uint8_t port_map[2];

	// read the current port remap status
	if (_interface->block_read(0xFB, port_map, 2, false) != 0) {
		// return with error
		return false;
	}//if

	// scan the all logical port numbers
	uint8_t log_num = 0, num;
	for (uint8_t i=0; i<2; i++) {

		// find the last logical number
		if ((num = (port_map[i] & 0xff) & 7) > log_num) {
			log_num = num;
		}
		if ((num = ((port_map[i] >> 4) & 0xff) & 7) > log_num) {
			log_num = num;
		}
	}//for

	// update the next logical number
	log_num++;

	// check the physical port number
	switch (port)
	{
		case 1:
			port_map[0] &= 0xF0;
			if (stat) {
				port_map[0] |= log_num;
			}
			break;

		case 2:
			port_map[0] &= 0x0F;
			if (stat) {
				port_map[0] |= (log_num << 4);
			}
			break;

		case 3:
			port_map[1] &= 0xF0;
			if (stat) {
				port_map[1] |= log_num;
			}
			break;

		case 4:
			port_map[1] &= 0x0F;
			if (stat) {
				port_map[1] |= (log_num << 4);
			}
			break;
	}//switch

    	// write the new port remap status
  	if (_interface->block_write(0xFB, port_map, 2, false) != 0) {
		// return with error
		return false;
	}//if

    	return true;
}//ConfigurePort


/*******************************************************************************
* Function Name  : ConfigureChip
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::ConfigureChip(uint32_t port_mask)
{
	//
	// Enable the MCU USB port
	//
	ConfigurePort(static_cast<uint8_t>(USBHUB_PORT_TYPE::MCU_PORT), true);

	//
	// Enable the F9H USB port
	//
	ConfigurePort(static_cast<uint8_t>(USBHUB_PORT_TYPE::F9H_PORT), (port_mask & (1 << static_cast<int>(USBHUB_PORT_TYPE::F9H_PORT))) != 0);

	//
	// Enable the F9P USB port
	//
	ConfigurePort(static_cast<uint8_t>(USBHUB_PORT_TYPE::F9P_PORT), (port_mask & (1 << static_cast<int>(USBHUB_PORT_TYPE::F9P_PORT))) != 0);

	//
	// Enable the XBEE USB port
	//
	ConfigurePort(static_cast<uint8_t>(USBHUB_PORT_TYPE::XBE_PORT), (port_mask & (1 << static_cast<int>(USBHUB_PORT_TYPE::XBE_PORT))) != 0);
}//ConfigureChip


/*******************************************************************************
* Function Name  : UIntToStr
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::UIntToStr(char* buf, uint8_t digits, uint32_t val)
{
	uint8_t i = digits - 1;
	char d;

	do
	{
		i--;
		d = '0';
		while (val >= divider[i]) {
			d++;
			val -= divider[i];
		}
		*buf++ = d;
	} while(i != 0);

	*buf++ = val + '0';
}//UIntToStr


/*******************************************************************************
* Function Name  : CreateUTF16
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int USB2514::CreateUTF16(const char *inp_str)
{
    // get the string length
    int slen = math::min(strlen(inp_str), (size_t)31);

    // convert the string value to the UTF16_LE
    memset(_utf16_str, 0, sizeof(_utf16_str));
    for (uint8_t i=0; i<slen; i++)
    {
        _utf16_str[2*i+0] = inp_str[i];
        _utf16_str[2*i+1] = 0;
    }//for

    return (2*slen);
}//CreateUTF16


/*******************************************************************************
* Function Name  : InsertManufacturer
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::InsertManufacturer(const char *mfr_str)
{
    // create the utf16 data
    int len = CreateUTF16(mfr_str);

    // update the bub config
    _hub_config[HUB_MFR_LEN_OFS] = len/2;
    memcpy(&_hub_config[HUB_MFR_STR_OFS], _utf16_str, len);
}//InsertManufacturer


/*******************************************************************************
* Function Name  : InsertProduct
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::InsertProduct(const char *prd_str)
{
    // create the utf16 data
    int len = CreateUTF16(prd_str);

    // update the bub config
    _hub_config[HUB_PRD_LEN_OFS] = len/2;
    memcpy(&_hub_config[HUB_PRD_STR_OFS], _utf16_str, len);
}//InsertProduct


/*******************************************************************************
* Function Name  : InsertSerial
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB2514::InsertSerial(const char *ser_str)
{
    // create the utf16 data
    int len = CreateUTF16(ser_str);

    // update the bub config
    _hub_config[HUB_SER_LEN_OFS] = len/2;
    memcpy(&_hub_config[HUB_SER_STR_OFS], _utf16_str, len);
}//InsertSerial


/*******************************************************************************
* Function Name  : Checksum
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t USB2514::Checksum(const void *buf_in, uint32_t buf_len, uint32_t init_crc)
{
	uint32_t *buf = (uint32_t*)buf_in;
	uint32_t crc = init_crc;
	uint32_t i, j;

	for (i = 0; i < buf_len; i++)
	{
		crc ^= buf[i];
		for (j = 0; j < 32; j++)
		{
			if (crc & 0x80000000) {
				crc = (crc << 1) ^ CRC32_POLY;
			} else {
				crc <<= 1;
			}
		}//for
	}//for

	return crc;
}//Checksum


/*******************************************************************************
* Function Name  : usb2514_main
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
extern "C" __EXPORT int usb2514_main(int argc, char *argv[])
{
	using ThisDriver = USB2514;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 100000;
	cli.i2c_address = HUB_SMB_ADDR;

	const char *verb = cli.parseDefaultArguments(argc, argv);
	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_HUB_DEVTYPE_SMBUS);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
