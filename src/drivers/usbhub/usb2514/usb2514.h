/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/drivers/smbus/SMBus.hpp>
#include <px4_platform_common/i2c_spi_buses.h>
#include <board_config.h>

using namespace time_literals;

// USB hub port number
enum class USBHUB_PORT_TYPE
{
    UPS_PORT,
    MCU_PORT,
    XBE_PORT,
    F9H_PORT,
    F9P_PORT,
};

class USB2514 : public I2CSPIDriver<USB2514>
{
public:
	USB2514(I2CSPIBusOption bus_option, const int bus, SMBus *interface);

	~USB2514();

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	friend SMBus;

	void RunImpl();

	bool Configure();

	void custom_method(const BusCLIArguments &cli) override;

private:
	SMBus   *_interface;
	bool 	 _attached{false};
    char 	 _utf16_str[64]{0};
	int 	 _ioe_fd {-1};
	uint8_t  _hub_config[256]
	{
		0x24,       // Vendor ID LSB
		0x04,       // Vendor ID MSB
		0x14,       // Product ID LSB
		0x25,       // Product ID MSB
		0x11,       // Device ID LSB
		0x22,       // Device ID MSB
		0xBB,		// Configuration Data Byte 1
		0x20,		// Configuration Data Byte 2
		0x09,		// Configuration Data Byte 3
		0x00,		// Non-Removable Devices
		0x00,		// Port Disable (Self)
		0x00,		// Port Disable (Bus)
		0x32,		// Max Power (Self)
		0xFA,		// Max Power (Bus)
		0x32,		// Hub Controller Max Current (Self)
		0xFA,		// Hub Controller Max Current (Bus)
		0x64,		// Power-on Time
		0x04,		// Language ID High (Turkish:0x041F, English:0x0409)
		0x09,		// Language ID Low
		0x08,		// Manufacturer String Length
		0x0F,		// Product String Length
		0x00,		// Serial String Length

		// Manufacturer String (max 31 character)
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00,

		// Product String (max 31 character)
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00,

		// Serial String (max 31 character)
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00,

		0x00,		// Battery charging enable

		// Reserved
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

		0x00,		// Boost_Up
		0x00,		// Reserved
		0x00,		// Boost_4:0
		0x00,		// Reserved
		0x0E,		// Port Swap
		0x00,		// Port Map 2-1
		0x00,		// Port Map 4-3
		0x00,		// Reserved
		0x00,		// Reserved
		0x00,		// Status/Command
	};

	void exit_and_cleanup() override;

	void Reset(bool stat);
	void PowerOn();
	void PowerOff();
	bool Attach();
	bool ConfigurePort(uint8_t port, bool stat);
	void ConfigureChip(uint32_t port_mask);
	void UIntToStr(char* buf, uint8_t digits, uint32_t val);
	int  CreateUTF16(const char *inp_str);
	void InsertManufacturer(const char *mfr_str);
	void InsertProduct(const char *prd_str);
	void InsertSerial(const char *ser_str);
	uint32_t Checksum(const void *buf_in, uint32_t buf_len, uint32_t init_crc);
};
