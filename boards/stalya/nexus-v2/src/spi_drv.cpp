/****************************************************************************
 *
 *   Copyright (C) 2025 Stalya Inc. All rights reserved.
 *
 *   Author: Volvox <volvox@stalya.com>
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

#include <px4_arch/spi_hw_description.h>
#include <px4_platform_common/spi.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

static const px4_spi_bus_t *_spi_bus2;

/****************************************************************************
 * Name: board_spix_select
 *
 * Select or deselect a device on the SPI bus.
 *
 * This function iterates over the list of devices on the SPI bus and
 * writes the chip select GPIO for the device specified by devid.
 * If selected is true, the chip select line is asserted (i.e. set to
 * low), otherwise it is deasserted (i.e. set to high).
 *
 * @param bus The SPI bus to select the device on.
 * @param dev The SPI device structure to select.
 * @param devid The device ID of the device to select.
 * @param selected True to select the device, false to deselect.
 *
 ****************************************************************************/

 static inline void board_spix_select(const px4_spi_bus_t *bus, struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i)
		{
			if (bus->devices[i].cs_gpio == 0)
				{
					break;
				}

			if (devid == bus->devices[i].devid)
				{
					/* SPI select is active low, so write !selected to select the device */
				}
		}
}

/****************************************************************************
 * Name: sapphire_spi2_select
 *
 * Select or deselect a SPI device on bus 2.
 *
 * @param dev  The SPI device to select/deselect.
 * @param devid  The device ID of the SPI device to select/deselect.
 * @param selected  true to select the device, false to deselect it.
 *
 ****************************************************************************/

 #ifdef CONFIG_SAPPHIRE_SPI2
__EXPORT void sapphire_spi2_select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	board_spix_select(_spi_bus2, dev, devid, selected);
}
#endif

/****************************************************************************
 * Name: board_spidev_init
 *
 * Initialize all SPI devices on all buses.
 *
 * This function initializes all SPI devices on all buses by configuring
 * the chip select GPIOs for each device. This function should be called
 * once at startup to initialize all SPI devices.
 *
 ****************************************************************************/

 __EXPORT void board_spidev_init(void)
{
	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus)
		{
			for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i)
				{
					if (px4_spi_buses[bus].devices[i].cs_gpio != 0)
						{
							px4_arch_configgpio(px4_spi_buses[bus].devices[i].cs_gpio);
						}
				}
		}
}

/****************************************************************************
 * Name: board_spibus_init
 *
 * Initialize the SPI buses on this board.
 *
 * This function initializes the SPI buses on this board by configuring
 * the chip select GPIOs for each device. This function should be called
 * once at startup to initialize all SPI devices.
 *
 * @return OK on success, -ENODEV on failure.
 *
 ****************************************************************************/

 __EXPORT int board_spibus_init(void)
{
	for (int i = 0; i < SPI_BUS_MAX_BUS_ITEMS; ++i)
		{
			switch (px4_spi_buses[i].bus)
			{
				case 2: _spi_bus2 = &px4_spi_buses[i]; break;
			}
		}

	struct spi_dev_s *spi_bus2 = px4_spibus_initialize(2);

	if (!spi_bus2)
		{
			return -ENODEV;
		}

	/* deselect all */

	for (int bus = 0; bus < SPI_BUS_MAX_BUS_ITEMS; ++bus)
		{
			for (int i = 0; i < SPI_BUS_MAX_DEVICES; ++i)
				{
					if (px4_spi_buses[bus].devices[i].cs_gpio != 0)
						{
							SPI_SELECT(spi_bus2, px4_spi_buses[bus].devices[i].devid, false);
						}
				}
		}

	return OK;
}
