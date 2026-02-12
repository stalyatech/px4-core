/****************************************************************************
 *
 *   Copyright (C) 2025 Stalya Inc. All rights reserved.
 *
 ****************************************************************************/
#pragma once

#include <px4_arch/hw_description.h>
#include <px4_platform_common/spi.h>

#include <sg2000_gpio.h>

#if defined(CONFIG_SPI)

static inline constexpr px4_spi_bus_device_t initSPIDevice(uint32_t devid, SPI::CS cs_gpio, SPI::DRDY drdy_gpio = {})
{
	px4_spi_bus_device_t ret{};
	(void)devid;
	(void)cs_gpio;
	(void)drdy_gpio;
	return ret;
}

static inline constexpr px4_spi_bus_t initSPIBusInternal(SPI::Bus bus, const px4_spi_bus_devices_t &devices,
		GPIO::GPIOPin power_enable = {})
{
	px4_spi_bus_t ret{};
	(void)bus;
	(void)devices;
	(void)power_enable;
	return ret;
}

struct bus_device_external_cfg_array_t {
	SPI::bus_device_external_cfg_t devices[SPI_BUS_MAX_DEVICES];
};

static inline constexpr px4_spi_bus_t initSPIBusExternal(SPI::Bus bus, const bus_device_external_cfg_array_t &devices)
{
	px4_spi_bus_t ret{};
	(void)bus;
	(void)devices;
	return ret;
}

static inline constexpr SPI::bus_device_external_cfg_t initSPIConfigExternal(SPI::CS cs_gpio, SPI::DRDY drdy_gpio = {})
{
	SPI::bus_device_external_cfg_t ret{};
	(void)cs_gpio;
	(void)drdy_gpio;
	return ret;
}

struct px4_spi_bus_array_t {
	px4_spi_bus_t item[SPI_BUS_MAX_BUS_ITEMS];
};

static inline constexpr px4_spi_bus_all_hw_t initSPIHWVersion(int hw_version_revision,
		const px4_spi_bus_array_t &bus_items)
{
	px4_spi_bus_all_hw_t ret{};
	(void)hw_version_revision;
	(void)bus_items;
	return ret;
}

#endif /* CONFIG_SPI */
