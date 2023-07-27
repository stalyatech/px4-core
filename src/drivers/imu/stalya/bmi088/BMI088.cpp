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

#include "BMI088.hpp"

#include "BMI088_Accelerometer.hpp"
#include "BMI088_Gyroscope.hpp"
#include <px4_arch/hw_description.h>


/*******************************************************************************
* Function Name  : instantiate
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
I2CSPIDriverBase *BMI088::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				      int runtime_instance)
{
	Device *interface_accel = nullptr;
	Device *interface_gyro = nullptr;
	BMI088 *instance = nullptr;

	if (iterator.busType() == BOARD_I2C_BUS) {
		interface_accel = BMI088_I2C_interface(DRV_ACC_DEVTYPE_BMI088, nullptr, iterator.bus(), 0x18, cli.bus_frequency);
		interface_gyro  = BMI088_I2C_interface(DRV_GYR_DEVTYPE_BMI088, nullptr, iterator.bus(), 0x68, cli.bus_frequency);
	} else if (iterator.busType() == BOARD_SPI_BUS) {
		interface_accel = BMI088_SPI_interface(DRV_ACC_DEVTYPE_BMI088, nullptr, iterator.bus(), iterator.devid(), cli.spi_mode, cli.bus_frequency, true);
		interface_gyro  = BMI088_SPI_interface(DRV_GYR_DEVTYPE_BMI088, nullptr, iterator.bus(), iterator.devid(), cli.spi_mode, cli.bus_frequency, false);
	}

	if ((interface_accel == nullptr) || (interface_gyro == nullptr)) {
		PX4_ERR("alloc failed");
		delete interface_accel;
		delete interface_gyro;
		return nullptr;
	}

	instance = new BMI088(interface_accel, interface_gyro,
				iterator.configuredBusOption(),
				iterator.bus(),
				cli.rotation,
				0,
				iterator.DRDYGPIO());


	if (!instance) {
		PX4_ERR("alloc failed");
		delete interface_accel;
		delete interface_gyro;
		return nullptr;
	}

	if (!instance->Reset()) {
		delete interface_accel;
		delete interface_gyro;
		delete instance;
		return nullptr;
	}

	return instance;
}//instantiate


/*******************************************************************************
* Function Name  : Constructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
BMI088::BMI088(Device *interface_accel, Device *interface_gyro, I2CSPIBusOption bus_option, int bus, enum Rotation rotation,
		int i2c_address, spi_drdy_gpio_t drdy_gpio) :
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(interface_accel->get_device_id()), bus_option, bus),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_bad_transfer_perf(perf_alloc(PC_COUNT, MODULE_NAME": bad transfers")),
	_bad_register_perf(perf_alloc(PC_COUNT, MODULE_NAME": bad registers")),
	_good_transfer_perf(perf_alloc(PC_COUNT, MODULE_NAME": good transfers"))
{
	/* create the sensor instances */
	_accel = new Bosch::BMI088::Accelerometer::BMI088_Accelerometer(interface_accel, rotation);
	_gyro  = new Bosch::BMI088::Gyroscope::BMI088_Gyroscope(interface_gyro, rotation);

	/* data ready pin */
	_drdy_gpio = drdy_gpio;
}


/*******************************************************************************
* Function Name  : Destructor
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
BMI088::~BMI088()
{
	/* delete the sensor instances */
	delete _accel;
	delete _gyro;

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_bad_register_perf);
	perf_free(_good_transfer_perf);
}


/*******************************************************************************
* Function Name  : exit_and_cleanup
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BMI088::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}//exit_and_cleanup


/*******************************************************************************
* Function Name  : print_status
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BMI088::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_good_transfer_perf);
}//print_status


/*******************************************************************************
* Function Name  : Reset
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool BMI088::Reset()
{
	_state = STATE::INIT;
	ScheduleClear();
	ScheduleNow();
	return true;
}//Reset


/*******************************************************************************
* Function Name  : Init
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088::Init()
{
	bool acc_conf = false;
	bool gyr_conf = false;
	int ret = PX4_ERROR;

	/* probe the accelerometer */
	if (_accel->Probe() == PX4_OK)
	{
		/* reset the sensor*/
		if (_accel->Reset() == PX4_OK) {
			/* configure the sensor */
			acc_conf = _accel->Configure();
		}
	}//if

	/* probe the gyroscope */
	if (_gyro->Probe() == PX4_OK)
	{
		/* reset the sensor*/
		if (_gyro->Reset() == PX4_OK) {
			/* configure the sensor */
			gyr_conf = _gyro->Configure();
		}
	}//if

	/* API uploads the bmi08x config file onto the device and wait for 150ms
	* to enable the data synchronization.
	*/
	if (acc_conf && gyr_conf) {
		if (_accel->ConfigureSyncMode() == PX4_OK) {

			/* Enable the data ready event */
			DataReadyInterruptConfigure();

			ret = PX4_OK;
		}
	}

	return ret;
}//Init


/*******************************************************************************
* Function Name  : RunImpl
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BMI088::RunImpl()
{
	switch (_state)
	{
	case STATE::INIT:
	{
		/* initialize and configure the sensors */
		if (Init() == PX4_OK) {
			/* read state is interrupt driven */
			_state = STATE::READ;
		} else {
			/* try to initialize again */
			ScheduleDelayed(100_ms);
		}
		break;
	}

	case STATE::READ:
	{
		/* start measuring */
		perf_begin(_sample_perf);

		/* get the time stamp */
		const hrt_abstime now = hrt_absolute_time();

		/* Read the accelerometer data */
		if (_accel->ReadData((int16_t*)&_accel_data) != PX4_OK) {
			perf_count(_bad_transfer_perf);
			perf_end(_sample_perf);
			return;
		}

		/* Read the gyroscope data */
		if (_gyro->ReadData((int16_t*)&_gyro_data) != PX4_OK) {
			perf_count(_bad_transfer_perf);
			perf_end(_sample_perf);
			return;
		}

		/* check configuration registers periodically */
		if (hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {

			/* update the time stamp */
			_last_config_check_timestamp = now;

			if ((_accel->Check() != PX4_OK) || (_gyro->Check() != PX4_OK)) {
				perf_count(_bad_register_perf);
				perf_end(_sample_perf);

				/* register check failed, force reset */
				Reset();
				return;
			}
		}

		/* periodically update temperature (~1 Hz) */
		if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {

			float temperature;

			/* update the time stamp */
			_temperature_update_timestamp = now;

			/* Read the temperature */
			if (_accel->ReadTemperature(&temperature) != PX4_OK) {
				perf_count(_bad_transfer_perf);
				perf_end(_sample_perf);
				return;
			}

			/* report the temperature */
			_accel->SetTemperature(temperature);
			_gyro->SetTemperature(temperature);
		}

		/* increment the good transfer count */
		perf_count(_good_transfer_perf);

		// report the error count as the sum of the number of bad
		// transfers and bad register reads. This allows the higher
		// level code to decide if it should use this sensor based on
		// whether it has had failures
		const uint64_t error_count = perf_event_count(_bad_transfer_perf) + perf_event_count(_bad_register_perf);
		_accel->SetErrorCount(error_count);
		_gyro->SetErrorCount(error_count);

		/*
		* 1) Scale raw value to SI units using scaling from datasheet.
		* 2) Subtract static offset (in SI units)
		* 3) Scale the statically calibrated values with a linear
		*    dynamically obtained factor
		*
		* Note: the static sensor offset is the number the sensor outputs
		* 	 at a nominally 'zero' input. Therefore the offset has to
		* 	 be subtracted.
		*
		*	 Example: A gyro outputs a value of 74 at zero angular rate
		*	 	  the offset is 74 from the origin and subtracting
		*		  74 from all measurements centers them around zero.
		*/


		/* NOTE: Axes have been swapped to match the board a few lines above. */

		_accel->Update(now, _accel_data.x, _accel_data.y, _accel_data.z);
		_gyro->Update(now, _gyro_data.x, _gyro_data.y, _gyro_data.z);

		/* stop measuring */
		perf_end(_sample_perf);
		break;
	}//READ

	case STATE::FIFO_READ:
	{
		sensor_accel_fifo_s accel{};
		sensor_gyro_fifo_s gyro{};

		/* start measuring */
		perf_begin(_sample_perf);

		/* get the time stamp */
		const hrt_abstime now = hrt_absolute_time();

		/* Read the accelerometer FIFO */
		if (_accel->ReadFIFO(now, accel) != PX4_OK) {
			perf_count(_bad_transfer_perf);
			perf_end(_sample_perf);
			return;
		}

		/* Read the gyroscope FIFO */
		if (_gyro->ReadFIFO(now, gyro) != PX4_OK) {
			perf_count(_bad_transfer_perf);
			perf_end(_sample_perf);
			return;
		}

		/* check configuration registers periodically */
		if (hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {

			/* update the time stamp */
			_last_config_check_timestamp = now;

			if ((_accel->Check() != PX4_OK) || (_gyro->Check() != PX4_OK)) {
				perf_count(_bad_register_perf);
				perf_end(_sample_perf);

				/* register check failed, force reset */
				Reset();
				return;
			}
		}

		/* periodically update temperature (~1 Hz) */
		if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {

			float temperature;

			/* update the time stamp */
			_temperature_update_timestamp = now;

			/* Read the temperature */
			if (_accel->ReadTemperature(&temperature) != PX4_OK) {
				perf_count(_bad_transfer_perf);
				perf_end(_sample_perf);
				return;
			}

			/* report the temperature */
			_accel->SetTemperature(temperature);
			_gyro->SetTemperature(temperature);
		}

		/* increment the good transfer count */
		perf_count(_good_transfer_perf);

		// report the error count as the sum of the number of bad
		// transfers and bad register reads. This allows the higher
		// level code to decide if it should use this sensor based on
		// whether it has had failures
		const uint64_t error_count = perf_event_count(_bad_transfer_perf) + perf_event_count(_bad_register_perf);
		_accel->SetErrorCount(error_count);
		_gyro->SetErrorCount(error_count);

		/*
		* 1) Scale raw value to SI units using scaling from datasheet.
		* 2) Subtract static offset (in SI units)
		* 3) Scale the statically calibrated values with a linear
		*    dynamically obtained factor
		*
		* Note: the static sensor offset is the number the sensor outputs
		* 	 at a nominally 'zero' input. Therefore the offset has to
		* 	 be subtracted.
		*
		*	 Example: A gyro outputs a value of 74 at zero angular rate
		*	 	  the offset is 74 from the origin and subtracting
		*		  74 from all measurements centers them around zero.
		*/


		/* NOTE: Axes have been swapped to match the board a few lines above. */

		if (accel.samples > 0) {
			_accel->UpdateFIFO(accel);
		}

		if (gyro.samples > 0) {
			_gyro->UpdateFIFO(gyro);
		}

		/* stop measuring */
		perf_end(_sample_perf);
		break;
	}//FIFO_READ

	}//switch
}//RunImpl


/*******************************************************************************
* Function Name  : DataReady
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BMI088::DataReady()
{
	/* data is available */
	ScheduleNow();
}//DataReady


/*******************************************************************************
* Function Name  : DataReadyInterruptConfigure
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool BMI088::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on rising edge
	return px4_arch_gpiosetevent(_drdy_gpio, true, false, true, &DataReadyInterruptCallback, this) == 0;
}//DataReadyInterruptConfigure


/*******************************************************************************
* Function Name  : DataReadyInterruptDisable
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
bool BMI088::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}//DataReadyInterruptDisable


/*******************************************************************************
* Function Name  : DataReadyInterruptCallback
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int BMI088::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<BMI088*>(arg)->DataReady();
	return 0;
}//DataReadyInterruptCallback
