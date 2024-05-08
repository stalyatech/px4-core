/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 *
 * Spray Module
 *
 * @author volvox <volvox@stalya.com>
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tank_status.h>
#include <uORB/topics/spray_status.h>
#include <uORB/topics/spray_event.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class Spray : public ModuleBase<Spray>, public ModuleParams, public px4::WorkItem
{
public:
	Spray();
	~Spray() override;

	/**
	 * @see ModuleBase::task_spawn().
	 * @brief Initializes the class in the same context as the work queue
	 *        and starts the background listener.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @see ModuleBase::custom_command().
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The input argument count.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage().
	 * @brief Prints the module usage to the nuttshell console.
	 * @param reason The requested reason for printing to console.
	 */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/**
	 * Initializes scheduling on work queue.
	 */
	bool init();

	/**
	 * Calculate dynamic spraying speed
	 * flyspeed		 : Uçuş hızı (m/s)
	 * flyheight	 : Uçuş yüksekliği (m)
	 * track_width   : İz genişliği (m)
	 * vol_per_acres : İlaçlama miktarı (Mililitre/Dekar)
	 * tank_level 	 : Kalan tank seviyesi miktarı (Litre)
	 * mode          : Sprayleme modu (Manuel/Otomatik)
	 */
	void Calculate(float flyspeed, float flyheight, float track_width, float vol_per_acres, float tank_level, int mode);

private:

	void Run() override;

	/**
	 * @brief Updates and checks for updated uORB parameters.
	 * @param force Boolean to determine if an update check should be forced.
	 */
	void update_params(const bool force = false);

	/**
	 * Print the values.
	 */
	int values();

	/**
	 * Reset the values.
	 */
	int reset();

	/**
	 * Publishes vehicle command.
	 */
	void publishVehicleCmdDoSetActuator(float flowrate);

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _tank_stat_sub{this, ORB_ID(tank_status)};
	uORB::SubscriptionCallbackWorkItem _spray_event_sub{this, ORB_ID(spray_event)};
	uORB::SubscriptionCallbackWorkItem _vehicle_pos_sub{this, ORB_ID(vehicle_local_position)};

	uORB::Publication<spray_status_s> _spray_stat_pub{ORB_ID(spray_status)};
	uORB::Publication<vehicle_command_s> _vehicle_cmd_pub{ORB_ID(vehicle_command)};

	spray_status_s _spray_stat{0};

	perf_counter_t _cycle_perf{nullptr};
	perf_counter_t _tank_stat_perf{nullptr};
	perf_counter_t _vehicle_pos_perf{nullptr};
	perf_counter_t _spray_event_perf{nullptr};

	DEFINE_PARAMETERS(
		(ParamInt  <px4::params::SPRAY_MODE>)    	_param_spray_mode,
		(ParamInt  <px4::params::SPRAY_ENABLE>)    	_param_spray_enab,
		(ParamInt  <px4::params::SPRAY_SWITCHOFF>)  _param_spray_switchoff,
		(ParamInt  <px4::params::SPRAY_CHANNEL>)    _param_spray_chan,
		(ParamFloat<px4::params::SPRAY_VOLUME>)  	_param_spray_volume,
		(ParamFloat<px4::params::SPRAY_WIDTH>)  	_param_spray_width,
		(ParamFloat<px4::params::SPRAY_VELOCITY>)  	_param_spray_velocity,
		(ParamInt  <px4::params::SPRAY_SPEED_CUR>)	_param_spray_speed_cur,
		(ParamFloat<px4::params::SPRAY_SPEED_MAX>)	_param_spray_speed_max
	)

	/* Spraying Modes */
	enum
	{
		MODE_MANUEL,
		MODE_AUTO
	};

	/* Spraying Switch Off Modes */
	enum
	{
		SWITCH_OFF_MANUEL,
		SWITCH_OFF_AUTO
	};

	/* Spraying actuator channels */
	enum
	{
		CHAN_NONE,
		CHAN_ACT1,
		CHAN_ACT2,
		CHAN_ACT3,
		CHAN_ACT4,
		CHAN_ACT5,
		CHAN_ACT6,
	};
};
