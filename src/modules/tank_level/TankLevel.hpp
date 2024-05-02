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
 * Tank level calculator according to manuel passthrough control group
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
#include <uORB/topics/tank_event.h>
#include <uORB/topics/freq_input.h>

using namespace time_literals;

class TankLevel : public ModuleBase<TankLevel>, public ModuleParams, public px4::WorkItem
{
public:
	TankLevel();
	~TankLevel() override;

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

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _tank_event_sub{this, ORB_ID(tank_event)};
	uORB::SubscriptionCallbackWorkItem _freq_input_sub{this, ORB_ID(freq_input)};

	uORB::Publication<tank_status_s> _tank_status_pub{ORB_ID(tank_status)};

	struct tank_status_s _tank_status{0};

	float _flow_rate{0};

	perf_counter_t _cycle_perf{nullptr};
	perf_counter_t _event_perf{nullptr};
	perf_counter_t _flowmeter_perf{nullptr};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::TANK_VOL_MAX>)   _param_tank_vol_max,
		(ParamInt  <px4::params::TANK_FLOW_INP>)  _param_tank_flow_inp,
		(ParamFloat<px4::params::TANK_FLOW_CONV>) _param_tank_flow_conv
	)
};
