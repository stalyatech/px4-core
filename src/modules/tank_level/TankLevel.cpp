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

#include "TankLevel.hpp"

TankLevel::TankLevel() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_event_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": event")),
	_freq_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": freq"))
{
}

TankLevel::~TankLevel()
{
	perf_free(_cycle_perf);
	perf_free(_event_perf);
	perf_free(_freq_perf);
}

bool TankLevel::init()
{
	// execute Run() on every "tank_event" publication
	if (!_tank_event_sub.registerCallback()) {
		PX4_ERR("tank_event callback registration failed");
		return false;
	}

	// execute Run() on every "freq_status" publication
	if (!_freq_status_sub.registerCallback()) {
		PX4_ERR("freq_status callback registration failed");
		return false;
	}

	update_params(true);

	reset();

	return true;
}

int TankLevel::values()
{
	PX4_INFO("Maximum: %f, Remain:%f, Consumed:%f, Flowrate:%f",
		(double)_tank_status.maxlevel,
		(double)_tank_status.remlevel,
		(double)_tank_status.consumed,
		(double)_tank_status.flowrate);

	return PX4_OK;
}//values

int TankLevel::reset()
{
	_tank_status.maxlevel  = _param_tank_vol_max.get();
	_tank_status.consumed  = 0;
	_tank_status.flowrate  = 0;
	_tank_status.remlevel  = _tank_status.maxlevel;
	_tank_status.timestamp = hrt_absolute_time();

	return values();
}//reset

void TankLevel::Run()
{
	if (should_exit()) {
		_tank_event_sub.unregisterCallback();
		_freq_status_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// update the parameters
	update_params();

	// start the performance counter
	perf_begin(_cycle_perf);

	struct tank_event_s  tank_event{0};
	struct freq_status_s freq_stat{0};

	// get the current time stamp
	hrt_abstime now  = hrt_absolute_time();

	// listen for frequency input event (flowmeter)
	if (_freq_status_sub.update(&freq_stat)) {

		// check the flowmeter channel
		if (freq_stat.channel == static_cast<uint32_t>(_param_tank_flow_inp.get())) {

			// performance counter for frequency meter
			perf_count(_freq_perf);

			// update the tank status using flow rate
			if (_tank_status.remlevel > 0.0f) {

				// get the difference time (minute)
				float diff = (now - _tank_status.timestamp) / (60 * 1000000.0f);

				if (diff > 0) {
					// calculate the flow rate (Liter/minute)
					float flowrate = freq_stat.frequency * _param_tank_flow_conv.get() * diff;

					_tank_status.empty_action = _param_tank_empty_act.get();
					_tank_status.maxlevel  = _param_tank_vol_max.get();
					_tank_status.consumed += flowrate;
					_tank_status.flowrate  = flowrate;
					_tank_status.remlevel  = _tank_status.maxlevel - _tank_status.consumed;
					if (_tank_status.remlevel < 0.0f) {
						_tank_status.remlevel = 0;
					}
					_tank_status.timestamp = now;
					_tank_status_pub.publish(_tank_status);
				}
			}
		}
	}

	// listen the tank event topic
	if (_tank_event_sub.update(&tank_event)) {

		// performance counter for event
		perf_count(_event_perf);

		// is it clear request ?
		if (tank_event.event & tank_event_s::EVENT_RESET) {
			_tank_status.maxlevel  = _param_tank_vol_max.get();
			_tank_status.consumed  = 0;
			_tank_status.flowrate  = 0;
			_tank_status.remlevel  = _tank_status.maxlevel - _tank_status.consumed;
			_tank_status.timestamp = now;
			_tank_status_pub.publish(_tank_status);
		}
	}

	// end the performance counter
	perf_end(_cycle_perf);
}

int TankLevel::task_spawn(int argc, char *argv[])
{
	TankLevel *instance = new TankLevel();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void TankLevel::update_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		ModuleParams::updateParams();
	}
}//update_params

int TankLevel::custom_command(int argc, char *argv[])
{
	const char *verb = argv[0];

	if (!strcmp(verb, "reset")) {
		if (is_running()) {
			return _object.load()->reset();
		}

		return PX4_ERROR;
	}

	if (!strcmp(verb, "values")) {
		if (is_running()) {
			return _object.load()->values();
		}

		return PX4_ERROR;
	}

	return print_usage("unknown command");
}//custom_command

int TankLevel::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_freq_perf);
	perf_print_counter(_event_perf);

	return PX4_OK;
}

int TankLevel::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements estimated tank liquid level from the pump working time.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tank_level", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("reset");
	PRINT_MODULE_USAGE_COMMAND("values");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}

extern "C" __EXPORT int tank_level_main(int argc, char *argv[])
{
	return TankLevel::main(argc, argv);
}
