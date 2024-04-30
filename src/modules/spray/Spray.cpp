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

#include "Spray.hpp"

Spray::Spray() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_event_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": event"))
{
}

Spray::~Spray()
{
	perf_free(_cycle_perf);
	perf_free(_event_perf);
}

bool Spray::init()
{
	// execute Run() on every "spray_event" publication
	if (!_spray_event_sub.registerCallback()) {
		PX4_ERR("spray_event callback registration failed");
		return false;
	}

	// execute Run() on every "local_vehicle_position" publication
	if (!_vehicle_pos_sub.registerCallback()) {
		PX4_ERR("local_vehicle_position callback registration failed");
		return false;
	}

	update_params(true);

	reset();

	return true;
}

int Spray::values()
{
	PX4_INFO("Flowrate:%f",
		(double)_spray_status.flowrate);

	return PX4_OK;
}//values

int Spray::reset()
{
	_spray_status.flowrate  = 0;
	_spray_status.timestamp = hrt_absolute_time();

	return values();
}//reset

/*
 * vel : m/s
 * l   : m
 * t   : liter/dek
 */
void Spray::Calculate(float vel, float l, float t)
{
	float pump_speed = (vel * l * 60.0f) * 0.001f * t;

	_spray_status.flowrate  = pump_speed;
	_spray_status.timestamp = hrt_absolute_time();
	_spray_status_pub.publish(_spray_status);
}

void Spray::Run()
{
	if (should_exit()) {
		_spray_event_sub.unregisterCallback();
		_vehicle_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// update the parameters
	update_params();

	// start the performance counter
	perf_begin(_cycle_perf);

	struct spray_event_s spray_event{0};
	struct vehicle_local_position_s vehicle_pos{0};

	// get the current time stamp
	hrt_abstime now  = hrt_absolute_time();

	// listen the spray event topic
	if (_spray_event_sub.update(&spray_event)) {

		// performance counter for event
		perf_count(_event_perf);

		// is it clear request ?
		if (spray_event.event & spray_event_s::EVENT_CLEAR) {
			_spray_status.flowrate  = 0;
			_spray_status.timestamp = now;
			_spray_status_pub.publish(_spray_status);
		}
	}

	// listen local position event
	if (_vehicle_pos_sub.update(&vehicle_pos)) {

		Calculate(vehicle_pos.vx, _param_spray_length.get(), _param_spray_volume.get());
	}

	// end the performance counter
	perf_end(_cycle_perf);
}

int Spray::task_spawn(int argc, char *argv[])
{
	Spray *instance = new Spray();

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

void Spray::update_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		ModuleParams::updateParams();
	}
}//update_params

int Spray::custom_command(int argc, char *argv[])
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

int Spray::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_event_perf);

	return PX4_OK;
}

int Spray::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements estimated spray liquid level from the pump working time.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("spray", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("reset");
	PRINT_MODULE_USAGE_COMMAND("values");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}

extern "C" __EXPORT int spray_main(int argc, char *argv[])
{
	return Spray::main(argc, argv);
}
