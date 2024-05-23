/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "FakeFlow.hpp"

FakeFlow::FakeFlow() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_flow_update_freq = 15.0f; 	// 7.5 * Flow rate (L/min)
	_flow_interval_us = (uint32_t)(1000000 / _flow_update_freq);

	_freq.channel 	  = 0;
	_freq.timestamp   = 0;
	_freq.error_count = 0;
	_freq.period 	  = 0;
	_freq.pulse_width = 0;
	_freq.frequency   = 0;
}

bool FakeFlow::init()
{
	update_params(true);

	ScheduleOnInterval(10_ms);
	return true;
}

void FakeFlow::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	/* prepare the frequency input message */
	_freq.channel 	  = 1;
	_freq.timestamp   = hrt_absolute_time();
	_freq.error_count = 0;
	_freq.period 	  = 0;
	_freq.pulse_width = 0;
	_freq.frequency   = 0;

	/* vehicle command */
	vehicle_command_s vcmd;

	/* check the vehicle command update */
	if (_vehicle_cmd_sub.update(&vcmd)) {

		/* check for actuator set command */
		/* it comes from "spray" module */
		if (vcmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR) {

			float act_out = -1;

			/* get the actuator channel value */
			switch (_param_spray_chan.get()) {
				case CHAN_ACT1:
					act_out = vcmd.param1;
					break;

				case CHAN_ACT2:
					act_out = vcmd.param2;
					break;

				case CHAN_ACT3:
					act_out = vcmd.param3;
					break;

				case CHAN_ACT4:
					act_out = vcmd.param4;
					break;

				case CHAN_ACT5:
					act_out = vcmd.param5;
					break;

				case CHAN_ACT6:
					act_out = vcmd.param6;
					break;
			}

			/* according to parameter value update the variables */
			if (act_out > -1) {

				float flow_rate = 0;

				/* get the maximum pump speed */
				float max_pump_speed = _param_spray_speed_max.get();

				/* check for valid pump speed */
				if (max_pump_speed > 0.0f) {
					flow_rate = ((act_out + 1) / 2) * max_pump_speed;
				}

				_flow_update_freq = 17.5f * flow_rate; // L/min
				_flow_interval_us = (uint32_t)(1000000 / _flow_update_freq);

				_freq.period 	  = _flow_interval_us;
				_freq.pulse_width = _flow_interval_us/2;
				_freq.frequency   = _flow_update_freq;
			}
		}
	}

	_freq_input_pub.publish(_freq);
}

int FakeFlow::task_spawn(int argc, char *argv[])
{
	FakeFlow *instance = new FakeFlow();

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

void FakeFlow::update_params(const bool force)
{
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		// update parameters from storage
		ModuleParams::updateParams();
	}
}//update_params

int FakeFlow::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FakeFlow::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fake_flow", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int fake_flow_main(int argc, char *argv[])
{
	return FakeFlow::main(argc, argv);
}
