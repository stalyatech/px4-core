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
	_tank_stat_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": tank stat")),
	_vehicle_pos_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": vehicle pos")),
	_spray_event_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": spray event"))
{
}

Spray::~Spray()
{
	perf_free(_cycle_perf);
	perf_free(_tank_stat_perf);
	perf_free(_vehicle_pos_perf);
	perf_free(_spray_event_perf);
}

bool Spray::init()
{
	// execute Run() on every "tank_status" publication
	if (!_tank_stat_sub.registerCallback()) {
		PX4_ERR("tank_status callback registration failed");
		return false;
	}

	// execute Run() on every "local_vehicle_position" publication
	if (!_vehicle_pos_sub.registerCallback()) {
		PX4_ERR("local_vehicle_position callback registration failed");
		return false;
	}

	// execute Run() on every "spray_event" publication
	if (!_spray_event_sub.registerCallback()) {
		PX4_ERR("spray_event callback registration failed");
		return false;
	}

	update_params(true);

	reset();

	return true;
}

int Spray::values()
{
	const char *Modes[] = {"MAN", "AUTO"};

	PX4_INFO("Mode:%s Speed:%f Height:%f Flowrate:%f",
		Modes[_spray_stat.spraymode],
		(double)_spray_stat.flyspeed,
		(double)_spray_stat.flyheight,
		(double)_spray_stat.flowrate);

	return PX4_OK;
}//values

int Spray::reset()
{
	_spray_stat.spraymode = MODE_MANUEL;
	_spray_stat.flyspeed  = 0;
	_spray_stat.flyheight = 0;
	_spray_stat.flowrate  = 0;
	_spray_stat.timestamp = hrt_absolute_time();

	return values();
}//reset

/*
 * flyspeed 	 : m/s
 * flyheight	 : m
 * track_width   : m
 * vol_per_acres : mililiter/acres
 */
void Spray::Calculate(float flyspeed, float flyheight, float track_width, float vol_per_acres, int mode)
{
	float flow_rate = 0;

	switch (mode)
	{
		case MODE_MANUEL: {
			int percent = _param_spray_speed_cur.get();
			if ((percent >= 0) && (percent <= 100)) {

				/* Pompa hızı (litre/dk) */
				flow_rate = (percent/100.0f) * _param_spray_speed_max.get();
			}
			break;
		}

		case MODE_AUTO: {

			/* check the minimum vehicle velocity */
			if (flyspeed >= _param_spray_velocity.get()) {

				/* Alınan yol X (m/dk) */
				float x = flyspeed * 60.0f;

				/* Alan A (dönüm/dk)*/
				float a = x * track_width * 0.001f;

				/* Pompa hızı (litre/dk) */
				flow_rate = vol_per_acres * a * 0.001f;
			}
			break;
		}
	}//switch

	/* prepare and publish the status message */
	_spray_stat.spraymode = mode;
	_spray_stat.flyspeed  = flyspeed;
	_spray_stat.flyheight = flyheight;
	_spray_stat.flowrate  = flow_rate;
	_spray_stat.timestamp = hrt_absolute_time();
	_spray_stat_pub.publish(_spray_stat);

	/* set actuator according to flow rate */
	publishVehicleCmdDoSetActuator(flow_rate);
}

void Spray::publishVehicleCmdDoSetActuator(float flowrate)
{
	vehicle_command_s command{};
	float act_out = -1;

	/* check for enable status */
	if (_param_spray_enab.get() == 0) {
		act_out = -1;
	} else {

		/* get the maximum pump speed */
		float max_pump_speed = _param_spray_speed_max.get();

		/* check for valid pump speed */
		if (max_pump_speed > 0.0f) {
			/* scale the flow rate to -1 .. +1 */
			act_out = ((flowrate/max_pump_speed) * 2) - 1.0f;
		} else {
			act_out = -1;
		}
	}

	command.timestamp = hrt_absolute_time();
	command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
	command.param1 = -1;	// Actuator 1
	command.param2 = -1; 	// Actuator 2
	command.param3 = -1; 	// Actuator 3
	command.param4 = -1; 	// Actuator 4
	command.param5 = -1; 	// Actuator 5
	command.param6 = -1; 	// Actuator 6
	command.param7 = 0; 	// Index

	/* update the actuator channel */
	switch (_param_spray_chan.get()) {
		case CHAN_ACT1:
			command.param1 = act_out;
			break;

		case CHAN_ACT2:
			command.param2 = act_out;
			break;

		case CHAN_ACT3:
			command.param3 = act_out;
			break;

		case CHAN_ACT4:
			command.param4 = act_out;
			break;

		case CHAN_ACT5:
			command.param5 = act_out;
			break;

		case CHAN_ACT6:
			command.param6 = act_out;
			break;
	}

	command.target_system = 1;
	command.target_component = 1;
	command.source_system = 1;
	command.source_component = 1;
	command.confirmation = false;
	command.from_external = false;

	// publish the vehicle command
	_vehicle_cmd_pub.publish(command);
}

void Spray::Run()
{
	if (should_exit()) {
		_tank_stat_sub.unregisterCallback();
		_vehicle_pos_sub.unregisterCallback();
		_spray_event_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// update the parameters
	update_params();

	// start the performance counter
	perf_begin(_cycle_perf);

	struct tank_status_s tank_stat{0};
	struct spray_event_s spray_event{0};
	struct vehicle_local_position_s vehicle_pos{0};

	// get the current time stamp
	hrt_abstime now  = hrt_absolute_time();

	// listen the tank status topic
	if (_tank_stat_sub.update(&tank_stat)) {

		// performance counter for tank status
		perf_count(_tank_stat_perf);
	}

	// listen the spray event topic
	if (_spray_event_sub.update(&spray_event)) {

		// performance counter for spray event
		perf_count(_spray_event_perf);

		// is it clear request ?
		if (spray_event.event & spray_event_s::EVENT_CLEAR) {
			reset();
			_spray_stat.timestamp = now;
			_spray_stat_pub.publish(_spray_stat);
		}
	}

	// listen local position event
	if (_vehicle_pos_sub.update(&vehicle_pos)) {

		// performance counter for vehicle position
		perf_count(_vehicle_pos_perf);

		/* vector speed of vehicle */
		float speed = sqrt(vehicle_pos.vx*vehicle_pos.vx + vehicle_pos.vy*vehicle_pos.vy);

		/* calculate the spraying speed */
		Calculate(speed, vehicle_pos.dist_bottom, _param_spray_width.get(), _param_spray_volume.get(), _param_spray_mode.get());
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
	perf_print_counter(_vehicle_pos_perf);
	perf_print_counter(_tank_stat_perf);
	perf_print_counter(_spray_event_perf);

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
