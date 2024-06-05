/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "tanklevelCheck.hpp"

void TankLevelChecks::checkAndReport(const Context &context, Report &reporter)
{
	spray_status_s spray_status;

	/* get the tank status values from system */
	if (!_spray_status_sub.copy(&spray_status)) {
		spray_status = {};
	}

	/* update the failsafe flags */
	reporter.failsafeFlags().spraying_done = (spray_status.status == spray_status_s::STATUS_DONE) ? (1) : (0);

	/* get the tank empty action */
	int action = _param_tank_empty_act.get();

	/* check the empty status */
	if (action != 0 && reporter.failsafeFlags().spraying_done) {

		if (action == spray_status_s::ACTION_RTL
			&& reporter.failsafeFlags().home_position_invalid) {

			/* EVENT
			* @description
			* <profile name="dev">
			* This check can be configured via <param>TANK_EMPTY_ACT</param> parameter.
			* </profile>
			*/
			reporter.armingCheckFailure(NavModes::All, health_component_t::system, events::ID("check_tank_level_no_home"),
							events::Log::Error, "Tank Level RTL requires valid home");

			if (reporter.mavlink_log_pub()) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: Tank Level RTL requires valid home");
			}
		}
	}
}
