/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file spray_params.c
 *
 * Parameters defined by the spray module
 *
 * @author volvox <volvox@stalya.com>
 */

/**
 * Spray mode
 *
 *
 * @value 0 Manuel
 * @value 1 Auto
 *
 * @group Spraying
 */
PARAM_DEFINE_INT32(SPRAY_MODE, 0);

/**
 * Spray enable/disable
 *
 *
 * @value 0 Disabled
 * @value 1 Enabled
 *
 * @group Spraying
 */
PARAM_DEFINE_INT32(SPRAY_ENABLE, 0);

/**
 * Spray switch off mode
 *
 *
 * @value 0 Manuel
 * @value 1 Auto
 *
 * @group Spraying
 */
PARAM_DEFINE_INT32(SPRAY_SWITCHOFF, 1);

/**
 * Spray pump output channel number
 *
 *
 * @value 0 Disabled
 * @value 1 Actuator 1
 * @value 2 Actuator 2
 * @value 3 Actuator 3
 * @value 4 Actuator 4
 * @value 5 Actuator 5
 * @value 6 Actuator 6
 *
 * @group Spraying
 */
PARAM_DEFINE_INT32(SPRAY_CHANNEL, 0);

/**
 * Spray volume (Mililiter/Acres)
 *
 *
 * @group Spraying
 */
PARAM_DEFINE_FLOAT(SPRAY_VOLUME, 1000.0f);

/**
 * Spray Track Width (m)
 *
 *
 * @group Spraying
 */
PARAM_DEFINE_FLOAT(SPRAY_WIDTH, 4.0f);

/**
 * Spraying minumum speed (m/s)
 *
 *
 * @group Spraying
 */
PARAM_DEFINE_FLOAT(SPRAY_VELOCITY, 2.0f);

/**
 * Maximum spraying speed (Liter/Minute)
 *
 *
 * @group Spraying
 */
PARAM_DEFINE_FLOAT(SPRAY_SPEED_MAX, 5.0f);

/**
 * Current spraying speed in manuel mode (Liter/Minute)
 *
 * @min 0
 * @max 100
 * @increment 1
 *
 * @group Spraying
 */
PARAM_DEFINE_INT32(SPRAY_SPEED_CUR, 0);
