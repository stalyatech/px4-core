/****************************************************************************
 *
 *   Copyright (C) 2025 Stalya Inc. All rights reserved.
 *
 *   Author: Volvox <volvox@stalya.com>
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
 * @file usr_mcu_version.c
 * Implementation of MPFS version API
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/board_determine_hw_info.h>
#include <px4_arch/device_info.h>
#include <stdint.h>

#include <board_config.h>
#include <lib/systemlib/px4_macros.h>

#include <uORB/uORB.h>
#include <uORB/topics/guid.h>
#include <uORB/topics/hw_info.h>
#include <uORB/topics/system_version.h>
#include <uORB/topics/system_version_string.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: board_get_hw_type_name
 *
 * Description:
 *   Optional returns a 0 terminated string defining the HW type.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   a 0 terminated string defining the HW type. This my be a 0 length string ""
 *
 ************************************************************************************/

__EXPORT const char *board_get_hw_type_name()
{
	return (const char *) 0;
}

#if defined(BOARD_HAS_HW_SPLIT_VERSIONING)
/************************************************************************************
 * Name: board_get_hw_base_type_name
 *
 * Description:
 *   Optional returns a 0 terminated string defining the base type.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   a 0 terminated string defining the HW type. This my be a 0 length string ""
 *
 ************************************************************************************/

__EXPORT const char *board_get_hw_base_type_name()
{
	return (const char *) 0;
}
#endif

/************************************************************************************
 * Name: board_get_hw_version
 *
 * Description:
 *   Optional returns a integer HW version
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware version.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having version.
 *
 ************************************************************************************/

__EXPORT int board_get_hw_version()
{
	return 0;
}

/************************************************************************************
 * Name: board_get_hw_revision
 *
 * Description:
 *   Optional returns a integer HW revision
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer value of this boards hardware revision.
 *   A value of -1 is the default for boards not supporting the BOARD_HAS_VERSIONING API.
 *   A value of 0 is the default for boards supporting the API but not having revision.
 *
 ************************************************************************************/

__EXPORT int board_get_hw_revision()
{
	return 0;
}

__EXPORT void board_get_uuid32(uuid_uint32_t uuid_words)
{
}

int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	return 0;
}

const char *board_bl_version_string(void)
{
	return 0;
}

const char *board_fpga_version_string(void)
{
	return 0;
}

int board_get_px4_guid(px4_guid_t px4_guid)
{
	return 0;
}

int board_get_px4_guid_formated(char *format_buffer, int size)
{
	return 0;
}

/************************************************************************************
  * Name: board_determine_hw_info
 *
 * Description:
 *	Uses the HW revision and version detection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0  - on success or negated errono
 *   1) The values for integer value of this boards hardware revision is set
 *   2) The integer value of this boards hardware version is set.
 *   3) hw_info is populated
 *
 ************************************************************************************/

int board_determine_hw_info(void)
{
	return OK;
}

int board_get_mfguid(mfguid_t mfgid)
{
	return 0;
}
