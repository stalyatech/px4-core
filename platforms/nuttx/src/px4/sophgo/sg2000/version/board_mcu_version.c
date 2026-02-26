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
#include <string.h>
#include <stdio.h>

#include <board_config.h>
#include <lib/systemlib/px4_macros.h>
#include <nuttx/arch.h>
#include <arch/csr.h>

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
	return "sg2000";
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
	return "sg2000";
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

/* SG2000 FSBL reads these eFuse words as SoC serial related data. */
#define SG2000_EFUSE_LEAKAGE 0x03050108u
#define SG2000_EFUSE_FTSN3   0x0305010cu
#define SG2000_EFUSE_FTSN4   0x03050110u

static const uint16_t soc_arch_id = PX4_SOC_ARCH_ID;

#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00ff0000) >> 8) | (((x) & 0x0000ff00) << 8) | ((x) << 24))

static void board_read_uuid_words(uuid_uint32_t uuid_words)
{
	uuid_words[0] = getreg32(SG2000_EFUSE_FTSN4);
	uuid_words[1] = getreg32(SG2000_EFUSE_FTSN3);
	uuid_words[2] = getreg32(SG2000_EFUSE_LEAKAGE);
	uuid_words[3] = READ_CSR(CSR_MIMPID);
}

__EXPORT void board_get_uuid32(uuid_uint32_t uuid_words)
{
	board_read_uuid_words(uuid_words);
}

int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	if (rev) {
		const uint32_t mimpid = READ_CSR(CSR_MIMPID);
		const uint32_t rev_id = mimpid & 0xf;
		*rev = (rev_id < 10) ? ('0' + rev_id) : ('A' + (rev_id - 10));
	}

	if (revstr) {
		*revstr = "SG2000";
	}

	if (errata) {
		*errata = NULL;
	}

	return (int)(READ_CSR(CSR_MIMPID) & 0xffff);
}

const char *board_bl_version_string(void)
{
	return "";
}

const char *board_fpga_version_string(void)
{
	return "";
}

int board_get_px4_guid(px4_guid_t px4_guid)
{
	uint8_t *pb = (uint8_t *)&px4_guid[0];
	*pb++ = (soc_arch_id >> 8) & 0xff;
	*pb++ = (soc_arch_id & 0xff);

	const int padding = PX4_GUID_BYTE_LENGTH - (int)(sizeof(soc_arch_id) + PX4_CPU_UUID_BYTE_LENGTH);

	for (int i = 0; i < padding; i++) {
		*pb++ = 0u;
	}

	uuid_uint32_t chip_uuid = {0};
	board_read_uuid_words(chip_uuid);

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		uint32_t uuid_bytes = SWAP_UINT32(chip_uuid[(PX4_CPU_UUID_WORD32_LENGTH - 1) - i]);
		memcpy(pb, &uuid_bytes, sizeof(uint32_t));
		pb += sizeof(uint32_t);
	}

	return PX4_GUID_BYTE_LENGTH;
}

int board_get_px4_guid_formated(char *format_buffer, int size)
{
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);
	int offset = 0;

	size = size & 1 ? size : size - 1;

	for (unsigned i = PX4_GUID_BYTE_LENGTH - size / 2; offset < size && i < PX4_GUID_BYTE_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", px4_guid[i]);
	}

	return offset;
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
	uint8_t *rv = &mfgid[0];
	uuid_uint32_t chip_uuid = {0};
	board_read_uuid_words(chip_uuid);

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		uint32_t uuid_bytes = SWAP_UINT32(chip_uuid[(PX4_CPU_UUID_WORD32_LENGTH - 1) - i]);
		memcpy(rv, &uuid_bytes, sizeof(uint32_t));
		rv += sizeof(uint32_t);
	}

	return PX4_CPU_MFGUID_BYTE_LENGTH;
}
