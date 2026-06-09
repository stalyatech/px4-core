/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file fw_manifest.c
 *
 * Self-attestation manifest for the TPU-v2 PX4 image.
 *
 * The manifest lives in a dedicated section (`.note.fwmanifest`) so the
 * post-link stamping tool can locate and rewrite it after CRC32 over the
 * final image is computed. At runtime the firmware seeds its own CRC and
 * size registers (PX4IO_P_FW_MGMT_ACTIVE_*) from this struct, giving the
 * FMU and MC a consistent view of the running image.
 *
 * Until the stamper runs the placeholder values are zero (or
 * `FWMGMT_MANIFEST_MAGIC` for the magic word so the stamper can locate the
 * section). Booting an unstamped image is harmless — the FMU will report a
 * CRC mismatch on first verify, which is the desired safety behaviour.
 */

#include <stdint.h>
#include <fwmgmt_proto.h>

/*
 * Place the struct in a dedicated section so the stamping tool can find it
 * by section name without parsing symbol tables. The section name uses the
 * standard `.note.*` prefix so that LDs preserve it through default scripts;
 * the linker script for the TPU board adds an explicit KEEP() to be safe.
 */
__attribute__((used, aligned(64), section(".note.fwmanifest")))
const struct fwmgmt_manifest_v1 g_self_manifest = {
	.magic            = FWMGMT_MANIFEST_MAGIC,
	.fmt_version      = FWMGMT_MANIFEST_FMT_VERSION,
	.flags            = 0,
	.crc32_image      = 0,
	.image_size       = 0,
	.git_hash         = {0},
	.semver           = 0,
	.build_timestamp  = 0,
	.board_id         = FWMGMT_MANIFEST_BOARD_TPU_V2,
	.fmu_min_compat   = 0,
	.fmu_max_compat   = 0,
	.reserved         = {0},
	.crc32_self       = 0u,
};
