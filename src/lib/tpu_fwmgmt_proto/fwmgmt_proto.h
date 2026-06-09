/****************************************************************************
 *
 *   Copyright (c) 2026 Stalya. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file fwmgmt_proto.h
 *
 * Shared protocol definitions for the TPU-v2 firmware management bridge.
 *
 * This header is the single source of truth for the cmdqu mailbox commands
 * and on-disk manifest format used by:
 *   - PX4 px4iofirmware running on TPU NuttX (sender of cmdqu requests).
 *   - The Linux mission-computer `tpufw` daemon (receiver / responder).
 *   - The build-time post-link manifest stamping tool.
 *
 * Three sides MUST consume the exact same definitions or the binary protocol
 * silently breaks. The Linux package mirrors this file (symlink or copy).
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * cmdqu cmd_id values used under IP_SYSTEM (cmdqu_t.ip_id == IP_SYSTEM == 6).
 *
 * Values 0x50..0x5F are reserved by the SOPHGO vendor SDK
 * (SYS_CMD_INFO_TRANS .. SYS_CMD_INFO_TRACE_STREAM_STOP).
 *
 * 0x60..0x67 are allocated for our firmware management bridge. cmd_id is
 * encoded as a 7-bit bitfield (cmd_id : 7) so any value <= 0x7F is legal.
 */
#define SYS_CMD_FWMGMT_GET_VERSION    0x60u  /* TPU/MC -> peer: identify running firmware */
#define SYS_CMD_FWMGMT_GET_CRC        0x61u  /* TPU -> MC: ask for active image CRC over /lib/firmware/firmware.elf */
#define SYS_CMD_FWMGMT_BEGIN_UPDATE   0x62u  /* TPU -> MC: announce upcoming staged image */
#define SYS_CMD_FWMGMT_CHUNK          0x63u  /* TPU -> MC: chunk has been written into the carveout staging buffer */
#define SYS_CMD_FWMGMT_END_UPDATE     0x64u  /* TPU -> MC: finalize staged image (CRC verified, request install) */
#define SYS_CMD_FWMGMT_RESTART        0x65u  /* TPU -> MC: request remoteproc stop/start */
#define SYS_CMD_FWMGMT_STATUS         0x66u  /* MC -> TPU/peer: progress / state report */
#define SYS_CMD_FWMGMT_ERROR          0x67u  /* MC -> TPU/peer: asynchronous error notification */

/*
 * On-disk manifest format. Lives next to firmware.elf as
 * /lib/firmware/firmware.manifest. The same byte layout is also embedded into
 * the ELF as a `.note.fwmanifest` section for redundancy.
 *
 * Layout is fixed at 128 bytes; all integer fields are little-endian on disk
 * (matches the SG2000 RV64 native ordering).
 */
#define FWMGMT_MANIFEST_MAGIC          0x464D5746u  /* 'FWMF' */
#define FWMGMT_MANIFEST_FMT_VERSION    1u
#define FWMGMT_MANIFEST_BYTES          128u
#define FWMGMT_MANIFEST_BOARD_TPU_V2   0x32555054u  /* 'TPU2' */

/*
 * Manifest flags. A clean release build sets none.
 */
#define FWMGMT_MANIFEST_FLAG_DIRTY     (1u << 0) /* git working tree was dirty at build time */
#define FWMGMT_MANIFEST_FLAG_DEBUG     (1u << 1) /* debug build (asserts/symbols retained) */

struct fwmgmt_manifest_v1 {
	uint32_t magic;             /* FWMGMT_MANIFEST_MAGIC */
	uint16_t fmt_version;       /* FWMGMT_MANIFEST_FMT_VERSION */
	uint16_t flags;             /* FWMGMT_MANIFEST_FLAG_* */
	uint32_t crc32_image;       /* CRC32 over the whole firmware.elf payload */
	uint32_t image_size;        /* size of firmware.elf in bytes */
	uint8_t  git_hash[20];      /* SHA-1 of the PX4 git HEAD that produced this image */
	uint32_t semver;            /* 0xMM_mm_pp_rr (Major.minor.patch.variant) */
	uint64_t build_timestamp;   /* unix epoch seconds */
	uint32_t board_id;          /* FWMGMT_MANIFEST_BOARD_* */
	uint32_t fmu_min_compat;    /* minimum FMU semver this firmware accepts (0 = any) */
	uint32_t fmu_max_compat;    /* maximum FMU semver (0 = no upper bound) */
	uint8_t  reserved[64];
	uint32_t crc32_self;        /* CRC32 over bytes [0..123] (everything before this field) */
} __attribute__((packed));

#define FWMGMT_MANIFEST_SELF_CRC_OFFSET   124u

/*
 * Compile-time check on the layout. All three consumers (PX4 firmware,
 * tpufw daemon, host stamping tool) must agree.
 */
#ifndef FWMGMT_NO_STATIC_ASSERT
#  if defined(__cplusplus) && (__cplusplus >= 201103L)
static_assert(sizeof(struct fwmgmt_manifest_v1) == FWMGMT_MANIFEST_BYTES,
	      "fwmgmt_manifest_v1 must be 128 bytes packed");
#  elif defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(struct fwmgmt_manifest_v1) == FWMGMT_MANIFEST_BYTES,
	       "fwmgmt_manifest_v1 must be 128 bytes packed");
#  endif
#endif

/*
 * Result codes embedded in the cmdqu IPC payload (PX4IOFwMgmtIpc::result and
 * the daemon -> firmware status messages). Mirrors the PX4IO_FW_MGMT_ERR_*
 * enum but is the canonical name for the TPU<->MC link.
 */
enum fwmgmt_result_e {
	FWMGMT_RESULT_OK            = 0,
	FWMGMT_RESULT_TIMEOUT       = 1,
	FWMGMT_RESULT_IPC           = 2,
	FWMGMT_RESULT_NO_AGENT      = 3,
	FWMGMT_RESULT_BAD_PARAM     = 4,
	FWMGMT_RESULT_UNSUPPORTED   = 5,
	FWMGMT_RESULT_INTERNAL      = 6,
	FWMGMT_RESULT_VERIFY_FAIL   = 7,
	FWMGMT_RESULT_ROLLBACK      = 8,
};

#ifdef __cplusplus
} /* extern "C" */
#endif
