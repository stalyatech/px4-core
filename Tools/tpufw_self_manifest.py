#!/usr/bin/env python3
"""
Stamp the TPU-v2 firmware manifest into a freshly linked ELF.

Locates the `.note.fwmanifest` section (a 128-byte block placed there by
`fw_manifest.c`), recomputes the manifest with real values for git hash,
build timestamp, image size and image CRC, and writes both the updated ELF
and a sidecar `firmware.manifest` file consumed by the mission computer's
`tpufw` daemon.

Layout is locked by `src/lib/tpu_fwmgmt_proto/fwmgmt_proto.h` and must stay
in sync with both the firmware C definition and the daemon's parser.

The CRC32 over the firmware image is computed with the manifest's
`crc32_image` and `crc32_self` fields zeroed in place. Both consumers
(firmware self-attestation and the MC daemon) must apply the same masking
when recomputing, otherwise verification fails.
"""

from __future__ import annotations

import argparse
import binascii
import os
import struct
import subprocess
import sys
import time
from pathlib import Path

MANIFEST_MAGIC = 0x464D5746  # 'FWMF'
MANIFEST_FMT_VERSION = 1
MANIFEST_BYTES = 128
MANIFEST_BOARD_TPU_V2 = 0x32555054  # 'TPU2'

FLAG_DIRTY = 1 << 0
FLAG_DEBUG = 1 << 1

# struct fwmgmt_manifest_v1 layout (little-endian, packed)
#   uint32 magic
#   uint16 fmt_version
#   uint16 flags
#   uint32 crc32_image
#   uint32 image_size
#   uint8[20] git_hash
#   uint32 semver
#   uint64 build_timestamp
#   uint32 board_id
#   uint32 fmu_min_compat
#   uint32 fmu_max_compat
#   uint8[64] reserved
#   uint32 crc32_self
MANIFEST_FMT = "<IHHII20sIQIII64sI"
assert struct.calcsize(MANIFEST_FMT) == MANIFEST_BYTES, \
    f"manifest layout drift: {struct.calcsize(MANIFEST_FMT)} != {MANIFEST_BYTES}"

# Fields zeroed before computing crc32_image. Offsets within the 128-byte
# manifest blob.
CRC32_IMAGE_OFFSET = 8        # uint32 magic + uint16 fmt_version + uint16 flags
CRC32_SELF_OFFSET = 124       # last uint32


def _git(cmd: list[str], cwd: Path) -> str:
    try:
        return subprocess.check_output(["git", *cmd], cwd=str(cwd),
                                       stderr=subprocess.DEVNULL).decode().strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


def collect_git_metadata(repo_root: Path) -> tuple[bytes, bool]:
    sha_hex = _git(["rev-parse", "HEAD"], repo_root)
    git_hash = bytes.fromhex(sha_hex) if len(sha_hex) == 40 else b"\x00" * 20
    dirty = bool(_git(["status", "--porcelain"], repo_root))
    return git_hash, dirty


def parse_semver(text: str) -> int:
    """Parse 'v1.16.1' or '1.16.1-beta' into 0xMM_mm_pp_rr."""
    if not text:
        return 0
    if text.startswith("v") or text.startswith("V"):
        text = text[1:]
    core = text.split("-", 1)[0]
    parts = core.split(".")
    try:
        major = int(parts[0]) & 0xFF
    except (IndexError, ValueError):
        major = 0
    try:
        minor = int(parts[1]) & 0xFF
    except (IndexError, ValueError):
        minor = 0
    try:
        patch = int(parts[2]) & 0xFF
    except (IndexError, ValueError):
        patch = 0
    variant = 0
    return (major << 24) | (minor << 16) | (patch << 8) | variant


def find_section(elf_bytes: bytes, name: str) -> tuple[int, int]:
    """Return (file_offset, size) for the named section. Pure-python ELF
    parser limited to what we need.
    """
    if elf_bytes[:4] != b"\x7fELF":
        raise SystemExit("input is not an ELF file")

    ei_class = elf_bytes[4]   # 1 = ELF32, 2 = ELF64
    ei_data = elf_bytes[5]    # 1 = LE, 2 = BE
    if ei_data != 1:
        raise SystemExit("only little-endian ELF supported")
    if ei_class not in (1, 2):
        raise SystemExit(f"unknown EI_CLASS {ei_class}")

    is64 = (ei_class == 2)
    # Header field offsets / formats per ELF spec.
    if is64:
        # Elf64_Ehdr: e_shoff at +40 (8 bytes), e_shentsize at +58 (2),
        # e_shnum at +60 (2), e_shstrndx at +62 (2)
        e_shoff = struct.unpack_from("<Q", elf_bytes, 0x28)[0]
        e_shentsize = struct.unpack_from("<H", elf_bytes, 0x3A)[0]
        e_shnum = struct.unpack_from("<H", elf_bytes, 0x3C)[0]
        e_shstrndx = struct.unpack_from("<H", elf_bytes, 0x3E)[0]
        # Elf64_Shdr: sh_name (4) sh_type (4) sh_flags (8) sh_addr (8)
        #            sh_offset (8) sh_size (8) ...
        sh_off_field = 0x18
        sh_size_field = 0x20
        sh_unpack = "<II"
    else:
        e_shoff = struct.unpack_from("<I", elf_bytes, 0x20)[0]
        e_shentsize = struct.unpack_from("<H", elf_bytes, 0x2E)[0]
        e_shnum = struct.unpack_from("<H", elf_bytes, 0x30)[0]
        e_shstrndx = struct.unpack_from("<H", elf_bytes, 0x32)[0]
        sh_off_field = 0x10
        sh_size_field = 0x14
        sh_unpack = "<II"

    # Locate .shstrtab so we can resolve section names.
    shstr_hdr = e_shoff + e_shstrndx * e_shentsize
    shstr_off = struct.unpack_from(
        "<Q" if is64 else "<I", elf_bytes, shstr_hdr + sh_off_field)[0]
    shstr_size = struct.unpack_from(
        "<Q" if is64 else "<I", elf_bytes, shstr_hdr + sh_size_field)[0]
    shstrtab = elf_bytes[shstr_off:shstr_off + shstr_size]

    target = name.encode() + b"\x00"
    for i in range(e_shnum):
        hdr = e_shoff + i * e_shentsize
        sh_name = struct.unpack_from("<I", elf_bytes, hdr)[0]
        # Section name is a NUL-terminated string at shstrtab[sh_name:].
        end = shstrtab.find(b"\x00", sh_name)
        sect_name = shstrtab[sh_name:end]
        if sect_name + b"\x00" == target:
            sh_offset = struct.unpack_from(
                "<Q" if is64 else "<I", elf_bytes, hdr + sh_off_field)[0]
            sh_size = struct.unpack_from(
                "<Q" if is64 else "<I", elf_bytes, hdr + sh_size_field)[0]
            return sh_offset, sh_size

    raise SystemExit(f"section {name!r} not found in ELF")


def build_manifest(*, image_crc: int, image_size: int, git_hash: bytes,
                   flags: int, semver: int, board_id: int) -> bytes:
    blob = struct.pack(
        MANIFEST_FMT,
        MANIFEST_MAGIC,
        MANIFEST_FMT_VERSION,
        flags,
        image_crc,
        image_size,
        git_hash[:20].ljust(20, b"\x00"),
        semver,
        int(time.time()),
        board_id,
        0,                      # fmu_min_compat
        0,                      # fmu_max_compat
        b"\x00" * 64,           # reserved
        0,                      # crc32_self placeholder
    )
    self_crc = binascii.crc32(blob[:CRC32_SELF_OFFSET]) & 0xFFFFFFFF
    return blob[:CRC32_SELF_OFFSET] + struct.pack("<I", self_crc)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--elf", required=True, type=Path,
                    help="Path to the linked PX4 ELF (modified in-place).")
    ap.add_argument("--manifest-out", required=True, type=Path,
                    help="Sidecar firmware.manifest written next to the ELF.")
    ap.add_argument("--repo-root", default=Path.cwd(), type=Path,
                    help="Root of the git repo for the build (default: cwd).")
    ap.add_argument("--semver", default="",
                    help="Version string parsed into the manifest semver field.")
    ap.add_argument("--board-id", default=MANIFEST_BOARD_TPU_V2, type=lambda v: int(v, 0),
                    help="board_id field (default: 'TPU2').")
    ap.add_argument("--debug", action="store_true",
                    help="Set FWMGMT_MANIFEST_FLAG_DEBUG.")
    args = ap.parse_args()

    elf_path = args.elf
    if not elf_path.is_file():
        raise SystemExit(f"ELF not found: {elf_path}")

    elf_bytes = bytearray(elf_path.read_bytes())
    sec_off, sec_size = find_section(elf_bytes, ".note.fwmanifest")
    if sec_size != MANIFEST_BYTES:
        raise SystemExit(
            f".note.fwmanifest size {sec_size} != expected {MANIFEST_BYTES}")

    git_hash, dirty = collect_git_metadata(args.repo_root)
    flags = 0
    if dirty:
        flags |= FLAG_DIRTY
    if args.debug:
        flags |= FLAG_DEBUG

    semver = parse_semver(args.semver)

    # The CRC32 over the ELF must be reproducible by the daemon and the
    # firmware itself, both of which see the *final* manifest in place. We
    # therefore compute the CRC with image_size set to its real value and
    # only the two CRC fields (crc32_image, crc32_self) zeroed — those are
    # the bits that are not knowable until after the CRC is computed.
    image_size = len(elf_bytes)

    placeholder = build_manifest(image_crc=0, image_size=image_size,
                                 git_hash=git_hash, flags=flags,
                                 semver=semver, board_id=args.board_id)
    placeholder = (placeholder[:CRC32_SELF_OFFSET] + b"\x00" * 4)
    elf_bytes[sec_off:sec_off + MANIFEST_BYTES] = placeholder

    image_crc = binascii.crc32(bytes(elf_bytes)) & 0xFFFFFFFF

    final_manifest = build_manifest(image_crc=image_crc,
                                    image_size=image_size,
                                    git_hash=git_hash, flags=flags,
                                    semver=semver, board_id=args.board_id)
    elf_bytes[sec_off:sec_off + MANIFEST_BYTES] = final_manifest

    elf_path.write_bytes(bytes(elf_bytes))
    args.manifest_out.parent.mkdir(parents=True, exist_ok=True)
    args.manifest_out.write_bytes(final_manifest)

    print(f"[tpufw] stamped {elf_path.name}: "
          f"crc=0x{image_crc:08x} size={image_size} "
          f"semver=0x{semver:08x} dirty={int(dirty)} "
          f"git={git_hash.hex()[:12]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
