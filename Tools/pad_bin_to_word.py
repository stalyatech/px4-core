#!/usr/bin/env python3
"""Pad a raw firmware .bin to a 4-byte boundary with 0xff.

The px4io bootloader's PROG_MULTI handler rejects any chunk whose byte count is
not a multiple of 4 (STM32F1 flash is half-word programmed and the protocol
treats writes as uint32_t words). When the firmware blob itself isn't a
multiple of 4, the FMU uploader's last partial chunk fails the bootloader's
check and leaves the IO with an erased-but-uncommitted app image.

0xff matches the post-erase flash state, so trailing pad bytes don't change the
CRC the bootloader computes during verify.
"""
import os
import sys

if len(sys.argv) != 2:
    sys.stderr.write("usage: pad_bin_to_word.py <bin>\n")
    sys.exit(2)

path = sys.argv[1]
size = os.path.getsize(path)
pad = (-size) & 3

if pad:
    with open(path, "ab") as f:
        f.write(b"\xff" * pad)
