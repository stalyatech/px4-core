/****************************************************************************
 * nuttx-config/scripts/script.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

MEMORY
{
  progmem (rx)  : ORIGIN = 0x00001000, LENGTH = 4M - 4K
  sram    (rwx) : ORIGIN = 0x00400000, LENGTH = 4M - 64K
  l2lim_sec (rw): ORIGIN = 0x007F0000, LENGTH = 32k
  l2lim_info (r): ORIGIN = 0x007F8000, LENGTH = 4k
}

OUTPUT_ARCH("riscv")

__ksram_start = ORIGIN(sram);
__ksram_size = LENGTH(sram);
__ksram_end = ORIGIN(sram) + LENGTH(sram);

EXTERN(__start)
ENTRY(__start)

PHDRS
{
    text PT_LOAD FLAGS(5); /* R + X */
    data PT_LOAD FLAGS(6); /* R + W */
}

SECTIONS
{
    .start : {
        *(.start .start.*)
    } > progmem :text

    .text : {
        _stext = ABSOLUTE(.);
        /*
        This signature provides the bootloader with a way to delay booting
        */
        . = 0x400;
        _bootdelay_signature = ABSOLUTE(.);
        FILL(0x01ecc2925d7d05c5)
        . += 8;
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.* .srodata .srodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        _etext = ABSOLUTE(.);
    } > progmem :text

    .init_section : ALIGN(4) {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > progmem :text

    _eronly = ABSOLUTE(.);

    .data : ALIGN(4) {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.sdata .sdata.* .sdata2.*)
        *(.gnu.linkonce.d.*)
        *(.gnu.linkonce.s.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > progmem :data

    PROVIDE(__global_pointer$ = _sdata + ((_edata - _sdata) / 2));

    .bss : ALIGN(4) {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.sbss .sbss.*)
        *(.gnu.linkonce.b.*)
        *(.gnu.linkonce.sb.*)
        *(COMMON)
        . = ALIGN(32);
        _ebss = ABSOLUTE(.);
    } > sram :data

    .secmem (NOLOAD) : ALIGN(4) {
    } > l2lim_sec :NONE

    .deviceinfo (NOLOAD) : ALIGN(4) {
    } > l2lim_info :NONE

    /* Stabs debugging sections. */

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
