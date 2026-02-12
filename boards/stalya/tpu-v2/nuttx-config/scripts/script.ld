/****************************************************************************
 * nuttx-config/scripts/script.ld
 ****************************************************************************/

MEMORY
{
    kflash (rx) : ORIGIN = 0x80200000, LENGTH = 2048K
    ksram  (rwx): ORIGIN = 0x80400000, LENGTH = 2048K
    pgram  (rwx): ORIGIN = 0x80600000, LENGTH = 4096K
    ramdisk (rwx) : ORIGIN = 0x80A00000, LENGTH = 16M
}

OUTPUT_ARCH("riscv")
ENTRY(sg2000_start)
EXTERN(sg2000_start)

__kflash_start = ORIGIN(kflash);
__kflash_size = LENGTH(kflash);
__ksram_start = ORIGIN(ksram);
__ksram_size = LENGTH(ksram);
__ksram_end = ORIGIN(ksram) + LENGTH(ksram);

__pgheap_start = ORIGIN(pgram);
__pgheap_size = LENGTH(pgram) + LENGTH(ramdisk);

__ramdisk_start = ORIGIN(ramdisk);
__ramdisk_size = LENGTH(ramdisk);
__ramdisk_end  = ORIGIN(ramdisk) + LENGTH(ramdisk);

SECTIONS
{
  . = 0x80200000;

  .text :
    {
      _stext = .;
      *(.text)
      *(.text.*)
      *(.gnu.warning)
      *(.stub)
      *(.glue_7)
      *(.glue_7t)
      *(.jcr)
      *(.gnu.linkonce.t.*)
      *(.init)
      *(.fini)
      _etext = .;
    } > kflash

  .rodata :
    {
      _srodata = .;
      *(.rodata)
      *(.rodata1)
      *(.rodata.*)
      *(.gnu.linkonce.r*)
      _erodata = .;
    } > kflash

  .init_section :
    {
      _sinit = .;
      KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
      KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
      _einit = .;
    } > kflash

  _eronly = ABSOLUTE(.);

  .data :
    {
      _sdata = .;
      *(.data)
      *(.data1)
      *(.data.*)
      *(.gnu.linkonce.d*)
      . = ALIGN(4);
      _edata = .;
    } > ksram AT > kflash

  .bss :
    {
      _sbss = .;
      *(.bss)
      *(.bss.*)
      *(.sbss)
      *(.sbss.*)
      *(.gnu.linkonce.b*)
      *(COMMON)
      . = ALIGN(32);
      _ebss = .;
    } > ksram

  .stack_top :
    {
      . = ALIGN(32);
      _ebss = ABSOLUTE(.);
    } > ksram

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
