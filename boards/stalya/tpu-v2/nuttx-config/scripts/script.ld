/****************************************************************************
 * nuttx-config/scripts/script.ld
 ****************************************************************************/

#include <nuttx/config.h>


MEMORY
{
    kflash (rx)  : ORIGIN = 0x9f800000, LENGTH = 2048K
    ksram (rwx)  : ORIGIN = 0x9fa00000, LENGTH = 2048K
    ramdisk (rwx): ORIGIN = 0x9fc00000, LENGTH = 2048K
    pgram (rwx)  : ORIGIN = 0x9fe00000, LENGTH = 2048K
}

OUTPUT_ARCH("riscv")

PHDRS
{
  kflash  PT_LOAD FLAGS(0x5);
  ksram   PT_LOAD FLAGS(0x6);
}

/* Keep/start from SG2000 head object entry. */

EXTERN(__start)
ENTRY(__start)

/* Provide the kernel boundaries */

__kflash_start = ORIGIN(kflash);
__kflash_size = LENGTH(kflash);
__ksram_start = ORIGIN(ksram);
__ksram_size = LENGTH(ksram);
__ksram_end = ORIGIN(ksram) + LENGTH(ksram);

/* Page heap */

__pgheap_start = ORIGIN(pgram);
__pgheap_size = LENGTH(pgram);

/* Application ramdisk */

__ramdisk_start = ORIGIN(ramdisk);
__ramdisk_size = LENGTH(ramdisk);
__ramdisk_end  = ORIGIN(ramdisk) + LENGTH(ramdisk);

SECTIONS
{
  . = ORIGIN(kflash);

  /* where the global variable out-of-bounds detection information located */

#ifdef CONFIG_MM_KASAN_GLOBAL
  .kasan.unused : {
    *(.data..LASANLOC*)
  }
  .kasan.global : {
    KEEP (*(.data..LASAN0))
    KEEP (*(.data.rel.local..LASAN0))
  }
  .kasan.shadows : {
    *(.kasan.shadows)
  }
#endif

  .text :
    {
      _stext = . ;
      *(.text)
      *(.text.*)
      *(.gnu.warning)
      *(.stub)
      *(.glue_7)
      *(.glue_7t)
      *(.jcr)

      /* C++ support:  The .init and .fini sections contain specific logic
       * to manage static constructors and destructors.
       */

      *(.gnu.linkonce.t.*)
      *(.init)             /* Old ABI */
      *(.fini)             /* Old ABI */
      _etext = . ;
    } > kflash :kflash

  .rodata :
    {
      _srodata = . ;
      *(.rodata)
      *(.rodata1)
      *(.rodata.*)
      *(.gnu.linkonce.r*)
      _erodata = . ;
    } > kflash :kflash

  .tdata :
    {
      _stdata = ABSOLUTE(.);
      *(.tdata .tdata.* .gnu.linkonce.td.*);
      _etdata = ABSOLUTE(.);
    } > kflash :kflash

  .tbss :
    {
      _stbss = ABSOLUTE(.);
      *(.tbss .tbss.* .gnu.linkonce.tb.* .tcommon);
      _etbss = ABSOLUTE(.);
    } > kflash :kflash

  _eronly = ABSOLUTE(.);

  .data :
    {
      _sdata = . ;
      *(.data)
      *(.data1)
      *(.data.*)
      *(.gnu.linkonce.d*)
      . = ALIGN(4);
      _edata = . ;
    } > ksram :ksram

  .bss :
    {
      _sbss = . ;
      *(.bss)
      *(.bss.*)
      *(.sbss)
      *(.sbss.*)
      *(.gnu.linkonce.b*)
      *(COMMON)
      . = ALIGN(32);
      _ebss = . ;
    } > ksram :ksram

  /* Page tables here, align to 4K boundary */

  .pgtables (NOLOAD) : ALIGN(0x1000)
    {
      *(.pgtables)
      . = ALIGN(4);
    } > ksram :ksram

  /* Stack top */

  .stack_top :
    {
      . = ALIGN(32);
      _ebss = ABSOLUTE(.);
    } > ksram :ksram

  /* Stabs debugging sections.    */

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
