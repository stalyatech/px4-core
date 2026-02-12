/****************************************************************************
 * nuttx-config/include/board_memorymap.h
 ****************************************************************************/

#pragma once

#include <stdint.h>

/* Kernel code/data memory */
#define KFLASH_START    (uintptr_t)__kflash_start
#define KFLASH_SIZE     (uintptr_t)__kflash_size
#define KSRAM_START     (uintptr_t)__ksram_start
#define KSRAM_SIZE      (uintptr_t)__ksram_size
#define KSRAM_END       (uintptr_t)__ksram_end

/* Page pool */
#define PGPOOL_START    (uintptr_t)__pgheap_start
#define PGPOOL_SIZE     (uintptr_t)__pgheap_size
#define PGPOOL_END      (PGPOOL_START + PGPOOL_SIZE)

/* Optional userspace flash (protected build) */
#define UFLASH_START    (uintptr_t)__uflash_start
#define UFLASH_SIZE     (uintptr_t)__uflash_size

/* RAMDisk */
#define RAMDISK_START   (uintptr_t)__ramdisk_start
#define RAMDISK_SIZE    (uintptr_t)__ramdisk_size

extern uint8_t __kflash_start[];
extern uint8_t __kflash_size[];
extern uint8_t __ksram_start[];
extern uint8_t __ksram_size[];
extern uint8_t __ksram_end[];
extern uint8_t __pgheap_start[];
extern uint8_t __pgheap_size[];
extern uint8_t __uflash_start[];
extern uint8_t __uflash_size[];
extern uint8_t __ramdisk_start[];
extern uint8_t __ramdisk_size[];

