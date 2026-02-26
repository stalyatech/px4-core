/****************************************************************************
 * nuttx-config/include/board_memorymap.h
 ****************************************************************************/

#pragma once

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DDR start address */

#define SG2000_DDR_BASE   (0x80000000)
#define SG2000_DDR_SIZE   (0x20000000)

#ifdef CONFIG_SG2000_LINUX_CARVEOUT
#  define SG2000_LITTLECORE_CARVEOUT_BASE CONFIG_SG2000_LINUX_CARVEOUT_BASE
#  define SG2000_LITTLECORE_CARVEOUT_SIZE CONFIG_SG2000_LINUX_CARVEOUT_SIZE
#else
#  define SG2000_LITTLECORE_CARVEOUT_BASE 0x9f800000
#  define SG2000_LITTLECORE_CARVEOUT_SIZE 0x00800000
#endif

/* Kernel code memory (RX) */

#define KFLASH_START    (uintptr_t)__kflash_start
#define KFLASH_SIZE     (uintptr_t)__kflash_size
#define KSRAM_START     (uintptr_t)__ksram_start
#define KSRAM_SIZE      (uintptr_t)__ksram_size
#define KSRAM_END       (uintptr_t)__ksram_end

/* Page pool (RWX) */

#define PGPOOL_START    (uintptr_t)__pgheap_start
#define PGPOOL_SIZE     (uintptr_t)__pgheap_size
#define PGPOOL_END      (PGPOOL_START + PGPOOL_SIZE)

/* Ramdisk (RW) */

#define RAMDISK_START   (uintptr_t)__ramdisk_start
#define RAMDISK_SIZE    (uintptr_t)__ramdisk_size

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Kernel code memory (RX)  */

extern uint8_t          __kflash_start[];
extern uint8_t          __kflash_size[];

/* Kernel RAM (RW) */

extern uint8_t          __ksram_start[];
extern uint8_t          __ksram_size[];
extern uint8_t          __ksram_end[];

/* Page pool (RWX) */

extern uint8_t          __pgheap_start[];
extern uint8_t          __pgheap_size[];

/* Ramdisk (RW) */

extern uint8_t          __ramdisk_start[];
extern uint8_t          __ramdisk_size[];


