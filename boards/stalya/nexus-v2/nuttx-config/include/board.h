/****************************************************************************
 * nuttx-config/include/board.h
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

#pragma once

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Global Macro Definitions
 ****************************************************************************/

#define BOARD_MAKE_GPIO(port, pin, irq) { (port), (pin), (irq) }
#define BOARD_MAKE_PIN(port, pin, irq) 	(((port) << 16U) | ((pin) << 8U) | (irq))

#define BOARD_GET_PRT(iomask) 					((iomask >> 16U) & 0xff)
#define BOARD_GET_PIN(iomask) 					((iomask >>  8U) & 0xff)
#define BOARD_GET_IRQ(iomask) 					((iomask >>  0U) & 0xff)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO pins used by the GPIO Subsystem */

#define BOARD_NGPIOOUT		5		/* Amount of GPIO Output pins */
#define BOARD_NGPIOIN			0		/* Amount of GPIO Input without Interruption */
#define BOARD_NGPIOINT		1		/* Amount of GPIO Input w/ Interruption pins */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

typedef struct dev_partition
{
  uint32_t offset;        /* Offset from the beginning of MTD */
  uint32_t nblocks;       /* Number of blocks */
  char     devpath[32];   /* Device path */
} dev_partition_s;


/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sapphire_boardinitialize
 ****************************************************************************/

void sapphire_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
