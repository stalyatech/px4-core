/****************************************************************************
 *
 *   Copyright (C) 2025 Stalya Inc. All rights reserved.
 *
 *   Author: Volvox <volvox@stalya.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <nuttx/mmcsd.h>
#include <nuttx/fs/partition.h>

#include "board_config.h"

#ifdef CONFIG_SAPPHIRE_SDIO

#include "sapphire_sdio.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct sdio_dev_s *g_sdio_dev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: partition_handler
 *
 * @brief Register a partition on the eMMCSD driver.
 *
 * @param part Pointer to the partition structure.
 * @param arg Pointer to an integer containing the partition number.
 *
 * This function is a callback for the parse_block_partition() function.
 * It checks if the current partition matches the given partition number
 * and registers the partition if so.
 *
 * @note This function is called by the parse_block_partition() function
 * and is not intended to be called directly by the application.
 *
 ****************************************************************************/

static void partition_handler(FAR struct partition_s *part, FAR void *arg)
{
	unsigned partition = *(int *)arg;
	char devname[] = "/dev/mmcsd0p0";

	if (partition < 10 && part->index == partition) {
		devname[sizeof(devname) - 2] = partition + 48;
		register_blockpartition(devname, 0, "/dev/mmcsd0", part->firstblock, part->nblocks);
	}
}

/****************************************************************************
 * Name: board_register_partition
 *
 * @brief Register a partition on the eMMCSD driver.
 *
 * @param partition The number of the partition to register.
 *
 * @return Zero (OK) is returned on success; A negated errno value is returned
 * to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_register_partition(uint32_t partition)
{
	return parse_block_partition("/dev/mmcsd0", partition_handler, &partition);
}

/****************************************************************************
 * Name: mmcsd_cardinserted
 *
 * Description:
 *   Check if a card is inserted into the selected MMCSD slot
 *
 ****************************************************************************/

static bool mmcsd_cardinserted(int slotno)
{
	return sapphire_sdio_get_card_detect();
}

/****************************************************************************
 * Name: mmcsd_isr_callback
 *
 * Description:
 *   Check if a card is inserted into the selected MMCSD slot
 *
 ****************************************************************************/

static void mmcsd_isr_callback(void *arg)
{
	bool cd;

	cd = mmcsd_cardinserted(SDIO_SLOTNO);
	finfo("Card detect: %d\n", cd);
	sdio_mediachange(arg, cd);

#ifdef HAVE_AUTOMOUNTER
	/* Let the automounter know about the insertion event */

	sapphire_automount_event(SDIO_SLOTNO, cd);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_emmcsd_init
 *
 * Description:
 *   Configure the eMMCSD driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_emmcsd_init(void)
{
	int ret = 0;

	finfo("Initializing SDIO slot %d\n", SDIO_SLOTNO);
	g_sdio_dev = sdio_initialize(SDIO_SLOTNO);
	if (g_sdio_dev == NULL)
		{
			ferr("ERROR: Failed to initialize SDIO slot %d\n", SDIO_SLOTNO);
			return -ENODEV;
		}

	finfo("Bind SDIO to the MMC/SD driver, minor=%d\n", SDIO_MINOR);
	ret = mmcsd_slotinitialize(SDIO_MINOR, g_sdio_dev);
	if (ret != OK)
		{
			ferr("ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
			return ret;
		}

	sapphire_sdio_set_card_isr(g_sdio_dev, &mmcsd_isr_callback, g_sdio_dev);

	finfo("Successfully bound SDIO to the MMC/SD driver\n");

	/* Assume that the SD card is inserted.
	 * The Ti60F225 board does not have the CD pin wired.
	 */

	sdio_mediachange(g_sdio_dev, sapphire_sdio_get_card_detect());

	return OK;
}

#endif /* CONFIG_SAPPHIRE_SDIO */
