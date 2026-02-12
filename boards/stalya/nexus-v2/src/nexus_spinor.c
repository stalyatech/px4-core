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
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/fs/fs.h>
#include <arch/board/board.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <sapphire_spi.h>
#include "board_config.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

dev_partition_s g_main_parts[CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTMAIN]={0};
#if CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTRESV > 0
dev_partition_s g_resv_parts[CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTRESV]={0};
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Can't support the IS25XP device if not enabled */

#define HAVE_IS25XP		1
#if !defined(CONFIG_MTD_IS25XP)
#  undef HAVE_IS25XP
#endif

/* Can't support IS25XP features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_IS25XP
#endif

/* Can't support both FAT and SMARTFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_SMARTFS)
#  warning "Can't support both FAT and SMARTFS -- using FAT"
#endif

#if CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTMAIN == 0
#  error "At least one main partition must be defined!"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef HAVE_IS25XP
/****************************************************************************
 * Name: create_main_partitions
 *
 * Description:
 *   Create the main MTD partitions.
 *
 ****************************************************************************/

int create_main_partitions(struct mtd_dev_s *mtd, off_t offset, off_t nblocks)
{
  struct mtd_dev_s *part[CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTMAIN]={0};
  char blockname[32];
  char charname[32];
  char mntpoint[32];
  int ret, i;

  /* Now create main MTD FLASH partitions */

  for (i = 0;
       i < CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTMAIN;
       offset += nblocks, i++)
    {
      syslog(LOG_INFO, "INFO: Partition %d. Block offset=%lu, size=%lu\n",
                       i, (unsigned long)offset, (unsigned long)nblocks);

      /* Create the partition */

      part[i] = mtd_partition(mtd, offset, nblocks);
      if (!part[i])
        {
          syslog(LOG_ERR, "ERROR: Failed to create main MTD partition\n");
          return -ENODEV;
        }

      /* Prepare to mount the partition data */

      snprintf(blockname, sizeof(blockname), "/dev/mtdblock%d", i);
      snprintf(charname, sizeof(charname), "/dev/mtd%d", i);
      snprintf(mntpoint, sizeof(mntpoint), "/data%d", i);

      /* Initialize to provide flash file system on the MTD partitions */

      ret = board_spiflash_init(part[i], blockname, mntpoint, true);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Flash file system initialization failed: %d\n", -ret);
        }

      /* Save the partition information */

      g_main_parts[i].offset  = offset;
      g_main_parts[i].nblocks = nblocks;
      strlcpy(g_main_parts[i].devpath, blockname, sizeof(blockname));
    }

  return offset;
}

/****************************************************************************
 * Name: create_resv_partitions
 *
 * Description:
 *   Create the reserved MTD partitions.
 *
 ****************************************************************************/
#if CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTRESV > 0
static int create_resv_partitions(struct mtd_dev_s *mtd, off_t offset, off_t nblocks)
{
  struct mtd_dev_s *part[CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTRESV]={0};
  char blockname[32];
  char charname[32];
  int ret, i;

  /* Now create reserved MTD FLASH partitions */

  for (i = 0;
       i < CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTRESV;
       offset += nblocks, i++)
    {
      syslog(LOG_INFO, "INFO: Partition %d. Block offset=%lu, size=%lu\n",
                       i, (unsigned long)offset, (unsigned long)nblocks);

      /* Create the partition */

      part[i] = mtd_partition(mtd, offset, nblocks);
      if (!part[i])
        {
          syslog(LOG_ERR, "ERROR: Failed to create reserved MTD partition\n");
          return -ENODEV;
        }

      /* Initialize to provide an FTL block driver on the MTD FLASH
       * interface
       */

      snprintf(blockname, sizeof(blockname), "/dev/imgblock%d", i);
      snprintf(charname, sizeof(charname), "/dev/img%d", i);

      ret = ftl_initialize_by_path(blockname, part[i]);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: ftl_initialize %s failed: %d\n", blockname, ret);
          return ret;
        }

      /* Now create a character device on the block device */

      ret = bchdev_register(blockname, charname, false);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n", charname, ret);
          return ret;
        }

      /* Save the partition information */

      g_resv_parts[i].offset  = offset;
      g_resv_parts[i].nblocks = nblocks;
      strlcpy(g_resv_parts[i].devpath, blockname, sizeof(blockname));
    }

  return offset;
}
#endif /* CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTRESV */
#endif /* HAVE_IS25XP */

/****************************************************************************
 * Name: create_img_partition
 *
 * Description:
 *   Create the image MTD partitions.
 *
 ****************************************************************************/

static int create_img_partition(struct mtd_dev_s *mtd, off_t offset, off_t nblocks, uint16_t minor)
{
	struct mtd_dev_s *mtd_part;
  char blockname[32];
  int ret;

	/* Create the partition */

	mtd_part = mtd_partition(mtd, offset, nblocks);
	if (!mtd_part)
		{
			syslog(LOG_ERR, "ERROR: Failed to create the MTD partition\n");
			return -ENODEV;
		}

	/* Prepare to mount the partition data */

	snprintf(blockname, sizeof(blockname), "/dev/imgblock%d", minor);

	/* Now register a block device */

	ret = register_mtddriver(blockname, mtd_part, 0777, NULL);
	if (ret < 0)
		{
			syslog(LOG_ERR, "ERROR: Failed to register (%s) MTD partition\n", blockname);
			return ret;
		}

  return offset;
}

/****************************************************************************
 * Name: board_spinor_init
 *
 * Description:
 *   Initialize and register the IS25 FLASH file system.
 *
 ****************************************************************************/

int board_spinor_init(void)
{
#ifdef HAVE_IS25XP
  struct mtd_geometry_s geo;
  struct mtd_dev_s *mtd;
	struct spi_dev_s *spi;
  uint32_t blkpererase;
  uint32_t blkreserved;
  off_t nblocks, blkboot;
  int ret;

	/* Initialize the SPI0 device for OTA block device */

#ifdef CONFIG_SAPPHIRE_SPI0
	spi = sapphire_spibus_initialize(0);
	if (spi == NULL)
		{
			syslog(LOG_ERR,
						 "ERROR: Failed to initialize SPI0\n");
			return -ENODEV;
		}

  /* Now bind the SPI interface to the IS25 SPI FLASH driver */

  mtd = is25xp_initialize(spi, 0);
  if (!mtd)
    {
      syslog(LOG_ERR,
						 "ERROR: Failed to bind SPI0 to the IS25XP FLASH driver\n");
      return -ENODEV;
    }

  /* Get the device geometry */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY,
                        (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR,
						 "ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

  nblocks = (geo.neraseblocks * geo.erasesize) / geo.blocksize;
	blkboot = 0x800000 / geo.blocksize;

  /* Create the first image partition */

  ret = create_img_partition(mtd, 0, blkboot, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
						 "ERROR: Failed to create image partition\n");
      return ret;
    }

  /* Create the second image partition */

  ret = create_img_partition(mtd, blkboot, nblocks-blkboot, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
						 "ERROR: Failed to create image partition\n");
      return ret;
    }
#endif /* CONFIG_SAPPHIRE_SPI0 */

#ifdef CONFIG_SAPPHIRE_SPI1
	spi = sapphire_spibus_initialize(1);
	if (spi == NULL)
		{
			syslog(LOG_ERR,
						 "ERROR: Failed to initialize SPI0\n");
			return -ENODEV;
		}

  /* Now bind the SPI interface to the IS25 SPI FLASH driver */

  mtd = is25xp_initialize(spi, 1);
  if (!mtd)
    {
      syslog(LOG_ERR,
						 "ERROR: Failed to bind SPI1 to the IS25XP FLASH driver\n");
      return -ENODEV;
    }

  /* Get the device geometry */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY,
                        (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR,
						 "ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

  /* Determine the size of each partition.  Make each partition an even
   * multiple of the erase block size (perhaps not using some space at the
   * end of the FLASH).
   */

  /* The reserved area will be used for firmware upgrade/recovery */

#ifdef CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_SIZERESV
  blkreserved = (CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_SIZERESV / geo.erasesize);
#else
	blkreserved = 0;
#endif
  blkpererase = geo.erasesize / geo.blocksize;
  nblocks     = ((geo.neraseblocks - blkreserved) / CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTMAIN) *
                blkpererase;

  /* Now create the main MTD FLASH partitions */

  ret = create_main_partitions(mtd, 0, nblocks);
  if (ret < 0)
    {
      return ret;
    }

  /* Create the reserved partition */

#if CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTRESV > 0
	off_t offset = ret;
  ret = create_resv_partitions(mtd,
                               offset,
                               blkreserved / CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTRESV *
                               blkpererase);
  if (ret < 0)
    {
      return ret;
    }
#endif /* CONFIG_SAPPHIRE_TI375C529DK_SPIFLASH_NPARTRESV */

#endif /* CONFIG_SAPPHIRE_SPI1 */

#endif /* HAVE_IS25XP */

  return OK;
}
