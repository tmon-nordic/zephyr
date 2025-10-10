/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2019-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sample_usbd.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_msc.h>
#include <zephyr/fs/fs.h>
#include <stdio.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static struct usbd_context *sample_usbd;


#include <zephyr/drivers/disk.h>

USBD_DEFINE_MSC_LUN(fake, "FAKE", "Zephyr", "FakeDisk", "0.00");

#define FAKE_SECTOR_COUNT 0xF000000
#define FAKE_SECTOR_SIZE  512

static int disk_fake_access_status(struct disk_info *disk)
{
	return DISK_STATUS_OK;
}

static int disk_fake_access_read(struct disk_info *disk, uint8_t *buff,
				uint32_t sector, uint32_t count)
{
	return 0;
}

static int disk_fake_access_write(struct disk_info *disk, const uint8_t *buff,
				 uint32_t sector, uint32_t count)
{
	return 0;
}

static int disk_fake_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	switch (cmd) {
	case DISK_IOCTL_CTRL_SYNC:
		break;
	case DISK_IOCTL_GET_SECTOR_COUNT:
		*(uint32_t *)buff = FAKE_SECTOR_COUNT;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		*(uint32_t *)buff = FAKE_SECTOR_SIZE;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(uint32_t *)buff = 1U;
		break;
	case DISK_IOCTL_CTRL_INIT:
	case DISK_IOCTL_CTRL_DEINIT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int disk_fake_access_init(struct disk_info *disk)
{
	return disk_fake_access_ioctl(disk, DISK_IOCTL_CTRL_INIT, NULL);
}

static const struct disk_operations fake_disk_ops = {
	.init = disk_fake_access_init,
	.status = disk_fake_access_status,
	.read = disk_fake_access_read,
	.write = disk_fake_access_write,
	.ioctl = disk_fake_access_ioctl,
};

static struct disk_info fake_info = {
	.name = "FAKE",
	.ops = &fake_disk_ops,
};

int main(void)
{
	int ret;

	disk_access_register(&fake_info);

	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	ret = usbd_enable(sample_usbd);
	if (ret) {
		LOG_ERR("Failed to enable device support");
		return ret;
	}

	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	LOG_INF("The device is put in USB mass storage mode");

	return 0;
}
