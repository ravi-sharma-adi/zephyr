/*
 * Copyright (c) 2024 Meta Platforms
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>

#include <zephyr/drivers/i3c.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(i3c, CONFIG_I3C_LOG_LEVEL);

/* Generate Names */
#define UNKNOWN_NAME_STR(i, _)                                                                     \
	{                                                                                          \
		.name = "unknown-" STRINGIFY(i)                                                    \
	}
const struct device dummy_devs[] = {
	LISTIFY(CONFIG_I3C_NUM_OF_DESC_MEM_SLABS, UNKNOWN_NAME_STR, (,)) };

/* TODO: Handle CONFIG_I3C_NUM_OF_DESC_MEM_SLABS being 0 for no memory overhead */
K_MEM_SLAB_DEFINE(i3c_common_device_desc, sizeof(struct i3c_device_desc),
		  CONFIG_I3C_NUM_OF_DESC_MEM_SLABS, 4);

struct i3c_device_desc *i3c_alloc_i3c_device_desc(void)
{
	struct i3c_device_desc *desc;

	if (k_mem_slab_alloc(&i3c_common_device_desc, (void **)&desc, K_NO_WAIT) == 0) {
		memset(desc, 0, sizeof(struct i3c_device_desc));
		*(const struct device **)&desc->dev =
			&dummy_devs[k_mem_slab_num_free_get(&i3c_common_device_desc)];
		LOG_DBG("I3C Device Desc allocated - %d free",
			k_mem_slab_num_free_get(&i3c_common_device_desc));
	} else {
		LOG_WRN("No memory left for I3C descriptors");
	}

	return desc;
}

void i3c_free_i3c_device_desc(struct i3c_device_desc *desc)
{
	k_mem_slab_free(&i3c_common_device_desc, (void *)desc);
	LOG_DBG("I3C Device Desc freed");
}

bool i3c_is_common_i3c_device_desc(struct i3c_device_desc *desc)
{
	const char *p = (const char *)desc;
	ptrdiff_t offset = p - i3c_common_device_desc.buffer;

	return (offset >= 0) &&
	       (offset < (i3c_common_device_desc.info.block_size *
		   i3c_common_device_desc.info.num_blocks)) &&
	       ((offset % i3c_common_device_desc.info.block_size) == 0);
}

/* TODO: Handle CONFIG_I3C_I2C_NUM_OF_DESC_MEM_SLABS being 0 for no memory overhead */
K_MEM_SLAB_DEFINE(i3c_i2c_common_device_desc, sizeof(struct i3c_i2c_device_desc),
	CONFIG_I3C_I2C_NUM_OF_DESC_MEM_SLABS, 4);

struct i3c_i2c_device_desc *i3c_alloc_i3c_i2c_device_desc(void)
{
	struct i3c_i2c_device_desc *desc;

	if (k_mem_slab_alloc(&i3c_i2c_common_device_desc, (void **)&desc, K_NO_WAIT) == 0) {
		memset(desc, 0, sizeof(struct i3c_i2c_device_desc));
		LOG_INF("I2C Device Desc allocated - %d free",
			k_mem_slab_num_free_get(&i3c_i2c_common_device_desc));
	} else {
		LOG_WRN("No memory left for I2C descriptors");
	}

	return desc;
}

void i3c_free_i3c_i2c_device_desc(struct i3c_i2c_device_desc *desc)
{
	k_mem_slab_free(&i3c_i2c_common_device_desc, (void *)desc);
	LOG_DBG("I2C Device Desc freed");
}

bool i3c_is_common_i3c_i2c_device_desc(struct i3c_i2c_device_desc *desc)
{
	const char *p =  (const char *)desc;
	ptrdiff_t offset = p - i3c_i2c_common_device_desc.buffer;

	return (offset >= 0) &&
	       (offset < (i3c_i2c_common_device_desc.info.block_size *
		   i3c_i2c_common_device_desc.info.num_blocks)) &&
	       ((offset % i3c_i2c_common_device_desc.info.block_size) == 0);
}
