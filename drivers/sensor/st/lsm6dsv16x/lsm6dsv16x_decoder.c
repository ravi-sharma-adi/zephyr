/* ST Microelectronics LSM6DSV16X 6-axis IMU sensor driver
 *
 * Copyright (c) 2023 Google LLC
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lsm6dsv16x.h"
#include "lsm6dsv16x_decoder.h"
#include <zephyr/dt-bindings/sensor/lsm6dsv16x.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LSM6DSV16X_DECODER, CONFIG_SENSOR_LOG_LEVEL);

#ifdef CONFIG_LSM6DSV16X_STREAM
static uint32_t accel_period_ns[] = {
	[LSM6DSV16X_XL_BATCHED_AT_1Hz875] = UINT32_C(1000000000000) / 1875,
	[LSM6DSV16X_XL_BATCHED_AT_7Hz5] = UINT32_C(1000000000000) / 7500,
	[LSM6DSV16X_XL_BATCHED_AT_15Hz] = UINT32_C(1000000000) / 15,
	[LSM6DSV16X_XL_BATCHED_AT_30Hz] = UINT32_C(1000000000) / 30,
	[LSM6DSV16X_XL_BATCHED_AT_60Hz] = UINT32_C(1000000000) / 60,
	[LSM6DSV16X_XL_BATCHED_AT_120Hz] = UINT32_C(1000000000) / 120,
	[LSM6DSV16X_XL_BATCHED_AT_240Hz] = UINT32_C(1000000000) / 240,
	[LSM6DSV16X_XL_BATCHED_AT_480Hz] = UINT32_C(1000000000) / 480,
	[LSM6DSV16X_XL_BATCHED_AT_960Hz] = UINT32_C(1000000000) / 960,
	[LSM6DSV16X_XL_BATCHED_AT_1920Hz] = UINT32_C(1000000000) / 1920,
	[LSM6DSV16X_XL_BATCHED_AT_3840Hz] = UINT32_C(1000000000) / 3840,
	[LSM6DSV16X_XL_BATCHED_AT_7680Hz] = UINT32_C(1000000000) / 7680,
};

static uint32_t gyro_period_ns[] = {
	[LSM6DSV16X_GY_BATCHED_AT_1Hz875] = UINT32_C(1000000000000) / 1875,
	[LSM6DSV16X_GY_BATCHED_AT_7Hz5] = UINT32_C(1000000000000) / 7500,
	[LSM6DSV16X_GY_BATCHED_AT_15Hz] = UINT32_C(1000000000) / 15,
	[LSM6DSV16X_GY_BATCHED_AT_30Hz] = UINT32_C(1000000000) / 30,
	[LSM6DSV16X_GY_BATCHED_AT_60Hz] = UINT32_C(1000000000) / 60,
	[LSM6DSV16X_GY_BATCHED_AT_120Hz] = UINT32_C(1000000000) / 120,
	[LSM6DSV16X_GY_BATCHED_AT_240Hz] = UINT32_C(1000000000) / 240,
	[LSM6DSV16X_GY_BATCHED_AT_480Hz] = UINT32_C(1000000000) / 480,
	[LSM6DSV16X_GY_BATCHED_AT_960Hz] = UINT32_C(1000000000) / 960,
	[LSM6DSV16X_GY_BATCHED_AT_1920Hz] = UINT32_C(1000000000) / 1920,
	[LSM6DSV16X_GY_BATCHED_AT_3840Hz] = UINT32_C(1000000000) / 3840,
	[LSM6DSV16X_GY_BATCHED_AT_7680Hz] = UINT32_C(1000000000) / 7680,
};

#if defined(CONFIG_LSM6DSV16X_ENABLE_TEMP)
static uint32_t temp_period_ns[] = {
	[LSM6DSV16X_TEMP_BATCHED_AT_1Hz875] = UINT32_C(1000000000000) / 1875,
	[LSM6DSV16X_TEMP_BATCHED_AT_15Hz] = UINT32_C(1000000000) / 15,
	[LSM6DSV16X_TEMP_BATCHED_AT_60Hz] = UINT32_C(1000000000) / 60,
};
#endif
#endif /* CONFIG_LSM6DSV16X_STREAM */

static int lsm6dsv16x_get_shift(enum sensor_channel channel,
				int accel_fs, int gyro_fs, int8_t *shift)
{
	int ret = 0;

	switch (channel) {
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
		switch (accel_fs) {
		case LSM6DSV16X_DT_FS_2G:
			*shift = 5;
			break;
		case LSM6DSV16X_DT_FS_4G:
			*shift = 6;
			break;
		case LSM6DSV16X_DT_FS_8G:
			*shift = 7;
			break;
		case LSM6DSV16X_DT_FS_16G:
			*shift = 8;
			break;
		default:
			LOG_ERR("bad accel fs = %d", accel_fs);
			return -EINVAL;
		}
		break;

	case SENSOR_CHAN_GYRO_XYZ:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
		switch (gyro_fs) {
		case LSM6DSV16X_DT_FS_125DPS:
			*shift = 2;
			break;
		case LSM6DSV16X_DT_FS_250DPS:
			*shift = 3;
			break;
		case LSM6DSV16X_DT_FS_500DPS:
			*shift = 4;
			break;
		case LSM6DSV16X_DT_FS_1000DPS:
			*shift = 5;
			break;
		case LSM6DSV16X_DT_FS_2000DPS:
			*shift = 6;
			break;
		case LSM6DSV16X_DT_FS_4000DPS:
			*shift = 7;
			break;
		default:
			LOG_ERR("bad gyro fs = %d", gyro_fs);
			return -EINVAL;
		}
		break;

#if defined(CONFIG_LSM6DSV16X_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP:
		*shift = 16; /* specification Q15.16 */
		break;
#endif
	default:
		LOG_ERR("bad channel num = %d", channel);
		return -EINVAL;
	}

	return ret;
}

static int lsm6dsv16x_convert_raw_to_q31(const struct lsm6dsv16x_decoder_header *hdr,
					 enum sensor_channel chan, int32_t sample,
					 q31_t *out)
{
	int32_t whole;
	int32_t fraction;
	int64_t intermediate;
	int8_t shift;
	int rc;

	rc = lsm6dsv16x_get_shift(chan, hdr->accel_fs, hdr->gyro_fs, &shift);
	if (rc != 0) {
		return rc;
	}

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z: {
		int64_t dval;
		struct sensor_value val;

		/* Sensitivity is exposed in ug/LSB */
		/* Convert to m/s^2 */
		dval = (int64_t)(sample) * lsm6dsv16x_calc_accel_gain(hdr->accel_fs);
		sensor_ug_to_ms2(dval, &val);

		whole = val.val1;
		fraction = val.val2;
		break;
	}
	case SENSOR_CHAN_GYRO_XYZ:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z: {
		int64_t dval;
		struct sensor_value val;

		/* Sensitivity is exposed in udps/LSB */
		/* So, calculate value in 10 udps unit and then to rad/s */
		dval = (int64_t)(sample) * lsm6dsv16x_calc_gyro_gain(hdr->gyro_fs) / 10;
		sensor_10udegrees_to_rad(dval, &val);

		whole = val.val1;
		fraction = val.val2;
		break;
	}
#if defined(CONFIG_LSM6DSV16X_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP: {
		int32_t micro_c;

		/* convert units to micro Celsius. Raw temperature samples are
		 * expressed in 256 LSB/deg_C units. And LSB output is 0 at 25 C.
		 */
		micro_c = (sample * 1000000) / 256;

		whole = micro_c / 1000000 + 25;
		fraction = micro_c % 1000000;

		break;
	}
#endif
	default:
		return -ENOTSUP;
	}

	intermediate = ((int64_t)whole * INT64_C(1000000) + fraction);
	if (shift < 0) {
		intermediate =
			intermediate * ((int64_t)INT32_MAX + 1) * (1 << -shift) / INT64_C(1000000);
	} else if (shift > 0) {
		intermediate =
			intermediate * ((int64_t)INT32_MAX + 1) / ((1 << shift) * INT64_C(1000000));
	}
	*out = CLAMP(intermediate, INT32_MIN, INT32_MAX);

	return 0;
}

static int lsm6dsv16x_decoder_get_frame_count(const uint8_t *buffer,
					      struct sensor_chan_spec chan_spec,
					      uint16_t *frame_count)
{
	struct lsm6dsv16x_fifo_data *data = (struct lsm6dsv16x_fifo_data *)buffer;
	const struct lsm6dsv16x_decoder_header *header = &data->header;

	if (chan_spec.chan_idx != 0) {
		return -ENOTSUP;
	}

	if (!header->is_fifo) {
		switch (chan_spec.chan_type) {
		case SENSOR_CHAN_ACCEL_X:
		case SENSOR_CHAN_ACCEL_Y:
		case SENSOR_CHAN_ACCEL_Z:
		case SENSOR_CHAN_ACCEL_XYZ:
		case SENSOR_CHAN_GYRO_X:
		case SENSOR_CHAN_GYRO_Y:
		case SENSOR_CHAN_GYRO_Z:
		case SENSOR_CHAN_GYRO_XYZ:
		case SENSOR_CHAN_DIE_TEMP:
			*frame_count = 1;
			return 0;
		default:
			return -ENOTSUP;
		}

		return 0;
	}

#ifdef CONFIG_LSM6DSV16X_STREAM
	*frame_count = data->fifo_count;
#endif
	return 0;
}

#ifdef CONFIG_LSM6DSV16X_STREAM
static int lsm6dsv16x_decode_fifo(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				  uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct lsm6dsv16x_fifo_data *edata = (const struct lsm6dsv16x_fifo_data *)buffer;
	const uint8_t *buffer_end;
	const struct lsm6dsv16x_decoder_header *header = &edata->header;
	int count = 0;
	uint8_t fifo_tag;
	uint16_t xl_count = 0, gy_count = 0;

#if defined(CONFIG_LSM6DSV16X_ENABLE_TEMP)
	uint16_t temp_count = 0;
#endif

	buffer += sizeof(struct lsm6dsv16x_fifo_data);
	buffer_end = buffer + LSM6DSV16X_FIFO_SIZE(edata->fifo_count);

	((struct sensor_data_header *)data_out)->base_timestamp_ns = edata->header.timestamp;

	while (count < max_count && buffer < buffer_end) {
		const uint8_t *frame_end = buffer;
		uint8_t skip_frame;

		skip_frame = 0;
		frame_end += LSM6DSV16X_FIFO_ITEM_LEN;

		fifo_tag = (buffer[0] >> 3);

		switch (fifo_tag) {
		case LSM6DSV16X_XL_NC_TAG: {
			struct sensor_three_axis_data *out = data_out;
			int16_t x, y, z;
			int rc;

			xl_count++;
			if ((uintptr_t)buffer < *fit) {
				/* This frame was already decoded, move on to the next frame */
				buffer = frame_end;
				continue;
			}

			if (!SENSOR_CHANNEL_IS_ACCEL(chan_spec.chan_type)) {
				buffer = frame_end;
				continue;
			}

			out->header.base_timestamp_ns = edata->header.timestamp;
			out->readings[count].timestamp_delta =
					(xl_count - 1) * accel_period_ns[edata->accel_batch_odr];

			x = *(int16_t *)&buffer[1];
			y = *(int16_t *)&buffer[3];
			z = *(int16_t *)&buffer[5];

			rc = lsm6dsv16x_get_shift(SENSOR_CHAN_ACCEL_XYZ, header->accel_fs,
						  header->gyro_fs, &out->shift);
			if (rc != 0) {
				return -EINVAL;
			}

			lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_ACCEL_X,
						      x, &out->readings[count].x);
			lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_ACCEL_Y,
						      y, &out->readings[count].y);
			lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_ACCEL_Z,
						      z, &out->readings[count].z);
			break;
		}
		case LSM6DSV16X_GY_NC_TAG: {
			struct sensor_three_axis_data *out = data_out;
			int16_t x, y, z;
			int rc;

			gy_count++;
			if ((uintptr_t)buffer < *fit) {
				/* This frame was already decoded, move on to the next frame */
				buffer = frame_end;
				continue;
			}

			if (!SENSOR_CHANNEL_IS_GYRO(chan_spec.chan_type)) {
				buffer = frame_end;
				continue;
			}

			out->header.base_timestamp_ns = edata->header.timestamp;
			out->readings[count].timestamp_delta =
					(gy_count - 1) * gyro_period_ns[edata->gyro_batch_odr];

			x = *(int16_t *)&buffer[1];
			y = *(int16_t *)&buffer[3];
			z = *(int16_t *)&buffer[5];

			rc = lsm6dsv16x_get_shift(SENSOR_CHAN_GYRO_XYZ, header->accel_fs,
						  header->gyro_fs, &out->shift);
			if (rc != 0) {
				return -EINVAL;
			}

			lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_GYRO_X,
						      x, &out->readings[count].x);
			lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_GYRO_Y,
						      y, &out->readings[count].y);
			lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_GYRO_Z,
						      z, &out->readings[count].z);
			break;
		}
#if defined(CONFIG_LSM6DSV16X_ENABLE_TEMP)
		case LSM6DSV16X_TEMPERATURE_TAG: {
			struct sensor_q31_data *out = data_out;
			int16_t t;
			int rc;

			temp_count++;
			if ((uintptr_t)buffer < *fit) {
				/* This frame was already decoded, move on to the next frame */
				buffer = frame_end;
				continue;
			}

			if (chan_spec.chan_type != SENSOR_CHAN_DIE_TEMP) {
				buffer = frame_end;
				continue;
			}

			out->header.base_timestamp_ns = edata->header.timestamp;
			out->readings[count].timestamp_delta =
					(temp_count - 1) * temp_period_ns[edata->temp_batch_odr];

			t = *(int16_t *)&buffer[1];

			rc = lsm6dsv16x_get_shift(SENSOR_CHAN_DIE_TEMP, header->accel_fs,
						  header->gyro_fs, &out->shift);
			if (rc != 0) {
				return -EINVAL;
			}

			lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_DIE_TEMP,
						      t, &out->readings[count].temperature);
			break;
		}
#endif
		default:
			/* skip unhandled FIFO tag */
			buffer = frame_end;
			LOG_DBG("unknown FIFO tag %02x", fifo_tag);
			continue;
		}

		buffer = frame_end;
		*fit = (uintptr_t)frame_end;
		count++;
	}

	return count;
}
#endif /* CONFIG_LSM6DSV16X_STREAM */

static int lsm6dsv16x_decode_sample(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				    uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct lsm6dsv16x_rtio_data *edata = (const struct lsm6dsv16x_rtio_data *)buffer;
	const struct lsm6dsv16x_decoder_header *header = &edata->header;
	int rc;

	if (*fit != 0) {
		return 0;
	}
	if (max_count == 0 || chan_spec.chan_idx != 0) {
		return -EINVAL;
	}

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ: {
		if (edata->has_accel == 0) {
			return -ENODATA;
		}

		struct sensor_three_axis_data *out = data_out;

		out->header.base_timestamp_ns = edata->header.timestamp;
		out->header.reading_count = 1;

		rc = lsm6dsv16x_get_shift(SENSOR_CHAN_ACCEL_XYZ, header->accel_fs, header->gyro_fs,
					  &out->shift);
		if (rc != 0) {
			return -EINVAL;
		}

		lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_ACCEL_X,
					      edata->acc[0], &out->readings[0].x);
		lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_ACCEL_Y,
					      edata->acc[1], &out->readings[0].y);
		lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_ACCEL_Z,
					      edata->acc[2], &out->readings[0].z);
		*fit = 1;
		return 1;
	}
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ: {
		if (edata->has_gyro == 0) {
			return -ENODATA;
		}

		struct sensor_three_axis_data *out = data_out;

		out->header.base_timestamp_ns = edata->header.timestamp;
		out->header.reading_count = 1;

		rc = lsm6dsv16x_get_shift(SENSOR_CHAN_GYRO_XYZ, header->accel_fs, header->gyro_fs,
					  &out->shift);
		if (rc != 0) {
			return -EINVAL;
		}

		lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_GYRO_X,
					      edata->gyro[0], &out->readings[0].x);
		lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_GYRO_Y,
					      edata->gyro[1], &out->readings[0].y);
		lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_GYRO_Z,
					      edata->gyro[2], &out->readings[0].z);
		*fit = 1;
		return 1;
	}
#if defined(CONFIG_LSM6DSV16X_ENABLE_TEMP)
	case SENSOR_CHAN_DIE_TEMP: {
		if (edata->has_temp == 0) {
			return -ENODATA;
		}

		struct sensor_q31_data *out = data_out;

		out->header.base_timestamp_ns = edata->header.timestamp;
		out->header.reading_count = 1;

		rc = lsm6dsv16x_get_shift(SENSOR_CHAN_DIE_TEMP, header->accel_fs, header->gyro_fs,
					  &out->shift);
		if (rc != 0) {
			return -EINVAL;
		}

		lsm6dsv16x_convert_raw_to_q31(header, SENSOR_CHAN_DIE_TEMP,
					      edata->temp, &out->readings[0].temperature);
		*fit = 1;
		return 1;
	}
#endif
	default:
		return -EINVAL;
	}
}

static int lsm6dsv16x_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				     uint32_t *fit, uint16_t max_count, void *data_out)
{
#ifdef CONFIG_LSM6DSV16X_STREAM
	const struct lsm6dsv16x_decoder_header *header =
		(const struct lsm6dsv16x_decoder_header *)buffer;

	if (header->is_fifo) {
		return lsm6dsv16x_decode_fifo(buffer, chan_spec, fit, max_count, data_out);
	}
#endif

	return lsm6dsv16x_decode_sample(buffer, chan_spec, fit, max_count, data_out);
}

static int lsm6dsv16x_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
					    size_t *frame_size)
{
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		*base_size = sizeof(struct sensor_three_axis_data);
		*frame_size = sizeof(struct sensor_three_axis_sample_data);
		return 0;
	case SENSOR_CHAN_DIE_TEMP:
		*base_size = sizeof(struct sensor_q31_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static bool lsm6dsv16x_decoder_has_trigger(const uint8_t *buffer, enum sensor_trigger_type trigger)
{
	return false;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = lsm6dsv16x_decoder_get_frame_count,
	.get_size_info = lsm6dsv16x_decoder_get_size_info,
	.decode = lsm6dsv16x_decoder_decode,
	.has_trigger = lsm6dsv16x_decoder_has_trigger,
};

int lsm6dsv16x_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}
