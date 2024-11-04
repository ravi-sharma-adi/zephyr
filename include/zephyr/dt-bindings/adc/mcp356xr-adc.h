/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Copyright 2024 Syslinbit SCOP SAS
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_ADC_MCP356XR_ADC_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_ADC_MCP356XR_ADC_H_

#include <zephyr/dt-bindings/adc/adc.h>

#define MCP356XR_INPUT_INTERNAL_VCM (15)
#define MCP356XR_INPUT_INTERNAL_TEMPERATURE_SENSOR_DIODE_M (14)
#define MCP356XR_INPUT_INTERNAL_TEMPERATURE_SENSOR_DIODE_P (13)
#define MCP356XR_INPUT_REFIN_NEGATIVE (12)
#define MCP356XR_INPUT_REFIN_POSITIVE (11)
#define MCP356XR_INPUT_RESERVED_DO_NOT_USE (10)
#define MCP356XR_INPUT_AVDD (9)
#define MCP356XR_INPUT_AGND (8)
#define MCP356XR_INPUT_CH7 (7)
#define MCP356XR_INPUT_CH6 (6)
#define MCP356XR_INPUT_CH5 (5)
#define MCP356XR_INPUT_CH4 (4)
#define MCP356XR_INPUT_CH3 (3)
#define MCP356XR_INPUT_CH2 (2)
#define MCP356XR_INPUT_CH1 (1)
#define MCP356XR_INPUT_CH0 (0)

#define MCP356XR_ANALOG_CLOCK_NO_DIV (0)
#define MCP356XR_ANALOG_CLOCK_DIV_2 (1)
#define MCP356XR_ANALOG_CLOCK_DIV_4 (2)
#define MCP356XR_ANALOG_CLOCK_DIV_8 (3)

#define MCP356XR_BOOST_CURRENT_BIAS_DIV_2 (0)
#define MCP356XR_BOOST_CURRENT_BIAS_MUL_0_66 (1)
#define MCP356XR_BOOST_CURRENT_BIAS_MUL_1 (2)
#define MCP356XR_BOOST_CURRENT_BIAS_MUL_2 (3)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_ADC_MCP356XR_ADC_H_ */
