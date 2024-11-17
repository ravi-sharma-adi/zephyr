/*
 * Copyright (c) 2024 Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PHY_STM32U5_OTG_HS_PHY_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PHY_STM32U5_OTG_HS_PHY_H_

/* Ideally we'd generate this list to match what's coming out of the YAML.
 * Make sure it's kept up to date with the yaml file in question.
 * (dts/bindings/phy/st,stm32u5-otghs-phy.yaml)
 */

#define OTGHS_PHY_CLK_16MHZ   1
#define OTGHS_PHY_CLK_19P2MHZ 2
#define OTGHS_PHY_CLK_20MHZ   3
#define OTGHS_PHY_CLK_24MHZ   4
#define OTGHS_PHY_CLK_26MHZ   5
#define OTGHS_PHY_CLK_32MHZ   6

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PHY_STM32U5_OTG_HS_PHY_H_ */
