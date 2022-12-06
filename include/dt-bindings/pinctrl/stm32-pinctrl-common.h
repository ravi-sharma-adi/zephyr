/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_STM32_PINCTRL_COMMON_H_
#define ZEPHYR_STM32_PINCTRL_COMMON_H_

/**
 * @brief numerical IDs for IO ports
 */

#define	STM32_PORTA 0	/* IO port A */
#define	STM32_PORTB 1	/* .. */
#define	STM32_PORTC 2
#define	STM32_PORTD 3
#define	STM32_PORTE 4
#define	STM32_PORTF 5
#define	STM32_PORTG 6
#define	STM32_PORTH 7
#define	STM32_PORTI 8
#define	STM32_PORTJ 9
#define	STM32_PORTK 10
#define	STM32_PORTZ 11	/* IO port Z */

#ifndef STM32_PORTS_MAX
#define STM32_PORTS_MAX (STM32_PORTK + 1)
#endif

#define STM32_PINMUX_FUNC_GPIO 0
#define STM32_PINMUX_FUNC_ANALOG (STM32_PINMUX_FUNC_ALT_MAX)

/**
 * @brief helper macro to encode an IO port pin in a numerical format
 */
#define STM32PIN(_port, _pin) \
	(_port << 4 | _pin)

#endif	/* ZEPHYR_STM32_PINCTRL_COMMON_H_ */
