/*
 * Copyright (c) 2020 Framework Computer - Kieran Levin <ktl@frame.work>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_PINMUX_STM32_PINMUX_STM32G070XX_H_
#define ZEPHYR_DRIVERS_PINMUX_STM32_PINMUX_STM32G070XX_H_

/* PORT A */

#define STM32G0_PINMUX_FUNC_PA0_USART4_TX \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PA1_USART4_RX \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PA1_TIM15_CH1N \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PA2_TIM15_CH1 \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PA3_TIM15_CH2 \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PA5_USART3_TX \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PA6_USART3_CTS \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PA7_TIM14_CH1 \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PA7_TIM17_CH1 \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PA9_TIM15_BKIN \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PA15_USART4_RTS_DE_CK \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PA15_USART3_RTS_DE_CK \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

/* PORT B */

#define STM32G0_PINMUX_FUNC_PB0_USART3_RX \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB1_USART3_RTS_DE_CK \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB2_USART3_TX \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB7_USART4_CTS \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB8_USART3_TX \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PB8_TIM15_BKIN \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB9_USART3_RX \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB10_USART3_TX \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB11_USART3_RX \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB12_TIM15_BKIN \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB13_USART3_CTS \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PB13_TIM15_CH1N \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB14_USART3_RTS_DE_CK \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PB14_TIM15_CH1 \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PB15_TIM15_CH1N \
		(STM32_PINMUX_ALT_FUNC_4 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PB15_TIM15_CH2 \
		(STM32_PINMUX_ALT_FUNC_5 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

/* PORT C */

#define STM32G0_PINMUX_FUNC_PC1_TIM15_CH1 \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PC2_SPI2_MISO \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PC2_TIM15_CH2 \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PC3_SPI2_MOSI \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PC4_USART3_TX \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PC4_USART1_TX \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PC5_USART3_RX \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PC5_USART1_RX \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PC8_TIM3_CH3 \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PC8_TIM1_CH1 \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PC9_I2S_CKIN \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PC9_TIM3_CH4 \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PC9_TIM1_CH2 \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PC10_USART3_TX \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PC10_USART4_TX \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PC10_TIM1_CH3 \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PC11_USART3_RX \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PC11_USART4_RX \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PC11_TIM1_CH4 \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PC12_TIM14_CH1 \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PC15_TIM15_BKIN \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

/* PORT D */

#define STM32G0_PINMUX_FUNC_PD2_USART3_RTS_DE_CK \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PD4_USART2_RTS_DE_CK \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PD4_SPI2_MOSI \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PD4_TIM1_CH3N \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PD5_USART2_TX \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PD5_SPI1_MISO__I2S1_MCK \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PD5_TIM1_BKIN \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PD6_USART2_RX \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PD6_SPI1_MOSI__I2S1_SD \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PD8_USART3_TX \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PD8_SPI1_SCK__I2S1_CK \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)


#define STM32G0_PINMUX_FUNC_PD9_USART3_RX \
		(STM32_PINMUX_ALT_FUNC_0 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PD9_SPI1_NSS__I2S1_WS \
		(STM32_PINMUX_ALT_FUNC_1 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#define STM32G0_PINMUX_FUNC_PD9_TIM1_BKIN2 \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

/* PORT F */

#define STM32G0_PINMUX_FUNC_PF1_TIM15_CH1N \
		(STM32_PINMUX_ALT_FUNC_2 | STM32_OTYPER_PUSH_PULL | STM32_PUPDR_NO_PULL)

#endif /* ZEPHYR_DRIVERS_PINMUX_STM32_PINMUX_STM32G070XX_H_ */
