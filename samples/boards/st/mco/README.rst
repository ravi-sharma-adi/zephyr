.. zephyr:code-sample:: stm32_mco
   :name: Master Clock Output (MCO)
   :relevant-api: pinctrl_interface

   Output an internal clock for external use by the application.

Overview
********

This sample is a minimum application to demonstrate how to output one of the internal clocks for
external use by the application.

Requirements
************

The SoC should support MCO functionality and use a pin that has the MCO alternate function.
To support another board, add a dts overlay file in boards folder.
Make sure that the output clock is enabled in dts overlay file.
Depending on the stm32 serie, several clock source are possible for each MCOx and prescaler.
The clock source is set by the DTS and converted in LL_RCC_MCO2SOURCE_xxx which is the parameter
for the LL_RCC_ConfigMCO function.
If prescaler is set by the DTS, the property prescaler = <5>; will be converted in LL_RCC_MCO2_DIV_5
which is the parameter for the LL_RCC_ConfigMCO function.


Building and Running
********************

.. zephyr-app-commands::
   :zephyr-app: samples/boards/st/mco
   :board: nucleo_u5a5zj_q
   :goals: build flash

After flashing, the LSE clock will be output on the MCO pin enabled in Device Tree.
The clock can be observed using a probing device, such as a logic analyzer.
