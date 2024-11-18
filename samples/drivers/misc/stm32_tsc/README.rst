.. zephyr:code-sample:: stm32_tsc
   :name: STM32 Touch Sensing Controller (TSC) Driver Demo

   Capacitive touch sensing with STM32 TSC

Overview
********

A simple demo for STM32 TSC peripheral and its driver. It can be used with interrupts
and input subsystem, or just with polling.

Building and Running
********************

This application can be built and executed as follows:

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/misc/stm32_tsc
   :board: stm32u083c_dk
   :host-os: unix
   :goals: build


To build for another supported board, change "stm32u083c_dk" to that board's
name (if supported).

Sample Output
=============

.. code-block:: console

   *** Booting Zephyr OS build 50720f835b51 ***
   TSC input event: dev tsc@40024000, type 239, sync 1, code 2, value 2722
   TSC input event: dev tsc@40024000, type 239, sync 1, code 2, value 2722
   TSC input event: dev tsc@40024000, type 239, sync 1, code 2, value 2692
   TSC input event: dev tsc@40024000, type 239, sync 1, code 2, value 2665
   TSC input event: dev tsc@40024000, type 239, sync 1, code 2, value 2662
   TSC input event: dev tsc@40024000, type 239, sync 1, code 2, value 2712
   TSC input event: dev tsc@40024000, type 239, sync 1, code 2, value 2702
   TSC input event: dev tsc@40024000, type 239, sync 1, code 2, value 2706
   TSC input event: dev tsc@40024000, type 239, sync 1, code 2, value 2693
