.. _mimxrt700_evk:

NXP MIMXRT700-EVK
##################

Overview
********

The new i.MX RT700 CPU architecture is composed of a high-performance main-compute subsystem,
a secondary “always-on” sense-compute subsystem and specialized coprocessors.

The main-compute subsystem has a primary Arm® Cortex®-M33 running at 325 MHz, with an integrated
Cadence® Tensilica® HiFi 4 DSP for more demanding DSP and audio processing tasks.
The sense-compute subsystem has a second Arm® Cortex®-M33 and an integrated Cadence® Tensilica®
HiFi 1 DSP. This removes the need for an external sensor hub, reducing system design complexity,
footprint and BOM costs.

The HiFi4 is a high performance DSP core based upon a Very Long Instruction Word (VLIW) architecture,
which is capable of processing up to eight 32x16 MACs per instruction cycle. It can be used for offloading
high-performance numerical tasks such as audio and image processing and supports both fixed-point and
floating-point operations.

The i.MX RT700 also features NXP’s eIQ Neutron NPU, enabled with the eIQ machine learning software
development environment.

Hardware
********

- Main Compute Subsystem:
      Arm Cortex-M33 up to 325 MHz
      HiFi 4 DSP up to 325 MHz
      eIQ Neutron NPU up to 325 MHz
- Sense Compute Subsystem:
      Arm Cortex-M33 up to 250 MHz
      HiFi 1 DSP up to 250 MHz
- 7.5 MB on-chip SRAM
- Three xSPI interfaces for off-chip memory expansion, supporting up to 16b wide external memories up to 250 MHz DDR
- eUSB support with integrated PHY
- Two SD/eMMC memory card interfaces—one supporting eMMC 5.0 with HS400/DDR operation
- USB high-speed host/device controller with on-chip PHY
- A digital microphone interface supporting up to 8 channels
- Serial peripherals (UART/I²C/I3C/SPI/HSPI/SAI)
- 2.5D GPU with vector graphics acceleration and frame buffer compression
- EZH-V using RISC-V core with additional SIMD/DSP instructions
- Full openVG 1.1 support
- Up to 720p@60FPS from on-chip SRAM
- LCD Interface + MIPI DSI
- Integrated JPEG and PNG support
- CSI 8/10/16-bit parallel (via FlexIO)

For more information about the MIMXRT798 SoC and MIMXRT700-EVK board, see
these references:

- `i.MX RT700 Website`_

Supported Features
==================

NXP considers the MIMXRT700-EVK as a superset board for the i.MX RT7xx
family of MCUs. This board is a focus for NXP's Full Platform Support for
Zephyr, to better enable the entire RT7xx family. NXP prioritizes enabling
this board with new support for Zephyr features.

The default configuration can be found in the defconfig file:

   :zephyr_file:`boards/nxp/mimxrt700_evk/mimxrt700_evk_mimxrt798s_hifi1_defconfig`
   :zephyr_file:`boards/nxp/mimxrt700_evk/mimxrt700_evk_mimxrt798s_hifi4_defconfig`

Other hardware features are not currently supported by the port.

Connections and IOs
===================

The MIMXRT798 SoC has IOCON registers, which can be used to configure the
functionality of a pin.

+---------+-----------------+----------------------------+
| Name    | Function        | Usage                      |
+=========+=================+============================+
| PIO0_31 | UART0           | UART RX                    |
+---------+-----------------+----------------------------+
| PIO1_0  | UART0           | UART TX                    |
+---------+-----------------+----------------------------+
| PIO0_18 | GPIO            | GREEN LED                  |
+---------+-----------------+----------------------------+
| PIO0_9  | GPIO            | SW5                        |
+---------+-----------------+----------------------------+
| PIO8_14 | UART19          | UART TX                    |
+---------+-----------------+----------------------------+
| PIO8_15 | UART19          | UART RX                    |
+---------+-----------------+----------------------------+

System Clock
============

The MIMXRT700 EVK is configured to use the Systick
as a source for the system clock.

HiFi1 DSP Core
==================

One can build a Zephyr application for the i.MX RT700 HiFi 1  DSP core by targeting the HiFi 1
SOC. Xtensa toolchain supporting RT700 DSP cores is included in Zephyr SDK.

To build the hello_world sample for the i.MX RT700 HiFi 1 DSP core:

.. code-block:: shell

   $ west build -b mimxrt700_evk/mimxrt798s/hifi1 samples/hello_world

HiFi4 DSP Core
==================

One can build a Zephyr application for the i.MX RT700 HiFi 4  DSP core by targeting the HiFi 4
SOC. Xtensa toolchain supporting RT700 DSP cores is included in Zephyr SDK.

To build the hello_world sample for the i.MX RT700 HiFi 4 DSP core:

.. code-block:: shell

   $ west build -b mimxrt700_evk/mimxrt798s/hifi4 samples/hello_world

Programming and Debugging
*************************

Build and flash applications as usual (see :ref:`build_an_application` and
:ref:`application_run` for more details).

Configuring a Console
=====================

Connect a USB cable from your PC to J54, and use the serial terminal of your choice
(minicom, putty, etc.) with the following settings:

- Speed: 115200
- Data: 8 bits
- Parity: None
- Stop bits: 1

.. _i.MX RT700 Website:
   https://www.nxp.com/products/processors-and-microcontrollers/arm-microcontrollers/i-mx-rt-crossover-mcus/i-mx-rt700-crossover-mcu-with-arm-cortex-m33-npu-dsp-and-gpu-cores:i.MX-RT700
