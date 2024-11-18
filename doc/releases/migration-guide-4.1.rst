:orphan:

.. _migration_4.1:

Migration guide to Zephyr v4.1.0 (Working Draft)
################################################

This document describes the changes required when migrating your application from Zephyr v4.0.0 to
Zephyr v4.1.0.

Any other changes (not directly related to migrating applications) can be found in
the :ref:`release notes<zephyr_4.1>`.

.. contents::
    :local:
    :depth: 2

Build System
************

Kernel
******

Boards
******

Modules
*******

Mbed TLS
========

Trusted Firmware-M
==================

LVGL
====

Device Drivers and Devicetree
*****************************

Controller Area Network (CAN)
=============================

Display
=======

* Displays using the MIPI DBI driver which set their MIPI DBI mode via the
  ``mipi-mode`` property in devicetree should now use a string property of
  the same name, like so:

  .. code-block:: devicetree

    /* Legacy display definition */

    st7735r: st7735r@0 {
        ...
        mipi-mode = <MIPI_DBI_MODE_SPI_4WIRE>;
        ...
    };

    /* New display definition */

    st7735r: st7735r@0 {
        ...
        mipi-mode = "MIPI_DBI_MODE_SPI_4WIRE";
        ...
    };


Enhanced Serial Peripheral Interface (eSPI)
===========================================

GNSS
====

Input
=====

Interrupt Controller
====================

LED Strip
=========

Sensors
=======

Serial
======

Regulator
=========

Bluetooth
*********

Bluetooth HCI
=============

Bluetooth Mesh
==============

Bluetooth Audio
===============

Bluetooth Classic
=================

Bluetooth Host
==============

Bluetooth Crypto
================

Networking
**********

Other Subsystems
****************

Flash map
=========

hawkBit
=======

MCUmgr
======

Modem
=====

Architectures
*************
