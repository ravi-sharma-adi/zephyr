.. zephyr:code-sample:: uart
   :name: UART circular mode
   :relevant-api: uart_interface

   Read data from the console and echo it back when half and full irq.

Overview
********

This sample demonstrates how to use the UART serial driver in circular mode.
It Read data from the console and echo it back on half and on full transfer complete interrupts.

We use Zephyr Ring Buffer API to instantly receive data once available.
User will need to adjust rx or tx buffer sizes based on usage in order to receive
the whole data frame without loss.

By default, the UART peripheral that is normally assigned to the Zephyr shell
is used, hence the majority of boards should be  able to run this sample

Building and Running
********************

Build and flash the sample as follows, changing ``nucleo_g071rb`` for
your board:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/st/uart/circular_dma
   :board: nucleo_g071rb
   :goals: build flash
   :compact:

Sample Output
=============

.. code-block:: console

    Enter message to fill RX buffer size and press enter :
    # Type e.g. 0123456789abcd
    0123456789abcd
