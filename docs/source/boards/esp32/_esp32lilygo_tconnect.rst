.. _esp32lilygo_tconnect:

=======================
LilyGO T-Connect (ESP32-S3)
=======================

.. image:: ../../../images/esp32lilygo_tconnect.jpg
   :width: 300px
   :alt: LilyGO T-Connect

The LilyGO T-Connect is a board based on the ESP32-S3 MCU.

Features
========

- MCU: ESP32-S3-WROOM-1-N8R8 (8MB Flash, 8MB PSRAM)
- CAN Transceiver: TJA1051T/3
- RGB LED: APA102
- USB:
    - USB-C for programming and native USB
    - USB-A for 5V power output

Default ArduPilot Configuration
===============================

The `defaults.parm` file for this board configures it for a DroneCAN-based vehicle with an ELRS receiver.

- **RC Input:** `SERIAL1` is configured for RCIN (ELRS) on Pins 12 (TX) and 13 (RX).
- **DroneCAN:** The CAN bus is enabled on `CAN1` (Pins 4 and 5).
- **Battery Monitor:** The battery monitor is configured to use DroneCAN (`BATT_MONITOR` = 8).
- **Motors/ESCs:** The motor outputs are configured for DroneCAN ESCs.
- **Steering Servo:** The steering servo function (Servo 3) is assigned to DroneCAN.

Where to Buy
============

- `LilyGO Store <https://www.lilygo.cc/products/t-connect>`_
