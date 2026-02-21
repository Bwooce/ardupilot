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

Connectivity and Default Configuration
======================================

The `defaults.parm` file for this board configures it for a DroneCAN-based vehicle with an ELRS receiver.

**RC Input (ELRS)**

The board is pre-configured for a CRSF/ELRS receiver on `SERIAL1`.

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - Function
     - Pin
   * - SERIAL1_TX
     - 12
   * - SERIAL1_RX
     - 13

**DroneCAN**

The CAN bus is enabled by default on `CAN1`.

.. list-table::
   :widths: 25 75
   :header-rows: 1

   * - Function
     - Pin
   * - CAN1_TX
     - 4
   * - CAN1_RX
     - 5

The following DroneCAN peripherals are enabled by default:

- **Battery Monitor:** `BATT_MONITOR` = 8
- **ESCs:** DroneCAN ESCs on `CAN1`
- **Steering Servo:** The steering servo function (Servo 3) is assigned to DroneCAN.

**RGB LED Status System**

The four onboard RGB LEDs are used to display the vehicle's status at a glance. Each LED is dedicated to a specific subsystem.

.. list-table::
   :widths: 10 20 25 45
   :header-rows: 1

   * - LED #
     - Subsystem
     - State
     - Color & Pattern
   * - **1**
     - **System / Arming**
     - Pre-arm Checks Failed
     - **Solid Red**
   * -
     -
     - Ready to Arm
     - **Pulsing Yellow**
   * -
     -
     - Armed
     - **Solid Green**
   * -
     -
     - Failsafe Active
     - **Pulsing Blue**
   * - **2**
     - **GPS**
     - No GPS Detected
     - **Off**
   * -
     -
     - No Fix
     - **Blinking Blue**
   * -
     -
     - 3D Fix or better
     - **Solid Blue**
   * - **3**
     - **Telemetry**
     - No Link
     - **Off**
   * -
     -
     - Link w/o GCS Heartbeat
     - **Blinking Cyan**
   * -
     -
     - GCS Link Active
     - **Solid Cyan**
   * - **4**
     - **Battery**
     - Good (> 50%)
     - **Solid Green**
   * -
     -
     - Low (25-50%)
     - **Solid Yellow**
   * -
     -
     - Critical (< 25%)
     - **Solid Red**
   * -
     -
     - Failsafe Triggered
     - **Blinking Red (Fast)**

Where to Buy
============

- `LilyGO Store <https://www.lilygo.cc/products/t-connect>`_