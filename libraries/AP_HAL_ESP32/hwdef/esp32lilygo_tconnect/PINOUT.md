# ESP32 LilyGo T-Connect Pinout

This file documents the pinout for the `esp32lilygo_tconnect` board.

## eLRS Telemetry Board Connection

To connect an eLRS telemetry board, use the following pins:

| Function | Pin |
|---|---|
| TX | 12 |
| RX | 13 |

## RGB LED Status System

The four onboard RGB LEDs are used to display the vehicle's status at a glance. Each LED is dedicated to a specific subsystem.

| LED # | Subsystem | State | Color & Pattern |
| :--- | :--- | :--- | :--- |
| **1** | **System / Arming** | Pre-arm Checks Failed | **Solid Red** |
| | | Ready to Arm | **Pulsing Yellow** |
| | | Armed | **Solid Green** |
| | | Failsafe Active | **Pulsing Blue** |
| **2** | **GPS** | No GPS Detected | **Off** |
| | | No Fix | **Blinking Blue** |
| | | 3D Fix or better | **Solid Blue** |
| **3** | **Telemetry** | No Link | **Off** |
| | | Link w/o GCS Heartbeat | **Blinking Cyan** |
| | | GCS Link Active | **Solid Cyan** |
| **4** | **Battery** | Good (> 50%) | **Solid Green** |
| | | Low (25-50%) | **Solid Yellow** |
| | | Critical (< 25%) | **Solid Red** |
| | | Failsafe Triggered | **Blinking Red (Fast)** |