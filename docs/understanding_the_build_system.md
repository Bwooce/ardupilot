# Understanding the ArduPilot Waf Build System

This document provides a high-level overview of the ArduPilot `waf` build system, focusing on the separation between the `configure` and `build` phases, and how this applies to different Hardware Abstraction Layers (HALs) like ChibiOS and ESP32.

## The Two Phases of Waf: `configure` and `build`

The `waf` build system operates in two distinct phases:

1.  **`configure` Phase:**
    *   **Purpose:** To inspect the system, detect compilers and libraries, and set up the entire build environment based on the selected board.
    *   **Execution:** This phase runs only when you execute `./waf configure --board <board_name>`.
    *   **Output:** It creates a build environment (a set of variables like `INCLUDES`, `CXXFLAGS`, etc.) which is then cached in the `build` directory.
    *   **Key Principle:** All board-specific and HAL-specific logic should be performed in this phase. The goal is to create a complete and correct environment so that the `build` phase can be as simple as possible.

2.  **`build` Phase:**
    *   **Purpose:** To compile the source code and link the final firmware.
    *   **Execution:** This phase runs every time you execute a build command like `./waf rover`.
    *   **Input:** It loads the cached environment created during the `configure` phase.
    *   **Key Principle:** The `build` phase should be generic. Build scripts (like `wscript` files in the libraries) should not contain complex conditional logic based on the board type. They should simply use the environment that the `configure` phase has already prepared.

## How the Build Process Applies to Different HALs

### ChibiOS (The Default)

*   **Configuration:** The logic for ChibiOS boards is handled in the main `wscript` file at the root of the repository. During the `configure` phase, when a ChibiOS board is detected, the `wscript` explicitly adds the `libraries/AP_HAL_ChibiOS` directory to the `INCLUDES` path in the environment.
*   **Build:** During the `build` phase, when a library like `AP_Baro` is compiled, it uses the global `INCLUDES` path from the environment. This allows it to find the ChibiOS-specific `canard_helpers_user.h` without needing any special logic.

### ESP32 (A Hybrid Approach)

*   **Configuration:** The ESP32 build process is a hybrid that uses both `waf` and `cmake` (for the ESP-IDF). The ESP32-specific configuration logic is located in `Tools/ardupilotwaf/esp32.py`. This file is loaded by `waf` as a tool during the `configure` phase *only* when an ESP32 board is selected.
*   **The Problem:** The `'Semaphore' in namespace 'Canard' does not name a type` error occurs because the `configure` function in `Tools/ardupilotwaf/esp32.py` does not add the `libraries/AP_HAL_ESP32` directory to the `INCLUDES` path.
*   **The Solution:** The correct fix is to add the include path to the `INCLUDES` variable within the `configure` function of `Tools/ardupilotwaf/esp32.py`. This ensures that the build environment is correctly prepared for ESP32 boards, mirroring the logic for ChibiOS boards and respecting the separation between the `configure` and `build` phases.
