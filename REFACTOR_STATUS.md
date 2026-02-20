# ESP32 Refactor Status

## PR #32245: Build System Infrastructure (Step 1)
**Branch:** `esp32-hwdef-infrastructure`  
**Focus:** Pure infrastructure improvements and auto-detection logic.

### Primary Files & Directories
- `Tools/ardupilotwaf/esp32.py`: Waf build tool integration for ESP-IDF.
- `libraries/AP_HAL_ESP32/hwdef/scripts/esp32_hwdef.py`: The new data-driven hardware definition generator.
- `libraries/AP_HAL_ESP32/MIGRATION.md`: Documentation for upgrading boards to the new system.
- `Tools/ardupilotwaf/boards.py`: Logic for toolchain selection (Xtensa vs RISC-V).

---

## Full Functional Refactor (Preserved Work)
**Branch:** `feature/esp32-full-refactor`  
**Focus:** Comprehensive update including driver migrations, DroneCAN enhancements, and advanced diagnostics.

### Primary Files & Directories
- `libraries/AP_HAL_ESP32/`: Core HAL implementation (I2C, SPI, RMT, TWAI migrations).
- `libraries/AP_DroneCAN/`: DNA server improvements and MAVLink bridge.
- `libraries/GCS_MAVLink/`: PARAM_EXT protocol implementation.
- `libraries/AP_Filesystem/`: ESP32 VFS and FatFS compatibility.
\n---\nLast CI reset: Sun Feb 22 10:35:27 AM AEDT 2026
