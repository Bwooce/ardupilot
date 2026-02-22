# ESP32 HAL TODO

## RMT peripheral for SoftSigReader

**File:** `SoftSigReaderInt.cpp`

The software signal reader currently uses GPIO interrupts for RC input pulse timing.
The ESP32 RMT (Remote Control Transceiver) peripheral could handle this in hardware,
reducing CPU overhead and improving timing accuracy.

- IDF docs: https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/rmt.html
- The RCOutput driver already uses RMT for output; input would be the counterpart
- Note: IDF 6.0 uses the new `rmt_rx` API (not the legacy `rmt_driver_install`)

## Remove defaults.parm legacy fallback in boards.py

**File:** `Tools/ardupilotwaf/boards.py:1155`

```python
# TODO: remove once hwdef.dat support is in place
defaults_file = 'libraries/AP_HAL_ESP32/hwdef/%s/defaults.parm' % self.get_name()
```

This fallback loads defaults.parm from board directories. Once upstream boards have
migrated to the new hwdef.dat infrastructure, this can be removed so parameters come
entirely from hwdef configuration. Blocked until upstream migration is complete.
