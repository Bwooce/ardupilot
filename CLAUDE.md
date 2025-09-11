## Development Notes

- The hwdef.dat file is not binary, use the cat tool to inspect it as the Read tool refuses
- waf build should always be done with -j 8 to use more cores
- ESP32 logging levels should be controlled via esp_log_level_set() in the HAL init code, not by changing individual log statements
- Use ESP_LOGE/LOGW/LOGI/LOGD/LOGV consistently and control verbosity centrally
- DNA allocation encoding issue: Expected 8 bytes but sending 35+ bytes in multi-frame transfer
- Remote node gets 4 DNA requests before response - investigate timing delays
- remember that the user will do the build, and it's always ardupilot rover
- remember to always set a timeout on serial communication scripts.
- Always cast to (unsigned long) and use %lX or %lu for uint32_t on ESP32
- ESP32 has different type sizes than x86_64
- Remember only the user will compile ardupilot, only for rover, and do not initiate this yourself unless requested.////////