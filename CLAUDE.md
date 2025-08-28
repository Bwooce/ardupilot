## Development Notes

- The hwdef.dat file is not binary, use the cat tool to inspect it as the Read tool refuses
- waf build should always be done with -j 8 to use more cores
- ESP32 logging levels should be controlled via esp_log_level_set() in the HAL init code, not by changing individual log statements
- Use ESP_LOGE/LOGW/LOGI/LOGD/LOGV consistently and control verbosity centrally
- DNA allocation encoding issue: Expected 8 bytes but sending 35+ bytes in multi-frame transfer
- Remote node gets 4 DNA requests before response - investigate timing delays