## Development Notes

- The hwdef.dat file is not binary, use the cat tool to inspect it as the Read tool refuses to read it because of it's assumptions about what files with .dat contain.
- waf build should always be done with -j 16 to use 16 cores
- ESP32 logging levels should be controlled via esp_log_level_set() in the HAL init code, not by changing individual log statements
- Use ESP_LOGE/LOGW/LOGI/LOGD/LOGV consistently and control verbosity centrally in the ESP32 HAL debug configuration
- DNA allocation encoding issue: Expected 8 bytes but sending 35+ bytes in multi-frame transfer
- remember that the user will do the build, and it's always ardupilot rover
- remember to always set a timeout on serial communication scripts.
- Always cast to (unsigned long) and use %lX or %lu for uint32_t on ESP32
- ESP32 has different type sizes than x86_64, so testing theories locally on intel does not prove anything
- Only the ardupilot-rover-builder agent will compile ardupilot, only for rover, and do not initiate this in any other agent or context.
- **ESP32 Build Commands**: ALWAYS use `./agent_build_wrapper.sh` for waf commands to ensure proper ESP-IDF environment:
  - `./agent_build_wrapper.sh configure --board esp32lilygo_tconnect --debug`
  - `./agent_build_wrapper.sh rover -j 16`
  - `./agent_build_wrapper.sh clean` or `distclean` when needed
- **Serial Port Coordination**: All scripts use lock files via SerialPortLock class. They respond to SIGUSR1 for graceful release.
- **Upload Process**: Always use `python3 esp32_upload_with_lock.py` which handles port locking automatically
- You can, and should prefer, to use the idf serial monitor commands in preference to screen/stty or custom python scripts.
- Do not create duplicate shell or python scripts for functionality that already exists (wholely or in part). Prefer to maintain the existing script and only create a new one after asking the user. 
- If a user tells you not to do something, remember that, and if you've already done it then undo the action e.g. delete the file you created.
- regularly commit to git, and never ever revert code without knowing exactly what is being reverted as working code can be lost if not committed.
- when encouring a stack trace use the IDF tooling to work out what occurred specifically it means before starting further analysis or proposing theories. 
- try and keep the param and hwdef.dat files from all the esp32 boards roughly aligned with changes, especially the t-2can and t-connect board definitions as they are very similar (both esp32s3, just the t-2can as two can interfaces).
- **ALWAYS use agents for builds and monitoring**:
  - Build/upload: Use the `ardupilot-rover-builder` agent (it uses agent_build_wrapper.sh internally)
  - Serial monitoring: Use the `ardupilot-serial-monitor` agent (handles lock files automatically)
  - These agents coordinate via SIGUSR1 signals - no manual intervention needed
- Don't use --verbose on anything unless you really need it as it just burns tokens for no gain
- UART2 is currently the GPS at 4800 baud
- use the ESP or HAL debugging only, and if the ESP ensure that the tag is configured on in the ESP HAL debug code configuration.

## PARAM_EXT Development for DroneCAN Parameter Access (Oct 2025)

### Goal
Implement MAVLink PARAM_EXT protocol (messages 320-324) to enable GCS tools like mavcan-cli to get/set DroneCAN node parameters via MAVLink. This bridges MAVLink Extended Parameter Protocol with existing AP_DroneCAN parameter functions.

### Development Workflow (Option 2: Develop on Branch, Extract to Master)

**Branch Strategy:**
1. Create feature branch from `esp32-build-refactor`: `git checkout -b param-ext-dronecan esp32-build-refactor`
2. Implement and test with ESP32 hardware on this branch
3. Once working, create clean upstream branch: `git checkout -b param-ext-dronecan-upstream origin/master`
4. Cherry-pick PARAM_EXT commits to upstream branch: `git cherry-pick <commit-hash>`
5. Submit upstream branch as PR to ArduPilot
6. Merge feature branch back to esp32-build-refactor: `git merge param-ext-dronecan`

**Critical Rules:**
- **PARAM_EXT commits MUST be completely separate from ESP32-specific changes**
- Each PARAM_EXT commit must compile and work independently
- No ESP32 debug code, no hwdef changes, no defaults.parm changes in PARAM_EXT commits
- Keep commits focused and atomic for clean cherry-picking

**Files Being Modified (PARAM_EXT only):**
- `libraries/GCS_MAVLink/GCS_Param.cpp` - New PARAM_EXT message handlers
- `libraries/GCS_MAVLink/GCS.h` - Function declarations, queues
- `libraries/GCS_MAVLink/GCS_Common.cpp` - Message routing for PARAM_EXT

These files are identical between master and esp32-build-refactor, so cherry-picking will be clean.

**Component ID Mapping (1:1 with DroneCAN Node IDs per MAVLink UAVCAN Spec):**
- **Requirement**: Component ID MUST equal Node ID (1:1 mapping)
- **Official Spec**: "Each unit that is capable of communicating via MAVLink and UAVCAN must use the same number for its MAVLink Component ID and the UAVCAN Node ID"
- **Range**: Supports all DroneCAN nodes 1-127
- **Examples**: Node 1 → Component 1, Node 42 → Component 42, Node 127 → Component 127
- **Component ID Conflicts**: Some node IDs conflict with MAVLink system components (e.g., camera=100-105, servo=140-153), but this is acceptable per spec. You intentionally assign conflicting node IDs only when bridging that specific component type.
- **References**:
  - **Official MAVLink UAVCAN spec**: https://mavlink.io/en/guide/uavcan_interaction.html
  - Implementation: `libraries/GCS_MAVLink/GCS_Param.cpp` (PARAM_EXT handlers)
  - Implementation: `libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp` (UAVCAN messages 310/311)

**MAVLink Messages and Commands:**

*PARAM_EXT Messages (320-324):*
- PARAM_EXT_REQUEST_READ, PARAM_EXT_REQUEST_LIST, PARAM_EXT_VALUE, PARAM_EXT_SET, PARAM_EXT_ACK
- Existing MAVLink messages from common message set

*UAVCAN/DroneCAN Messages (310-311):*
- **UAVCAN_NODE_STATUS (310)** - Periodic node health/status reporting
- **UAVCAN_NODE_INFO (311)** - Node details (name, HW/SW version, UID)
- Enables GCS to discover and monitor DroneCAN nodes

*UAVCAN/DroneCAN Commands:*
- **MAV_CMD_PREFLIGHT_UAVCAN (243)** - Start actuator enumeration for ESCs/servos
- **MAV_CMD_UAVCAN_GET_NODE_INFO (5200)** - Request node info for all online nodes

Note: UAVCAN messages are on esp32-build-refactor branch, not yet on param-ext-dronecan-upstream

**Implementation Components:**
1. PARAM_EXT message handlers (5 new handlers: REQUEST_READ, REQUEST_LIST, VALUE, SET, ACK)
2. Component ID to node ID mapping (1:1 per MAVLink UAVCAN spec: `node_id = component_id`)
3. DroneCAN parameter request queue (async operations with callback handlers)
4. Parameter enumeration state machine (index-based discovery with EMPTY response detection)
5. Protocol bridge (PARAM_EXT messages → AP_DroneCAN get/set parameter functions)
6. Async callback handlers (DroneCAN response → PARAM_EXT_VALUE/ACK replies)

**Implementation Status:**
- Parameter enumeration ported to param-ext-dronecan-upstream branch (Oct 2025)
- Compilation successful on all platforms
- Buffer implementation: Uses `uint8_t buf[MAVLINK_MAX_PACKET_LEN]` (matches ArduPilot convention, MAVLINK_ALIGNED_BUF macro doesn't exist in ArduPilot's MAVLink)
- Enumeration callbacks use static Functor pattern with `(nullptr, function_ptr)` construction
- Tried_types bitmask (3-bit) tracks callback retry attempts for index-based enumeration

**Testing:**
- Test on esp32-build-refactor with actual ESP32 hardware and DroneCAN nodes
- Verify with mavcan-cli tool
- Test parameter get, set, and save operations
- Test parameter enumeration (REQUEST_LIST with empty param_id)
- Verify timeout handling and error cases
- Verify EMPTY response detection (end of parameter list)

**Upstreaming:**
- Cherry-pick only PARAM_EXT commits to param-ext-dronecan-upstream branch
- Verify clean cherry-pick (no conflicts, no ESP32-specific code)
- Test on SITL or non-ESP32 hardware if possible
- Submit PR to ArduPilot master with clear description of functionality