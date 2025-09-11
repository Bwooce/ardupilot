# DroneCAN FOTA (Firmware Over-The-Air) Implementation Plan

## Current Status: Partial Support

### ✅ What's Currently Implemented

#### 1. Node Discovery & Info Collection
- **DNA Server**: Automatically assigns node IDs and discovers new nodes
- **GetNodeInfo Requests**: ArduPilot automatically requests node information when nodes join
- **Node Information Processing**: 
  - Extracts hardware version (`hardware_version.major`, `hardware_version.minor`, `hardware_version.unique_id`)
  - Extracts firmware version (`software_version.major`, `software_version.minor`, `software_version.vcs_commit`)
  - Logs node information to dataflash logs (CAND message)
  - Sends node info via MAVLink to ground control stations
  - Displays human-readable text: "DroneCAN Node X: NodeName vX.Y online"

#### 2. Target Node Support (AP_Periph)
- **BeginFirmwareUpdate Handler**: AP_Periph can receive `uavcan.protocol.file.BeginFirmwareUpdate` requests
- **Bootloader Integration**: Can jump into bootloader mode for firmware updates
- **Update Response**: Sends appropriate response messages

### ❌ What's Missing (Not Implemented)

#### 1. Automatic Firmware Update Initiation
- **No automatic version comparison**: Flight controller doesn't check if newer firmware is available
- **No BeginFirmwareUpdate sending**: ArduPilot doesn't initiate firmware updates automatically
- **No firmware repository**: No system for tracking available firmware versions

#### 2. Firmware File Management
- **No file storage**: No mechanism to store firmware files on flight controller
- **No firmware transfer**: No implementation of `uavcan.protocol.file` services
- **No firmware validation**: No verification of firmware authenticity or compatibility

#### 3. Update Management
- **No update policies**: No configuration for when/how to update nodes
- **No rollback mechanism**: No way to revert failed updates
- **No progress tracking**: No monitoring of update status

## Current Workflow

```
1. Node Joins Network
   └── DNA server assigns node ID
   └── DNA server requests GetNodeInfo
   
2. Flight Controller Processing (IMPLEMENTED)
   └── Receives GetNodeInfoResponse
   └── Logs node information (CAND log)
   └── Sends node info to GCS via MAVLink
   └── Displays "DroneCAN Node X: NodeName vX.Y online"
   
3. Manual Update Process (CURRENT REALITY)
   └── User/GCS must manually initiate updates
   └── External tools handle firmware transfer
   └── Manual BeginFirmwareUpdate message sending
```

## Implementation Plan

### Phase 1: Firmware Repository & Management

#### 1.1 Firmware Storage System
- **Location**: Create `libraries/AP_DroneCAN_FOTA/` directory
- **Storage**: SD card-based firmware repository (`/FIRMWARE/` directory)
- **File Structure**:
  ```
  /FIRMWARE/
  ├── manifest.json          # Available firmware versions
  ├── ap_periph_v1.2.3.bin   # Firmware files
  ├── some_node_v2.1.0.bin
  └── checksums.txt          # File integrity verification
  ```

#### 1.2 Firmware Manifest System
- **Manifest Format** (JSON):
  ```json
  {
    "repository_version": "1.0",
    "firmware_list": [
      {
        "node_type": "ap_periph",
        "hardware_id": "0x0001",
        "version": "1.2.3",
        "filename": "ap_periph_v1.2.3.bin",
        "checksum": "sha256:abc123...",
        "min_hardware_version": "1.0",
        "release_date": "2024-01-15"
      }
    ]
  }
  ```

#### 1.3 Version Comparison Logic
- **Semantic versioning**: Compare major.minor.patch versions
- **Hardware compatibility**: Check hardware version requirements
- **Downgrade protection**: Optional prevention of firmware downgrades

### Phase 2: Update Policy & Configuration

#### 2.1 Parameters
- `CAN_FOTA_ENABLE`: Enable/disable automatic FOTA (default: 0)
- `CAN_FOTA_CHECK_INTERVAL`: How often to check for updates (default: 86400 seconds)
- `CAN_FOTA_AUTO_UPDATE`: Automatically update or just notify (default: 0)
- `CAN_FOTA_ALLOWED_NODES`: Bitmask of node types allowed to update
- `CAN_FOTA_MAX_CONCURRENT`: Maximum simultaneous updates (default: 1)

#### 2.2 Update Policies
- **Safety checks**: Don't update during flight, armed state, or critical operations
- **Timing**: Configurable update windows (e.g., only during maintenance periods)
- **Prioritization**: Critical security updates vs. feature updates
- **Node criticality**: Different policies for essential vs. non-essential nodes

### Phase 3: File Transfer Implementation

#### 3.1 UAVCAN File Protocol
- **Implement services**:
  - `uavcan.protocol.file.GetInfo`
  - `uavcan.protocol.file.Read`
  - `uavcan.protocol.file.Write` (if needed for firmware upload to FC)
- **Transfer management**: Handle chunked file transfers with error recovery
- **Bandwidth management**: Rate limiting to avoid saturating CAN bus

#### 3.2 Firmware Transfer Process
```
1. BeginFirmwareUpdate Request
   └── Include firmware file info (size, checksum)
   
2. Node Response
   └── Accept/reject with reason
   
3. File Transfer Loop
   └── Read chunks via uavcan.protocol.file.Read
   └── Node validates each chunk
   └── Progress reporting
   
4. Transfer Completion
   └── Final checksum verification
   └── Node jumps to bootloader
```

### Phase 4: Core FOTA Logic

#### 4.1 Update Orchestrator Class
```cpp
class AP_DroneCAN_FOTA {
public:
    // Core functionality
    void init();
    void update();  // Called from main loop
    
    // Firmware management
    bool load_firmware_manifest();
    bool check_node_for_updates(uint8_t node_id, const GetNodeInfoResponse& info);
    
    // Update execution
    bool initiate_node_update(uint8_t node_id, const FirmwareInfo& firmware);
    void monitor_update_progress();
    
    // Configuration
    void set_update_policy(UpdatePolicy policy);
    bool is_update_allowed();
};
```

#### 4.2 Integration Points
- **DNS Server Enhancement**: Add FOTA hooks to `handleNodeInfo()`
- **Scheduler Integration**: Periodic update checks
- **MAVLink Integration**: Status reporting and manual control
- **Logging**: Comprehensive update attempt logging

### Phase 5: Safety & Reliability

#### 5.1 Safety Mechanisms
- **Pre-flight checks**: Verify all critical nodes are operational and updated
- **Rollback capability**: Mechanism to revert failed updates
- **Update verification**: Post-update health checks
- **Emergency stop**: Ability to abort updates in critical situations

#### 5.2 Error Handling
- **Transfer failures**: Retry logic with exponential backoff
- **Node unresponsive**: Timeout handling and recovery
- **Corrupted firmware**: Checksum validation and rejection
- **Bootloader failures**: Detection and reporting

#### 5.3 Logging & Diagnostics
- **Update attempts**: Success/failure with detailed reasons
- **Transfer statistics**: Speed, retry counts, error rates
- **Node status tracking**: Pre/post update health
- **MAVLink status messages**: Real-time progress reporting

### Phase 6: User Interface & Control

#### 6.1 MAVLink Integration
- **New MAVLink messages**:
  - `DRONECAN_FOTA_STATUS`: Current update status
  - `DRONECAN_FOTA_PROGRESS`: Transfer progress
  - `DRONECAN_FOTA_NODE_INFO`: Node firmware information
- **MAVLink commands**:
  - `MAV_CMD_DRONECAN_FOTA_CHECK`: Manual update check
  - `MAV_CMD_DRONECAN_FOTA_UPDATE`: Force update specific node

#### 6.2 Ground Control Station Support
- **Mission Planner integration**: UI for firmware management
- **QGroundControl support**: Update status display
- **Web interface**: For ESP32-based flight controllers

### Phase 7: Testing & Validation

#### 7.1 Test Framework
- **SITL testing**: Simulated DroneCAN network with virtual nodes
- **Hardware-in-loop**: Real AP_Periph nodes with test firmware
- **Stress testing**: Multiple concurrent updates, network congestion
- **Failure injection**: Network errors, power loss, corrupted transfers

#### 7.2 Validation Criteria
- **Update success rate**: >99% under normal conditions
- **Transfer efficiency**: Minimal CAN bus impact
- **Safety compliance**: No updates during unsafe conditions
- **Recovery capability**: 100% rollback success rate

## Implementation Priority

### High Priority (Phase 1-2)
1. Basic firmware repository and manifest system
2. Configuration parameters and safety policies
3. Integration with existing DNA server

### Medium Priority (Phase 3-4)
1. File transfer protocol implementation
2. Core FOTA orchestrator
3. Basic update execution

### Lower Priority (Phase 5-6)
1. Advanced safety mechanisms
2. User interface enhancements
3. Ground control station integration

## File Structure

```
libraries/AP_DroneCAN_FOTA/
├── AP_DroneCAN_FOTA.h              # Main class definition
├── AP_DroneCAN_FOTA.cpp            # Core implementation
├── AP_DroneCAN_FOTA_Repository.h   # Firmware repository management
├── AP_DroneCAN_FOTA_Repository.cpp
├── AP_DroneCAN_FOTA_Transfer.h     # File transfer protocol
├── AP_DroneCAN_FOTA_Transfer.cpp
├── AP_DroneCAN_FOTA_Policy.h       # Update policies and safety
├── AP_DroneCAN_FOTA_Policy.cpp
└── examples/
    └── FOTA_test/                   # Test application
```

## Dependencies

- **Existing**: AP_DroneCAN, AP_DroneCAN_DNA_Server, AP_Filesystem
- **New**: JSON parsing library (or custom minimal parser)
- **Optional**: AP_Scripting integration for custom update policies

## Compatibility

- **ArduPilot**: All platforms with DroneCAN support
- **Target Nodes**: Requires bootloader support (already in AP_Periph)
- **Hardware**: SD card or sufficient flash storage for firmware repository
- **CAN Bus**: Additional bandwidth requirements during updates

## Risk Assessment

### High Risk
- **Bricking nodes**: Corrupted firmware transfers
- **Network congestion**: Large transfers affecting flight-critical communication
- **Security**: Malicious firmware injection

### Mitigation
- **Checksum validation**: Multiple levels of integrity checking
- **Bandwidth management**: Rate limiting and priority scheduling
- **Access control**: Signed firmware and authentication (future enhancement)

## Future Enhancements

1. **Firmware signing**: Cryptographic verification of firmware authenticity
2. **Delta updates**: Transfer only changed portions of firmware
3. **Mesh updates**: Node-to-node firmware sharing
4. **Cloud integration**: Automatic firmware download from ArduPilot servers
5. **A/B partitioning**: Dual firmware slots for safer updates