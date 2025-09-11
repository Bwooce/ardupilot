# GCS Changes Required for ESP32 MAVLink Firmware Updates

## **Executive Summary**

**Good News**: **Minimal GCS changes required** due to leveraging existing MAVLink infrastructure.

**Required Changes**: ~200-400 lines of code per GCS
**Optional Changes**: Enhanced UI for better user experience

---

## **Why GCS Changes Are Needed**

### **1. New MAVLink Messages**
The ESP32 firmware update system requires 4 new MAVLink messages that current GCS don't understand:
- `FIRMWARE_UPDATE_REQUEST`
- `FIRMWARE_UPDATE_STATUS`  
- `FIRMWARE_UPDATE_CHUNK`
- `FIRMWARE_UPDATE_COMPLETE`

### **2. Chunked Binary Transfer**
Unlike parameter uploads (small text), firmware requires:
- **Large binary file handling** (1.7MB vs ~50KB parameters)
- **Chunked streaming protocol** (1KB chunks over potentially unreliable links)
- **Progress tracking and error recovery**

### **3. Safety & User Experience**
- **Pre-flight safety checks** (battery level, connection quality)
- **User confirmation dialogs** (firmware version, compatibility warnings)
- **Progress monitoring** (transfer progress, flash progress)
- **Error handling** (timeout recovery, corruption detection)

---

## **Required Changes by GCS**

## **QGroundControl Changes**

### **Minimal Required Changes** (~200-300 LOC)

**1. MAVLink Message Support** (~50-100 LOC)
```cpp
// Add to src/MAVLink/MAVLinkProtocol.cc
void MAVLinkProtocol::_handleFirmwareUpdateStatus(LinkInterface* link, mavlink_message_t& message) {
    mavlink_firmware_update_status_t status_msg;
    mavlink_msg_firmware_update_status_decode(&message, &status_msg);
    emit firmwareUpdateStatus(status_msg.progress, status_msg.status);
}

// Add message routing in existing message handler switch statement
case MAVLINK_MSG_ID_FIRMWARE_UPDATE_STATUS:
    _handleFirmwareUpdateStatus(link, message);
    break;
```

**2. Basic Upload Interface** (~150-200 LOC)
```cpp
// Add to src/FirmwarePlugin/GenericFirmwarePlugin.cc
class ESP32FirmwareUpdater : public QObject {
    Q_OBJECT
public:
    void startFirmwareUpdate(const QString& firmwarePath);
    void sendNextChunk();
    void handleStatusMessage(uint8_t progress, uint8_t status);
    
private:
    QFile _firmwareFile;
    uint32_t _totalSize;
    uint32_t _bytesSent;
    QTimer _chunkTimer;
    uint32_t _expectedCRC;
};
```

### **Enhanced User Experience** (+200-300 LOC)

**3. UI Integration** (~100-150 LOC)
```qml
// Add to src/ui/preferences/FirmwareUpgrade.qml
Dialog {
    title: "ESP32 Firmware Update"
    
    FileDialog {
        id: firmwareFileDialog
        nameFilters: ["Firmware files (*.bin)", "All files (*)"]
        onAccepted: firmwareUpdater.startFirmwareUpdate(fileUrl)
    }
    
    ProgressBar {
        value: firmwareUpdater.progress
        text: firmwareUpdater.statusText
    }
}
```

**4. Safety & Validation** (~100-150 LOC)
```cpp
// Pre-upload validation
bool ESP32FirmwareUpdater::validateFirmware(const QString& path) {
    // Check file size (< 1.7MB for ESP32 partition)
    // Validate firmware header/magic bytes
    // Check compatibility with target board
    // Verify CRC integrity
    return true;
}

bool ESP32FirmwareUpdater::checkSafetyConditions() {
    // Verify good radio link quality (RSSI > -80dBm)
    // Check battery level (> 30%)
    // Ensure vehicle is disarmed
    // Confirm stable connection
    return true;
}
```

## **Mission Planner Changes**

### **Minimal Required Changes** (~200-300 LOC)

**1. MAVLink Handler** (~100-150 LOC)
```csharp
// Add to MAVLink.cs message handlers
public void handleFirmwareUpdateStatus(MAVLinkMessage msg) {
    var status = msg.ToStructure<mavlink_firmware_update_status_t>();
    BeginInvoke((Action)delegate {
        updateProgressBar(status.progress);
        updateStatusText(getStatusString(status.status));
    });
}
```

**2. Firmware Upload Dialog** (~100-150 LOC)
```csharp
// Add new form: FirmwareUpdateDialog.cs
public partial class FirmwareUpdateDialog : Form {
    private BackgroundWorker firmwareWorker;
    private FileInfo firmwareFile;
    
    private void btnSelectFirmware_Click(object sender, EventArgs e) {
        // File selection dialog
        // Firmware validation
        // Start upload process
    }
    
    private void sendFirmwareChunk(int chunkIndex) {
        // Read chunk from file
        // Send FIRMWARE_UPDATE_CHUNK message
        // Update progress
    }
}
```

## **PyMavlink Changes (Existing Foundation)**

**Already Exists**: Basic MAVLink file transfer capabilities in `mavutil.py`
```python
# Existing reboot capability
def reboot_autopilot(self, hold_in_bootloader=False, force=False):
    '''reboot the autopilot'''
    # Already implemented for bootloader mode
```

**Required Addition** (~100-150 LOC):
```python
# Add to mavutil.py
def upload_firmware_esp32(self, firmware_path, progress_callback=None):
    '''Upload firmware to ESP32 via MAVLink'''
    with open(firmware_path, 'rb') as f:
        firmware_data = f.read()
    
    # Send FIRMWARE_UPDATE_REQUEST
    self.mav.firmware_update_request_send(
        len(firmware_data),
        self.calculate_crc32(firmware_data),
        1024,  # chunk size
        self.target_system,
        self.target_component,
        "ESP32_Firmware_v1.0".encode('ascii')
    )
    
    # Stream chunks
    for chunk_id in range(0, len(firmware_data), 1024):
        chunk = firmware_data[chunk_id:chunk_id+1024]
        self.mav.firmware_update_chunk_send(...)
        if progress_callback:
            progress_callback(chunk_id / len(firmware_data))
```

---

## **Changes NOT Required**

### **✅ Existing Infrastructure Can Be Leveraged**

**1. File Transfer Framework**
- QGC already handles large file uploads (mission files, parameters)
- Mission Planner has firmware upload UI (for PX4/ArduPilot .px4/.apj files)
- PyMavlink has binary data streaming capabilities

**2. MAVLink Message System**
- Message parsing/routing already exists
- Binary data handling already implemented
- Progress reporting patterns already established

**3. Connection Management**
- Link quality monitoring exists
- Timeout handling already implemented  
- Error recovery mechanisms already available

### **✅ No Protocol Changes**
- Uses standard MAVLink v2 format
- No new connection types required
- No new authentication mechanisms
- Leverages existing TARGET_SYSTEM/TARGET_COMPONENT addressing

---

## **Implementation Strategy**

### **Phase 1: Proof of Concept** (~100-200 LOC per GCS)
Focus on **PyMavlink first**:
```python
# Simple command-line tool
python mavproxy.py --master=/dev/ttyUSB0 --cmd="upload_firmware_esp32 rover_firmware.bin"
```

**Why PyMavlink First?**
- ✅ Fastest to implement (~100 LOC)
- ✅ Great for testing and validation
- ✅ Can be used by developers immediately
- ✅ Foundation for other GCS implementations

### **Phase 2: QGroundControl Integration** (~200-400 LOC)
Add to existing firmware upgrade section:
- Detect ESP32 boards
- Add "Upload Firmware" button
- Progress dialog with safety checks

### **Phase 3: Mission Planner Integration** (~200-400 LOC)
Add to Initial Setup → Install Firmware:
- ESP32 board detection
- Custom firmware upload option
- Integration with existing firmware management

---

## **User Experience Flow**

### **QGroundControl Example**
```
1. User connects to ESP32 rover
2. QGC detects "ESP32 TConnect" board
3. Vehicle Setup → Firmware → [Upload Custom Firmware] button appears
4. User selects .bin file from build output
5. QGC validates file size, CRC, shows confirmation dialog
6. Upload progress bar: "Uploading firmware... 45% (30 seconds remaining)"
7. Flash progress: "Writing to flash... 80%"  
8. "Update complete! Rebooting..."
9. QGC reconnects, shows new firmware version in heartbeat
```

### **Mission Planner Example**
```
1. Initial Setup → Install Firmware
2. Board detection shows "ESP32 TConnect"
3. [Upload Custom Firmware] button next to standard options
4. File selection → firmware validation → safety checks
5. Progress tracking with abort capability
6. Automatic reconnection after reboot
```

---

## **Why These Changes Are Minimal**

### **1. Leverage Existing Patterns**
```cpp
// QGC already handles similar operations:
- Parameter file upload/download (large text files)
- Mission file upload/download (binary waypoint data)  
- Log file download (large binary streaming)
- Firmware flashing (for standard PX4/ArduPilot boards)
```

### **2. Standard MAVLink Architecture**
```cpp
// No custom protocols needed:
- Uses existing MAVLink v2 message format
- Follows standard request/response patterns
- Leverages existing error handling
- Uses proven chunked transfer approach
```

### **3. Proven UI Patterns**
```cpp
// Familiar user workflows:
- File selection dialogs (already exist)
- Progress bars with cancel (already exist)
- Safety confirmation dialogs (already exist)  
- Firmware management UI (already exists for other boards)
```

---

## **Risk Assessment**

### **Low Risk Changes** ✅
- **MAVLink message handlers**: Standard pattern, low complexity
- **File upload logic**: Similar to existing firmware upload
- **Progress reporting**: Copy existing mission upload patterns

### **Medium Risk Changes** ⚪
- **Safety validation**: Needs ESP32-specific checks
- **Error recovery**: Must handle radio link interruptions
- **User experience**: Need clear warnings about update risks

### **No High-Risk Changes** 
- Uses proven MAVLink infrastructure
- Follows established GCS patterns
- No custom network protocols
- No authentication/security complexity

---

## **Bottom Line**

**Required GCS Changes: ~200-400 LOC per GCS**

**Why So Small?**
1. **Leverages existing MAVLink infrastructure** (message parsing, routing, error handling)
2. **Follows proven patterns** (file uploads, progress reporting, firmware management)  
3. **Minimal new UI required** (add to existing firmware sections)
4. **Standard binary transfer** (similar to mission/parameter uploads)

**Development Effort**: ~1-2 weeks per GCS (after ArduPilot implementation complete)

**User Benefit**: Transformative capability with minimal development cost

The key insight is that GCS already have 95% of the required infrastructure - we're just adding 4 new MAVLink messages and integrating them into existing firmware management workflows.