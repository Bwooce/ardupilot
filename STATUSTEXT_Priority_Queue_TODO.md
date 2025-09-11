# STATUSTEXT Tiered Drop Strategy Implementation TODO

## Problem Statement
Currently, ArduPilot's MAVLink STATUSTEXT messages are processed in FIFO order regardless of severity level. This means critical messages (EMERGENCY, ALERT, CRITICAL) can be queued behind less important messages (INFO, DEBUG) during periods of high message volume or limited bandwidth.

## Current Implementation Issues
- All STATUSTEXT messages treated equally in `GCS::StatusTextQueue`
- Simple FIFO `ObjectArray<statustext_t>` processing in `service_statustext()`  
- No consideration of MAV_SEVERITY levels during flow control
- Critical system alerts may be delayed behind routine telemetry messages

## Proposed Solution
Implement a tiered message dropping strategy that preserves time-ordering while ensuring critical messages get through during buffer congestion:

### Service Tiers (Drop Strategy):

#### **Tier 1: Critical (Always Preserved)**
- **EMERGENCY** (0) - System unusable, panic condition
- **ALERT** (1) - Action should be taken immediately  
- **CRITICAL** (2) - Action must be taken immediately

#### **Tier 2: Important (Dropped during moderate congestion)**
- **ERROR** (3) - Secondary system errors
- **WARNING** (4) - Future error warnings
- **NOTICE** (5) - Unusual events needing investigation

#### **Tier 3: Routine (Dropped during any congestion)**
- **INFO** (6) - Normal operational messages
- **DEBUG** (7) - Non-operational debugging messages (dropped first)

### Drop Strategy Logic
1. **Normal Operation**: Process all messages in FIFO order (preserves time ordering)
2. **Light Congestion**: Drop DEBUG messages first, log drop count
3. **Moderate Congestion**: Drop all Tier 3 messages, log drop count  
4. **Heavy Congestion**: Drop Tier 2 and Tier 3 messages, only send Tier 1 critical messages

### Implementation Tasks

#### 1. Modify StatusTextQueue Processing
- **File**: `libraries/GCS_MAVLink/GCS_Common.cpp:2681`
- **Current**: `if (txspace() < payload_size) { break; }`
- **New**: Implement tiered drop logic based on available space and message severity
- **Preserve**: FIFO processing order within each tier

#### 2. Add Drop Counting and Logging
- **Track**: Number of messages dropped by severity level
- **Log**: Periodic drop statistics (e.g., "Dropped 45 DEBUG, 12 INFO messages due to congestion")
- **Reset**: Drop counters after logging to prevent overflow

#### 3. Implement Tier-based Flow Control
```cpp
enum class DropTier {
    NONE = 0,        // Normal - process all messages
    DEBUG_ONLY = 1,  // Light congestion - drop DEBUG only
    ROUTINE = 2,     // Moderate congestion - drop INFO+DEBUG  
    NON_CRITICAL = 3 // Heavy congestion - drop everything except EMERGENCY/ALERT/CRITICAL
};
```

#### 4. Configuration Parameters
- **Parameter**: `GCS_DROP_TIER` - Set drop threshold (0=disabled, 1-3=tier levels)
- **Thresholds**: Configurable txspace() percentages for each tier activation
- **Defaults**: Conservative settings that maintain current behavior unless congestion detected

#### 5. Update service_statustext() Algorithm
```cpp
// Pseudo-code for new logic:
DropTier current_tier = calculate_drop_tier(txspace(), payload_size);
for (each message in queue) {
    if (should_drop_message(message.severity, current_tier)) {
        increment_drop_counter(message.severity);
        remove_message_from_queue();
        continue;
    }
    // Send message normally
    send_statustext_message();
}
log_drop_statistics_if_needed();
```

## Benefits
- **Preserves time ordering** of messages within each tier
- **Guarantees delivery** of critical safety messages
- **Provides visibility** into message dropping through logging
- **Configurable behavior** - can be disabled or tuned per application
- **Graceful degradation** under increasing congestion levels
- **No complex priority queue** - simple FIFO with selective dropping

## Implementation Considerations
- **Zero overhead when uncongested**: Normal FIFO processing continues
- **Minimal CPU impact**: Simple severity level comparisons during congestion
- **Thread safety**: Maintain existing semaphore protection
- **Drop statistics**: Prevent integer overflow in drop counters
- **Hysteresis**: Avoid rapid tier switching due to momentary congestion spikes

## Files to Modify
- `libraries/GCS_MAVLink/GCS_Common.cpp` - service_statustext() function (main logic)
- `libraries/GCS_MAVLink/GCS.h` - Add drop statistics tracking to StatusTextQueue
- Parameter system for GCS_DROP_TIER configuration

## Success Criteria
1. **Critical messages always delivered** during any level of congestion
2. **Time ordering preserved** within severity tiers (no message reordering)
3. **Zero performance impact** during normal (non-congested) operation
4. **Drop statistics logging** provides visibility into message loss
5. **Backward compatibility** - existing behavior when feature disabled
6. **Tunable thresholds** allow per-vehicle optimization

## Example Drop Statistics Log Messages
```
STATUSTEXT: Dropped 127 DEBUG messages due to congestion
STATUSTEXT: Dropped 45 DEBUG, 12 INFO messages due to heavy congestion  
STATUSTEXT: Congestion cleared, resumed normal message processing
```