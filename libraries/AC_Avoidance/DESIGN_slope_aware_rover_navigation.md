# Design: Slope-Aware Path Planning for ArduPilot Rover

## Problem Statement

ArduPilot Rover has no awareness of terrain slope. In AUTO, Guided, or RTL modes, the
path planner will happily route a rover up a cliff face or down an embankment. There is
no speed reduction on grades, no rejection of impassable slopes, and no preference for
flatter routes when alternatives exist.

This matters for:
- **Desert/off-road rovers** operating in terrain with gullies, ridges, and washouts
- **Agricultural rovers** on hillside farms
- **Survey rovers** that need to maintain consistent ground contact for sensor accuracy
- **Any rover** where rollover or loss of traction on steep grades is a risk

## Existing Infrastructure

All the building blocks exist in ArduPilot today:

| Component | Status | What it provides |
|-----------|--------|-----------------|
| AP_Terrain | Available (disabled on ESP32, easy to enable) | Elevation at any GPS coordinate via `height_amsl(Location, &height)` |
| AP_OABendyRuler | Available | Evaluates candidate headings, rejects unsafe ones |
| AP_OAPathPlanner | Available | Orchestrates avoidance algorithms in background thread |
| AR_WPNav_OA | Available | Rover waypoint nav with OA integration |
| AP_Proximity | Available | Abstraction for lidar/radar obstacle detection |
| OA Database | Available | Stores detected obstacles with position, radius, expiry |
| Location::offset_bearing() | Available | Projects points along candidate paths |

**What's missing:**
- A slope margin check in BendyRuler
- Slope-based speed scaling in the Rover throttle controller
- Vehicle footprint awareness in obstacle margin calculations
- Integration of proximity sensor data with terrain slope for combined decision-making

## Proposed Design

### Four independent features that complement each other:

**Feature 1: Slope-aware path selection** (in BendyRuler)
- Reject candidate headings where terrain slope exceeds a maximum
- Prefer flatter routes when multiple options exist

**Feature 2: Speed scaling on grades** (in Rover throttle control)
- Reduce speed proportional to terrain slope
- Prevents loss of traction and improves stability

**Feature 3: Vehicle footprint awareness** (in BendyRuler + AR_WPNav)
- Account for vehicle length, width, and turning radius in obstacle margins
- Critical for vehicles with sensor booms or trailers that extend the footprint

**Feature 4: Proximity sensor integration with slope planning** (in BendyRuler)
- Combine radar/lidar obstacle detection with terrain slope data
- Unified margin calculation considers both obstacles and terrain

These are independent -- a rover can use any combination.

---

## Feature 1: Slope-Aware BendyRuler

### Algorithm

BendyRuler already evaluates candidate bearings in 5-degree increments, checking each
for proximity obstacles and fence violations via `calc_avoidance_margin()`. We add a new
check: `calc_margin_from_terrain_slope()`.

For each candidate bearing at a given lookahead distance:

```
1. Sample terrain elevation at N points along the path
   (current position, 1/3 lookahead, 2/3 lookahead, full lookahead)

2. Calculate slope between consecutive sample points:
   slope_pct = 100 * abs(elevation_delta) / horizontal_distance

3. If max slope along path > OA_BR_SLOPE_MAX:
   Return margin = -1 (path rejected, same as hitting a fence)

4. If slope is within limits:
   Return margin proportional to how far below the limit:
   margin = (OA_BR_SLOPE_MAX - max_slope) / OA_BR_SLOPE_MAX * base_margin
```

This integrates naturally -- BendyRuler already picks the heading with the best margin.
Flat terrain gets high margins, steep terrain gets low margins, impassable terrain gets
rejected. The existing bearing-change resistance (`resist_bearing_change()`) prevents
oscillation between flat and steep routes.

### Integration Point

In `AP_OABendyRuler::calc_avoidance_margin()` (AP_OABendyRuler.cpp, around line 420),
add after the existing fence/proximity checks:

```cpp
#if AP_TERRAIN_AVAILABLE
    if (_slope_max > 0) {
        float slope_margin = calc_margin_from_terrain_slope(start, end);
        if (slope_margin < margin) {
            margin = slope_margin;
        }
    }
#endif
```

### New Method

```cpp
float AP_OABendyRuler::calc_margin_from_terrain_slope(
    const Location &start, const Location &end)
{
    AP_Terrain *terrain = AP_Terrain::get_singleton();
    if (terrain == nullptr || !terrain->enabled()) {
        return FLT_MAX;  // no terrain data, don't constrain
    }

    const float horiz_dist = start.get_distance(end);
    if (horiz_dist < 1.0f) {
        return FLT_MAX;
    }

    // sample terrain at 4 points along the path
    const uint8_t num_samples = 4;
    float heights[num_samples];
    float max_slope_pct = 0;

    for (uint8_t i = 0; i < num_samples; i++) {
        Location sample = start;
        float fraction = (float)i / (num_samples - 1);
        sample.offset_bearing(start.get_bearing_to(end) * 0.01f,
                             horiz_dist * fraction);
        if (!terrain->height_amsl(sample, heights[i])) {
            return FLT_MAX;  // terrain data unavailable, don't constrain
        }
    }

    // calculate maximum slope between consecutive samples
    float segment_dist = horiz_dist / (num_samples - 1);
    for (uint8_t i = 1; i < num_samples; i++) {
        float slope_pct = 100.0f * fabsf(heights[i] - heights[i-1]) / segment_dist;
        max_slope_pct = MAX(max_slope_pct, slope_pct);
    }

    if (max_slope_pct > _slope_max) {
        return -1.0f;  // path rejected
    }

    // return margin proportional to slope headroom
    // flatter paths get higher margins and are preferred
    return (_slope_max - max_slope_pct) / _slope_max * _margin_max;
}
```

### Terrain Data Availability

`height_amsl()` returns false when terrain data is not cached. This can happen when:
- Terrain data hasn't been downloaded from GCS yet
- SD card not present
- Cache miss for the queried location

**Policy: fail-open.** When terrain data is unavailable, the slope check returns
`FLT_MAX` (no constraint). The rover proceeds without slope awareness rather than
refusing to move. This matches how BendyRuler handles missing proximity data.

The terrain cache handles ~12 blocks by default (coverage ~3.1km x 2.7km per block).
For a rover moving at 2-5 m/s with 15m lookahead, the cache will almost certainly have
data for all sample points since they're near the current position. GCS pre-loading
(Mission Planner terrain download) ensures coverage for planned mission areas.

### Terrain Query Performance

`height_amsl()` costs ~20us per call (cache hit with bilinear interpolation). With 4
sample points per candidate bearing and up to 68 candidate bearings (340 degrees / 5
degree increments), worst case is 68 * 4 = 272 queries = ~5.4ms.

**Mitigation options if this is too expensive:**
1. Only check slope for bearings that pass proximity/fence checks (most are already
   rejected, typically only 5-15 bearings remain)
2. Reduce to 2-3 sample points (start + end + midpoint)
3. Cache terrain heights in a local grid around the vehicle (amortise across bearings)

For Rover at 10Hz navigation rate, 5ms is 5% of the cycle budget -- acceptable but
worth monitoring. BendyRuler already runs in the OAPathPlanner background thread, so
it doesn't block the main loop.

---

## Feature 2: Speed Scaling on Grades

### Algorithm

In `Mode::calc_throttle()` or `AR_WPNav::update()`, query terrain slope along the
current heading and scale the target speed:

```
1. Get terrain height at current position and at a point 5-10m ahead
2. Calculate grade: slope_pct = 100 * elevation_delta / horizontal_distance
3. Scale speed: speed_factor = 1.0 - (slope_pct / SLOPE_SPEED_SCALE_PCT)
4. Clamp speed_factor to [SLOPE_SPEED_MIN, 1.0]
5. Apply: target_speed *= speed_factor
```

### Integration Point

In `AR_WPNav::advance_wp_frac_and_target_speed()` or `Mode::calc_throttle()`,
before the speed is sent to the motor controller:

```cpp
#if AP_TERRAIN_AVAILABLE
float speed = get_speed();
float slope_factor = calc_terrain_speed_factor();
speed *= slope_factor;
set_speed(speed);
#endif
```

### Separate from BendyRuler

Speed scaling works independently of path planning:
- Even on a chosen path, uphill segments should slow down
- Downhill segments should also slow down (braking/stability)
- Works in all modes (AUTO, Guided, RTL, SmartRTL), not just OA modes

---

## Feature 3: Vehicle Footprint Awareness

### Problem

ArduPilot Rover treats the vehicle as a **point mass**. BendyRuler calculates obstacle
margins as `distance_to_obstacle - obstacle_radius`, with no consideration of the
vehicle's own dimensions. The only dimension parameter today is `TURN_RADIUS` (default
0.9m), used by the steering controller for low-speed turn rate limits.

This is dangerous for:
- Vehicles with sensor booms (e.g., a magnetometer boom extending 1-2m behind the rover)
- Wide vehicles that can't fit through gaps the point-mass model considers clear
- Vehicles towing equipment or carrying wide payloads
- Any vehicle where the turning circle is much larger than the chassis

### Existing Turning Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| TURN_RADIUS | 0.9m | Steering controller turn rate at low speed |
| ATC_TURN_MAX_G | 0.6G | Maximum lateral acceleration |
| WP_RADIUS | 2.0m | Distance to consider waypoint reached |

None of these affect obstacle margin calculations.

### Proposed Solution

Add vehicle dimension parameters and incorporate them into margin calculations:

```cpp
// In Rover ParametersG2 (or AP_OABendyRuler for cross-vehicle use)
// @Param: OA_VEH_LENGTH
// @DisplayName: Vehicle length
// @Description: Total vehicle length in meters including any booms or trailers.
//   Used to inflate obstacle margins so the entire vehicle clears obstacles.
//   Set to 0 to treat vehicle as a point (legacy behaviour).
// @Range: 0 10
// @Units: m
AP_GROUPINFO("VEH_LENGTH", xx, ..., _veh_length, 0),

// @Param: OA_VEH_WIDTH
// @DisplayName: Vehicle width
// @Description: Total vehicle width in meters. Used to inflate obstacle margins.
// @Range: 0 5
// @Units: m
AP_GROUPINFO("VEH_WIDTH", xx, ..., _veh_width, 0),
```

### Margin Adjustment

In `calc_avoidance_margin()`, after computing the raw margin from each source, subtract
the vehicle's half-dimensions:

```cpp
// adjust margin for vehicle footprint
// use the larger of half-length and half-width as a conservative buffer
if (_veh_length > 0 || _veh_width > 0) {
    float veh_radius = MAX(_veh_length, _veh_width) * 0.5f;
    margin -= veh_radius;
}
```

This is conservative -- it uses the larger dimension as a circular buffer. A more
sophisticated approach would consider the heading-relative projection of the vehicle
rectangle, but the circular approximation is simpler, safer, and matches how obstacles
are modelled (as circles with radius).

### Turning Radius Integration

The existing `TURN_RADIUS` parameter should also inform path planning. When BendyRuler
evaluates a bearing change, the vehicle can't turn instantaneously -- it sweeps an arc.
The swept path of a long vehicle on a tight turn extends well beyond the point-mass path.

For a vehicle of length L turning with radius R, the outer rear corner sweeps:
```
swept_width = sqrt(R^2 + L^2) - R
```

This should inflate the margin requirement when evaluating bearings that require
significant heading changes. Implementation: in `search_xy_path()`, when a candidate
bearing differs from the current heading by more than ~10 degrees, add the swept width
to the required margin.

---

## Feature 4: Proximity Sensor Integration with Slope Planning

### Current Proximity Architecture

BendyRuler already integrates proximity sensors (lidar, radar) via the OA Database.
The data flow is:

```
Sensor Hardware (RPLidar, SF45B, MR72 radar, etc.)
    -> AP_Proximity_Backend (sensor driver)
    -> AP_OADatabase (object tracking with position, radius, expiry)
    -> AP_OABendyRuler::calc_margin_from_object_database()
```

**Supported sensors relevant to Rover:**

| Type | Sensor | Interface | Range | Notes |
|------|--------|-----------|-------|-------|
| 2D Lidar | RPLidar A2 | UART | 12m | 360-degree scanning |
| 2D Lidar | LightWare SF45B | UART | 50m | 320-degree scanning |
| 2D Lidar | LD06 | UART | 12m | Low cost, 360-degree |
| Radar | MR72 | DroneCAN | 50m+ | All-weather, long range |
| Radar | Hexsoon | DroneCAN | 20m | Compact |
| Rangefinder array | Various | Multiple | Varies | Forward-facing |

**OA Database parameters:**

| Parameter | Default | Purpose |
|-----------|---------|---------|
| OA_DB_SIZE | 100 | Maximum tracked obstacles |
| OA_DB_EXPIRE | 10s | Object removal timeout |
| OA_DB_BEAM_WIDTH | 5 deg | Lidar beam width for radius estimation |
| OA_DB_RADIUS_MIN | 0.01m | Minimum obstacle radius |
| OA_DB_DIST_MAX | 0 (unlimited) | Maximum storage distance |

### Combined Decision Making

Slope awareness and proximity obstacle avoidance are complementary:
- **Terrain slope** catches large-scale hazards (gullies, ridges, embankments)
- **Proximity sensors** catch local obstacles (rocks, vegetation, fences, animals)

Both feed into the same margin calculation in `calc_avoidance_margin()`. The minimum
margin across all sources determines whether a bearing is acceptable. No special
integration is needed -- the architecture already supports this.

### Recommendations for Rover Proximity Setup

For a desert survey rover with magnetometer boom:

1. **Forward-facing lidar or radar** -- detects obstacles in the path of travel
2. **Mount height matters** -- ground-level returns from desert scrub can overwhelm a
   low-mounted lidar. Mount at chassis height or use radar (less affected by dust/scrub)
3. **OA_DB_EXPIRE** -- increase to 30-60s in featureless desert (obstacles don't move)
4. **OA_DB_DIST_MAX** -- set to 2x lookahead distance to limit memory/CPU use
5. **PROX_IGN_ANG/PROX_IGN_WID** -- configure ignore zones for the magnetometer boom
   so the sensor doesn't detect the boom as an obstacle

### Proximity + Slope Combined Example

With both terrain slope and proximity sensors enabled:

```
OA_TYPE = 1                   # BendyRuler
OA_BR_LOOKAHEAD = 20          # 20m lookahead
OA_BR_SLOPE_MAX = 30          # reject slopes > 30%
OA_MARGIN_MAX = 5             # 5m obstacle clearance
OA_VEH_LENGTH = 3.0           # 3m total (rover + boom)
OA_VEH_WIDTH = 0.8            # 0.8m wide
PROX1_TYPE = 7                # RPLidar A2 (example)
PROX1_ORIENT = 0              # forward facing
OA_DB_SIZE = 50               # 50 obstacles tracked
OA_DB_EXPIRE = 30             # 30s persistence
```

BendyRuler will evaluate each candidate heading against:
1. Fence boundaries (if enabled)
2. Proximity obstacles (from lidar/radar, with vehicle footprint buffer)
3. Terrain slope (from AP_Terrain)

The heading with the best combined margin wins.

---

## PSRAM Performance Across ESP32 Architectures

Understanding PSRAM speed is important for deciding where to allocate large data
structures (terrain cache, OA database, Lua heap) on ESP32 boards.

### Speed Comparison

| Chip | PSRAM Interface | Clock | Bus Width | Throughput | Random Access Latency | vs Internal SRAM |
|------|----------------|-------|-----------|------------|----------------------|-----------------|
| ESP32 | SPI | 40/80 MHz | 1-bit | ~10 MB/s | 1-2 us | ~100x slower |
| ESP32-S2 | SPI/QSPI | 40-80 MHz | 1-4 bit | ~10-20 MB/s | 1-2 us | ~80-100x slower |
| ESP32-S3 | Octal SPI (OPI) | 120 MHz | 8-bit | 40-50 MB/s | 300-500 ns | ~15-30x slower |

### Internal SRAM Reference

| Chip | Internal SRAM | Usable DRAM | Access Latency |
|------|--------------|-------------|----------------|
| ESP32 | 520 KB | ~280-300 KB | ~10 ns (single cycle) |
| ESP32-S2 | 320 KB | ~200-230 KB | ~10 ns |
| ESP32-S3 | 512 KB | ~320-370 KB | ~10 ns |

### Practical Implications for ArduPilot

**Keep in internal SRAM** (latency-sensitive):
- Control loop variables, PID state, filter delay lines
- Interrupt handlers and DMA buffers (PSRAM not safely accessible from ISR)
- Small frequently-accessed structures (sensor data, EKF state)

**Safe to put in PSRAM** (bulk data, sequential access):
- Terrain cache blocks (2 KB each, accessed sequentially during queries)
- OA Database obstacle array (100 entries * ~40 bytes = 4 KB)
- Lua scripting heap (43-200 KB depending on mem class)
- Logger write buffers (sequential writes)
- Mission waypoint storage (read once per waypoint)

**ESP32-S3 advantage:** The OPI interface makes PSRAM 3-5x more viable than on the
original ESP32. Terrain cache queries (~20us each on S3 vs ~50-100us on ESP32) and Lua
execution (heap objects in PSRAM) benefit significantly.

**HAL_MEM_CLASS policy:** The platform default `HAL_MEM_CLASS` is based on internal DRAM
only (ESP32/S3 = 300, S2 = 192). Boards with PSRAM can override to 500 or 1000 in their
hwdef.dat for larger Lua heaps and logger buffers, accepting the PSRAM speed penalty.

---

## Parameters

### BendyRuler parameters (in AP_OABendyRuler::var_info)

| Parameter | Name | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| OA_BR_SLOPE_MAX | Slope Max | 0 (disabled) | 0-100 | Maximum terrain slope in percent. 0 disables slope checking. 30 = ~17 degrees, typical for off-road vehicles. |

### Vehicle dimension parameters (in AP_OAPathPlanner or ParametersG2)

| Parameter | Name | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| OA_VEH_LENGTH | Vehicle Length | 0 (point mass) | 0-10 | Total vehicle length in meters including booms/trailers. |
| OA_VEH_WIDTH | Vehicle Width | 0 (point mass) | 0-5 | Total vehicle width in meters. |

Note: `TURN_RADIUS` (default 0.9m) already exists in Rover ParametersG2 and should be
used for swept-path calculations when evaluating heading changes.

### Rover speed scaling parameters (in Mode or ParametersG2)

| Parameter | Name | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| SLOPE_SPEED_SC | Speed Scale | 0 (disabled) | 0-100 | Slope percent at which speed is reduced to minimum. 0 disables. |
| SLOPE_SPEED_MIN | Speed Min | 0.3 | 0.1-1.0 | Minimum speed factor on slopes (fraction of cruise speed). |

### Example configurations

**Conservative off-road rover:**
```
OA_BR_SLOPE_MAX = 25      # reject paths steeper than 25% (~14 degrees)
OA_VEH_LENGTH = 1.5       # 1.5m long rover
OA_VEH_WIDTH = 0.6        # 0.6m wide
SLOPE_SPEED_SC = 40        # full speed reduction at 40% grade
SLOPE_SPEED_MIN = 0.3      # never slower than 30% of cruise speed
```

**Desert survey rover with magnetometer boom:**
```
OA_BR_SLOPE_MAX = 30       # desert terrain, moderate slopes
OA_VEH_LENGTH = 3.0        # rover body + 1.5m boom behind
OA_VEH_WIDTH = 0.8
TURN_RADIUS = 2.5          # wider turns needed with boom
SLOPE_SPEED_SC = 40
SLOPE_SPEED_MIN = 0.3
PROX1_TYPE = 7             # forward lidar/radar
PROX1_IGN_ANG1 = 180       # ignore boom behind rover
PROX1_IGN_WID1 = 30        # 30-degree ignore zone
```

**Aggressive all-terrain:**
```
OA_BR_SLOPE_MAX = 50       # reject only very steep terrain
OA_VEH_LENGTH = 0          # point mass (small agile rover)
SLOPE_SPEED_SC = 60
SLOPE_SPEED_MIN = 0.2
```

---

## Dependencies

| Requirement | Status |
|-------------|--------|
| AP_TERRAIN_AVAILABLE = 1 | Must be enabled (currently disabled on ESP32, guarded with #ifndef) |
| AP_OAPATHPLANNER_ENABLED = 1 | Must be enabled for slope-aware path selection |
| SD card with terrain data | Needed for persistent terrain cache |
| GCS terrain download or pre-load | Needed to populate terrain data |
| Fence (optional) | BendyRuler works with or without fence |

**Graceful degradation:** Both features fail-open. If terrain data is unavailable,
slope checks return "no constraint" and speed scaling returns 1.0 (no change).

---

## Files to Modify

| File | Change |
|------|--------|
| `libraries/AC_Avoidance/AP_OABendyRuler.h` | Add `_slope_max` parameter, `calc_margin_from_terrain_slope()` method |
| `libraries/AC_Avoidance/AP_OABendyRuler.cpp` | Implement slope margin check, add to `calc_avoidance_margin()` |
| `libraries/AC_Avoidance/AP_OAPathPlanner.h` | Add `_veh_length`, `_veh_width` parameters |
| `libraries/AC_Avoidance/AP_OAPathPlanner.cpp` | Pass vehicle dimensions to BendyRuler |
| `libraries/AC_Avoidance/AP_OABendyRuler.cpp` | Subtract vehicle radius from obstacle margins |
| `libraries/AR_WPNav/AR_WPNav.h` | Add `calc_terrain_speed_factor()` method |
| `libraries/AR_WPNav/AR_WPNav.cpp` | Implement speed scaling, apply in `advance_wp_frac_and_target_speed()` |
| Board configs (optional) | Enable `AP_TERRAIN_AVAILABLE 1` and `AP_OAPATHPLANNER_ENABLED 1` |

No new files needed. No changes to AP_Terrain or AP_Proximity.

---

## Testing Strategy

### SITL Testing

1. **Terrain data:** SITL supports terrain via SRTM data. Use a location with known
   hills (e.g., Colorado foothills).

2. **Test cases:**
   - Waypoint behind a steep ridge: verify BendyRuler routes around it
   - Waypoint up a moderate slope: verify speed reduction on approach
   - Waypoint on flat terrain: verify no speed penalty
   - No terrain data available: verify fail-open (no speed penalty, no path rejection)
   - Terrain data gap mid-route: verify graceful handling

3. **Log analysis:** Check `OA.margin` logs for slope margin values, `CTUN.TargSpd`
   for speed scaling effects.

### Hardware Testing

1. **Pre-load terrain data** on SD card for the test area
2. **Run AUTO mission** over known terrain with GPS logging
3. **Compare:** Logged slope estimates vs actual terrain (from survey data or GPS altitude)
4. **Measure:** Speed reduction on grades, path deviation around steep areas

### Unit Tests

- `calc_margin_from_terrain_slope()` with mocked terrain data
- Speed scaling math at various slope percentages
- Edge cases: zero distance, very short segments, terrain at cache boundary

---

## Risks and Mitigations

1. **Terrain data accuracy:** SRTM data has ~30m horizontal resolution and ~16m vertical
   accuracy. At 100m grid spacing (ArduPilot default), slope calculations may miss narrow
   gullies or cliffs.
   **Mitigation:** This is a planning aid, not a safety system. The rover still has normal
   failsafe mechanisms (battery, geofence, manual override).

2. **Performance:** 272 terrain queries per BendyRuler cycle in worst case.
   **Mitigation:** Runs in background thread. Early-out on already-rejected bearings.
   Cache hit rate should be >99% for nearby points.

3. **Oscillation:** Terrain data boundaries could cause flickering slope estimates.
   **Mitigation:** BendyRuler's existing `resist_bearing_change()` prevents rapid heading
   changes. Speed scaling uses a filtered slope estimate (EMA or moving average).

4. **Compass/GPS accuracy:** Slope calculation depends on knowing which direction the
   rover is heading and where it is. Poor GPS (<3m accuracy) or compass errors could
   cause incorrect slope estimates for nearby sample points.
   **Mitigation:** Minimum lookahead distance of 5m ensures sample points are far enough
   apart that position error doesn't dominate slope calculation.

---

## Implementation Phases

**Phase 1: Vehicle footprint parameters** (simplest, high impact for safety)
- Add `OA_VEH_LENGTH` and `OA_VEH_WIDTH` parameters
- Subtract vehicle half-dimension from obstacle margins in `calc_avoidance_margin()`
- SITL test with simulated proximity obstacles

**Phase 2: Slope-aware BendyRuler**
- Add `OA_BR_SLOPE_MAX` parameter
- Implement `calc_margin_from_terrain_slope()`
- Wire into `calc_avoidance_margin()`
- SITL test with terrain data

**Phase 3: Speed scaling on grades**
- Add `SLOPE_SPEED_SC` and `SLOPE_SPEED_MIN` parameters
- Implement `calc_terrain_speed_factor()`
- Apply in AR_WPNav speed calculation
- Test in SITL and on hardware

**Phase 4: Swept-path turning radius** (optional refinement)
- Use `TURN_RADIUS` + `OA_VEH_LENGTH` to calculate swept width on turns
- Inflate margin requirements when candidate bearing requires large heading change
- Test with tight waypoint patterns

**Phase 5: Upstream PR**
- Clean up, document, write wiki page
- Submit PR to ArduPilot/ardupilot
- Address maintainer review feedback

---

## Reference: Slope Percentage to Angle Conversion

| Slope % | Angle (degrees) | Typical terrain |
|---------|-----------------|-----------------|
| 10% | 5.7 | Gentle hill |
| 20% | 11.3 | Moderate slope |
| 30% | 16.7 | Steep hillside |
| 45% | 24.2 | Very steep, off-road limit for most vehicles |
| 60% | 31.0 | Extreme, risk of rollover |
| 100% | 45.0 | Cliff face |
