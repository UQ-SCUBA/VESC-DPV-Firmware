# CUDA DPV - Acoustic Battery Status Specification

## Project Overview
Adding FOC tone-based acoustic battery status feedback to custom VESC firmware for CUDA underwater scooters (DPV).

## Hardware Specifications

### VESC/Controller
- **Model**: FlipSky 75100 PRO
- **Firmware Version**: 1.0
- **Hardware Version**: 1.0

### Battery Configuration
- **Battery Type**: Li-ion
- **Nominal Voltage**: 48.1V
- **Cell Configuration**: 13S
- **Voltage Range**:
  - Fully Charged: 54.6V
  - Nominal: 48.1V
  - Low Warning: 44.0V
  - Cutoff: 40.0V

### Motor
- **Model**: BFA-42-200
- **Max Motor RPM**: 6000 RPM
- **Max ERPM**: 10,000 (will update in code from current 8000)
- **Pole Pairs**: 4

### Environment
- **Operating Environment**: Underwater (affects acoustic propagation)
- **Typical Depth**: 100 meters
- **Water Type**: Saltwater

## Current Functionality (Existing Code)

### Tap Control System
- Single trigger pull: Start motor at default speed
- Release + 1 tap: Speed down
- Release + 2 taps: Speed up
- Release + 4 taps: Reverse mode (20% max ERPM)
- Quick release (no taps): Stop motor

### Speed Settings
Current speed array: 10%, 15%, 25%, 35%, 40%, 55%, 75%, 85%, 100% of MAX_ERPM
- Start speed index: 2 (25%)

## Desired Acoustic Feedback Features

### Primary Requirement
**Battery status indication through FOC motor beeps**

### Acoustic Feedback Scenarios

#### 1. Startup/Power-On Beep
- **When**: When the battery is first connected to the VESC
- **Purpose**: Confirm system is active and provide immediate battery status
- **Pattern**: Based on battery voltage at startup

#### 2. Periodic Battery Status
- **When**: 
  - After motor has run for 20+ seconds AND trigger is released to stop
  - When battery state transitions between voltage thresholds during operation
- **Pattern**: Based on current battery voltage

### Battery Level Indicator Patterns

All beeps at **3000 Hz** frequency:

| Battery Level | Voltage Range | Pattern | Description |
|---------------|---------------|---------|-------------|
| **Critical** | ≤25% (≤43.15V) | 1 long beep | 3.0 second duration |
| **Low** | 26-50% (43.16-47.35V) | 2 short beeps | 0.5s beep, 0.5s gap, 0.5s beep |
| **Medium** | 51-75% (47.36-51.45V) | 3 short beeps | 0.5s beep, 0.5s gap (×3) |
| **Good** | 76-100% (51.46-54.6V) | 4 short beeps | 0.5s beep, 0.5s gap (×4) |

**Voltage Calculations:**
- 100%: 54.6V
- 75%: 51.45V  (54.6 - 40.0) × 0.75 + 40.0
- 50%: 47.35V  (54.6 - 40.0) × 0.50 + 40.0
- 25%: 43.15V  (54.6 - 40.0) × 0.25 + 40.0
- 0%: 40.0V (cutoff)

### Acoustic Considerations for Underwater Use

#### Frequency Selection
- **Frequency**: 3000 Hz
- **Reasoning**: Will require underwater testing to verify audibility
- **Notes**: Water attenuates high frequencies more than air; may need adjustment after testing

#### Duration & Timing
- **Long beep**: 3.0 seconds
- **Short beep**: 0.5 seconds
- **Gap between beeps**: 0.5 seconds
- **Notes**: Longer durations may improve underwater audibility

## E-Clutch Safety Feature

### Primary Requirement
**Protect motor from stall conditions and out-of-water operation**

### Functionality
- **What it does**: Reduces motor to low power & low RPM safety mode
- **When it activates**:
  1. High current draw AND near-zero RPM (stall condition)
  2. Very low load for the current RPM (prop out of water)

### Protection Rationale
- Prevents motor overheating in stall conditions
- Allows safe trigger release if runaway occurs out of water
- Protects mechanical shaft seal from damage

### Implementation Details
- **Stall Detection Threshold**: 
  - Current: >XX amps (TBD based on testing)
  - RPM: <200 ERPM
- **Low Load Detection**:
  - Expected current at RPM significantly lower than actual
  - Indicates prop is not in water
- **Safety Mode**:
  - Reduce to 10-15% of target ERPM
  - Limit current to safe thermal level
  - Audible warning (rapid beep pattern?)
  - Require trigger release to reset

## Implementation Requirements

### Code Integration Points
1. Add `mcpwm_foc.h` include for beep functions
2. Create battery voltage monitoring function
3. Create beep pattern functions for each battery level
4. Add runtime timer for 20-second motor run detection
5. Implement e-clutch logic with current/RPM monitoring
6. Integrate into main thread loop

### New Constants/Variables Needed
```c
#define MAX_ERPM 10000.0f  // Update from 8000
#define BEEP_FREQUENCY 3000.0f
#define MOTOR_RUN_TIME_FOR_BEEP_MS 20000  // 20 seconds
#define ECLUTCH_STALL_CURRENT_THRESHOLD 40.0f  // TBD
#define ECLUTCH_STALL_RPM_THRESHOLD 200.0f
```

### Configuration Parameters
Should these be configurable via VESC Tool?
- [ ] Battery voltage thresholds
- [ ] Beep frequency (3000 Hz default)
- [ ] Beep durations
- [ ] Enable/disable acoustic feedback
- [ ] E-clutch thresholds
- [ ] Enable/disable e-clutch

### Safety Considerations
- Beeps should not interfere with motor control
- E-clutch should engage safely without sudden stops
- Battery beeps should not trigger during critical maneuvers
- Startup beep confirms system is operational

## Testing Plan

### Bench Testing (Out of Water)
- [ ] Verify beep generation works at 3000 Hz
- [ ] Confirm voltage thresholds trigger correctly (test with power supply at different voltages)
- [ ] Test all 4 battery level beep patterns
- [ ] Verify 20-second run timer works correctly
- [ ] Test e-clutch activation with blocked prop (stall)
- [ ] Test e-clutch activation with no load (out of water simulation)
- [ ] Verify no interference with motor control during beeps

### Underwater Testing
- [ ] Test audibility of 3000 Hz beeps at operating depth (100m)
- [ ] Test audibility at various depths (surface, 30m, 60m, 100m)
- [ ] Verify beep patterns are distinguishable underwater
- [ ] Test in saltwater
- [ ] Confirm beep duration (0.5s, 3.0s) is adequate
- [ ] May need to adjust frequency if 3000 Hz not audible

### Battery Drain Testing
- [ ] Monitor battery from full (54.6V) to cutoff (40.0V)
- [ ] Verify transitions between battery states at correct voltages
- [ ] Confirm startup beep accurately reflects battery level
- [ ] Test periodic beep after 20+ second runs

### E-Clutch Testing
- [ ] Test stall protection (block prop while running)
- [ ] Test out-of-water protection (run in air)
- [ ] Verify safe power reduction
- [ ] Confirm recovery after e-clutch engagement
- [ ] Test at various power levels

## Future Enhancements (Optional)

### Possible Additional Features
- [ ] Different beep patterns for different speed settings
- [ ] Reverse mode confirmation beep (on 4-tap activation)
- [ ] E-clutch engagement warning beep (rapid pulse?)
- [ ] Overheat warning beeps
- [ ] Overcurrent warning beeps
- [ ] Customizable beep sequences via VESC Tool
- [ ] Battery remaining time estimation (based on current draw)
- [ ] Data logging of battery state, motor temps, e-clutch events

## Notes and Observations

### Development Notes
- Need to determine optimal e-clutch current threshold through testing
- May need low-pass filtering on current readings to avoid false e-clutch triggers
- Consider hysteresis on battery voltage thresholds to prevent beep oscillation at boundaries

### Testing Observations
- 3000 Hz frequency selection is preliminary - may need adjustment based on underwater testing
- Longer beep durations underwater may improve audibility but use more battery
- E-clutch may need different thresholds for different speed settings

### Known Issues
- None yet - initial development phase

---

## References
- VESC Firmware Repository: https://github.com/vedderb/bldc
- Your Repository: https://github.com/UQ-SCUBA/CUDA_BT_UP_DATE
- FlipSky 75100 PRO Documentation: [Add if available]
- BFA-42-200 Motor Specifications: [Add if available]

## Version History
- v1.0 - Initial specification document
- v1.1 - Cleaned up formatting, added voltage calculations, detailed e-clutch requirements
