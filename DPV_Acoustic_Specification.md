# CUDA DPV - Acoustic Battery Status Specification

## Project Overview
Adding FOC tone-based acoustic battery status feedback to custom VESC firmware for CUDA underwater scooters (DPV).

## Hardware Specifications

### VESC/Controller
- **Model**: [Fill in your VESC/FOCBOX model]
- **Firmware Version**: [Current version]
- **Hardware Version**: [e.g., 4.12, 6.x]

### Battery Configuration
- **Battery Type**: [e.g., Li-ion, LiPo]
- **Nominal Voltage**: [e.g., 44.4V for 12S]
- **Cell Configuration**: [e.g., 12S4P]
- **Voltage Range**:
  - Fully Charged: [e.g., 50.4V] volts
  - Nominal: [e.g., 44.4V] volts
  - Low Warning: [e.g., 42.0V] volts
  - Cutoff: [e.g., 39.6V] volts

### Motor
- **Model**: [Your motor model]
- **Max ERPM**: 8000 (as defined in code)
- **Pole Pairs**: [number]

### Environment
- **Operating Environment**: Underwater (affects acoustic propagation)
- **Typical Depth**: [meters]
- **Water Type**: [Saltwater/Freshwater]

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
- **When**: Motor first starts (speedSetting changes from 0 to active)
- **Pattern**: [Describe desired pattern, e.g., "3 quick beeps at 2kHz"]
- **Purpose**: Confirm system is active and provide immediate battery status

#### 2. Periodic Battery Status
- **When**: [e.g., Every 60 seconds during operation, or on speed changes]
- **Frequency**: [How often?]
- **Pattern**: [Describe]

#### 3. Battery Level Indicators
Define beep patterns for different battery states:

**Full Battery (>90% / >XX.X volts)**
- Beeps: [e.g., 3 beeps]
- Frequency: [e.g., 2000 Hz]
- Duration: [e.g., 150ms each]
- Spacing: [e.g., 100ms between beeps]

**Good Battery (70-90% / XX.X - XX.X volts)**
- Beeps: [e.g., 2 beeps]
- Frequency: [e.g., 2000 Hz]
- Duration: [e.g., 150ms each]
- Spacing: [e.g., 100ms between beeps]

**Medium Battery (40-70% / XX.X - XX.X volts)**
- Beeps: [e.g., 2 beeps, lower tone]
- Frequency: [e.g., 1500 Hz]
- Duration: [e.g., 150ms each]
- Spacing: [e.g., 100ms between beeps]

**Low Battery (20-40% / XX.X - XX.X volts)**
- Beeps: [e.g., 1 long beep]
- Frequency: [e.g., 1000 Hz]
- Duration: [e.g., 300ms]
- Spacing: N/A

**Critical Battery (<20% / <XX.X volts)**
- Beeps: [e.g., Rapid pulsing pattern]
- Frequency: [e.g., 800 Hz]
- Duration: [e.g., 100ms pulses]
- Pattern: [e.g., 5 rapid beeps]

#### 4. Warning Conditions
- **Low Battery Warning**: [When voltage drops below threshold]
- **Critical Battery**: [Approaching cutoff voltage]
- **Pattern**: [More urgent/distinctive pattern]

### Acoustic Considerations for Underwater Use

#### Frequency Selection
- **Preferred Frequency Range**: [e.g., 1000-3000 Hz]
- **Reasoning**: [e.g., "Better propagation underwater" or "Testing required"]
- **Notes**: Water attenuates high frequencies more than air

#### Volume/Intensity
- **Beep Duration**: [Longer may be more audible underwater]
- **Notes on Testing**: [Plan for underwater acoustic testing]

## Implementation Requirements

### Code Integration Points
1. Add `mcpwm_foc.h` include for beep functions
2. Create battery monitoring function
3. Create beep pattern functions
4. Integrate into main thread loop

### Configuration Parameters
Should these be configurable via VESC Tool?
- [ ] Battery voltage thresholds
- [ ] Beep frequencies
- [ ] Beep durations
- [ ] Enable/disable acoustic feedback

### Safety Considerations
- Beeps should not interfere with motor control
- Should not trigger during critical operations
- Consider startup self-test beep sequence

## Testing Plan

### Bench Testing (Out of Water)
- [ ] Verify beep generation works
- [ ] Confirm voltage thresholds trigger correctly
- [ ] Test all battery level patterns
- [ ] Verify no interference with motor control

### Underwater Testing
- [ ] Test audibility at operating depth
- [ ] Verify frequency selection is appropriate
- [ ] Test in both saltwater and freshwater (if applicable)
- [ ] Confirm patterns are distinguishable underwater

### Battery Drain Testing
- [ ] Monitor battery from full to cutoff
- [ ] Verify transitions between battery states
- [ ] Confirm thresholds are accurate for your battery

## Future Enhancements (Optional)

### Possible Additional Features
- [ ] Different beep patterns for different speed settings
- [ ] Reverse mode confirmation beep
- [ ] Error condition beeps (overheat, overcurrent, etc.)
- [ ] Customizable beep sequences via VESC Tool
- [ ] Battery remaining time estimation (based on current draw)

## Notes and Observations

### Development Notes
[Add notes during development]

### Testing Observations
[Record findings from testing]

### Known Issues
[Document any problems or limitations]

---

## References
- VESC Firmware Repository: https://github.com/vedderb/bldc
- Your Repository: https://github.com/UQ-SCUBA/CUDA_BT_UP_DATE
- Andreas Spiess Video: [Add link if relevant]

## Version History
- v1.0 - Initial specification document
- [Add versions as you update]
