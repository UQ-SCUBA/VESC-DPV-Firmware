# CUDA DPV - Acoustic Battery Status Specification

## Project Overview
Adding FOC tone-based acoustic battery status feedback to custom VESC firmware for CUDA underwater scooters (DPV).

## Hardware Specifications

### VESC/Controller
- **Model**: [FlipSky 75100 PRO]
- **Firmware Version**: [1.0]
- **Hardware Version**: [1.0]

### Battery Configuration
- **Battery Type**: [Li-ion]
- **Nominal Voltage**: [54.6]
- **Cell Configuration**: [13s]
- **Voltage Range**:
  - Fully Charged: [54.6V] volts
  - Nominal: [48.1] volts
  - Low Warning: [44.0V] volts
  - Cutoff: [40.0v] volts

### Motor
- **Model**: [BFA-42-200]
- **Max ERPM**: 10,000 (as defined in code)(max motor rpm 6000)
- **Pole Pairs**: [4]

### Environment
- **Operating Environment**: Underwater (affects acoustic propagation)
- **Typical Depth**: [100]
- **Water Type**: [Saltwater and salt]

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
- **When**: [When the battery is connected]
- **Pattern**: [25% battery or less one beep long, 25% to 50% two beeps short, 51% to 75% 3 beeps short, 76% to 100% 4 beeps 0.5 short, long beep 3 seconds, short beeps 0.5 seconds with 0.5 second between them]
- **Purpose**: Confirm system is active and provide immediate battery status

#### 2. Periodic Battery Status
- **When**: [after motor has run for 20 seconds or longer and then the trigger is released to stop the motor or when the battery state changes from one threshold to another]
- **Pattern**: [25% battery or less one beep long, 25% to 50% two beeps short, 51% to 75% 3 beeps short, 76% to 100% 4 beeps 0.5 short, long beep 3 seconds, short beeps 0.5 seconds with 0.5 second between them]



### Acoustic Considerations for Underwater Use

#### Frequency Selection
- **Preferred Frequency Range**: [3000hz]
- **Reasoning**: ["Testing required"]
- **Notes**: Water attenuates high frequencies more than air

#### Volume/Intensity
- **Beep Duration**: [Longer may be more audible underwater]
- **Notes on Testing**: [Plan for underwater acoustic testing]

   ## E-clutch
### Primary Requirement
- **to protect the motor from stall conditions** 
  **what it should do it**:[put the motor into a low power & rpm mode]
   **when it should do it**: [when the controller has a high current draw and near zero rpm or if it senses very low load for the RPM ie out of the water]
-**Reasons** to protect the motor from over heat and allow the user to stop a runaway trigger out of water it will protect the mechanical seal from damage
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
