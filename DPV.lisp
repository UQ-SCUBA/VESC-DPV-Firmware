; ============================================================================
; TFM DPV - Diver Propulsion Vehicle Control Application
; VESC LispBM Port (from C firmware app_TFM_DPV.c)
; Finite State Machine architecture
;
; Hardware target: Flipsky 75/100 V2 (hw_75_100_V2_divex)
; Motor:           DiveX Cuda 650 thruster
; Encoder:         AS5047P magnetic encoder via SPI1
; Trigger:         Momentary switch on PPM/Servo pin (PB6)
;
; Features:
;   - 9-speed discrete control via trigger tap commands
;   - Double-tap to start from stopped
;   - Battery level feedback on 3-second hold from stopped
;       Delivered as encoder-controlled motor oscillation (tactile vibration):
;       Critical(0): 1 long burst  Low(1): 2 short  Medium(2): 3 short  Good(3): 4 short
;   - E-clutch stall protection (acoustic alarm + motor cut)
;   - 3-tap + 2s hold: Speed 8 (100% max)
;   - 4-tap + 2s hold: Engage cruise control (trigger press or e-clutch disengages)
;   - All tunable parameters adjustable via VESC Tool scripting tab (persisted to EEPROM)
;
; Trigger Commands:
;   From stopped:
;     Double-tap (tap + release + tap):   Start motor at speed index 2 (25%)
;     Hold 3 seconds:                     Battery level vibration feedback
;       (release then re-press to stop feedback)
;   While running:
;     Release (no re-tap within window):  Stop motor
;     1 tap + hold:                       Speed down
;     2 taps + hold:                      Speed up
;     3 taps + hold 2s:                   Speed 8 (100%, ramped)
;     4 taps + hold 2s:                   Engage cruise control
;   While in cruise control:
;     Trigger press:                      Disengage cruise, stop motor
;     E-clutch stall:                     Disengage cruise, stop motor
;
; FSM States:
;   startup       Plays startup battery vibration feedback, then ? idle
;   idle          Motor off, trigger up, waiting for input
;   idle-press    Trigger held from idle: timing 3s hold vs. first tap of start sequence
;   idle-tap1     First tap released: waiting for second tap within window
;   batt-fb       Playing battery vibration feedback (motor stopped)
;   running       Motor on, trigger held
;   tapping       Trigger released while running: counting taps (trigger up)
;   tap-held      Trigger held within tap sequence: waiting for release or window timeout
;   hold-confirm  Waiting 2s hold confirmation for 3-tap or 4-tap command
;   cruise-wait   Cruise just activated: waiting for trigger to be released
;   cruise        Cruise active: motor holds speed, trigger released
;   ec-warn       E-clutch stall: motor cut, playing acoustic warning
;   stopping      Motor off, waiting for trigger release before ? idle
;
; Hardware: Connect momentary trigger switch between SERVO pin and 3.3V.
;           Switch HIGH (3.3V) = squeezed. Pin has internal pull-down.
;
; SETUP STEPS:
;   1. Flash fw_75_100_V2_divex firmware
;   2. In VESC Tool: Motor -> FOC -> run motor detection
;   3. Set Sensor Mode = Encoder, Encoder Type = AS504x
;   4. Note the detected ERPM at full throttle -> set max-erpm below
;   5. Set app timeout > 4s (already set in hw header to 10s)
;   6. Load this script via VESC Tool > LispBM Scripting tab
;
; Note: Ensure the VESC motor timeout is set longer than the longest
;       vibration sequence to prevent timeout during feedback playback.
; ============================================================================

; ============================================================================
; CONFIGURATION - Adjust these values for your specific setup
; ============================================================================

; --- Trigger Input Pin ---
(def trigger-pin 'pin-ppm)

; --- Speed Control ---
; IMPORTANT: Set max-erpm to the value from VESC Tool motor detection.
(def max-erpm 15000.0)         ; Maximum motor ERPM - SET FROM MOTOR DETECTION
(def start-speed-idx 2)        ; Index into speeds list for initial start
(def ramp-up-time 2.5)         ; Seconds to ramp from 0 to max-erpm

; Speed levels as percentage of max-erpm (adjustable via scripting tab)
; Index:  0    1    2    3    4    5    6    7    8
(def speeds (list 8.0 15.0 25.0 35.0 40.0 55.0 75.0 85.0 100.0))
(def num-speeds (length speeds))

; Firmware defaults for speeds (used by dpv-defaults command)
(def default-speeds (list 10.0 15.0 25.0 35.0 40.0 55.0 75.0 85.0 100.0))

; --- Tap Timing (seconds) ---
(def tap-duration 0.25)        ; Time window for next tap (250ms)
(def debounce-time 0.02)       ; Debounce delay after press/release (20ms)
(def max-taps 4)               ; Maximum taps in a running command sequence
(def hold-confirm-time 2.0)    ; Hold duration to confirm 3-tap/4-tap command (s)
(def start-tap-window 0.5)     ; Window after first tap to detect second tap (s)
(def start-hold-time 3.0)      ; Hold duration from stopped for battery feedback (s)

; --- Battery Thresholds (adjustable via scripting tab) ---
; Set for your DPV battery. Examples:
;   12S Li-ion: 75%=46.2V  50%=43.8V  25%=40.8V  (3.85/3.65/3.40 per cell)
;   10S Li-ion: 75%=38.5V  50%=36.5V  25%=34.0V
; Multiply per-cell voltage by cell count to get pack threshold.
(def batt-75v 46.2)            ; 75% voltage threshold (12S default)
(def batt-50v 43.8)            ; 50% voltage threshold (12S default)
(def batt-25v 40.8)            ; 25% voltage threshold (12S default)

; Firmware defaults for battery thresholds
(def default-batt-75v 46.2)
(def default-batt-50v 43.8)
(def default-batt-25v 40.8)

; --- Vibration Feedback (Battery Level) ---
; Battery level is communicated as encoder-controlled motor oscillation felt
; through the handle.  set-current alternates between +vibe-current and
; -vibe-current at vibe-freq Hz, creating a physical thump/pulse.
; Using set-current (torque control) rather than set-rpm means the current
; drawn is exactly vibe-current - it cannot exceed this value, protecting
; the drivetrain during feedback.
;
; Tuning notes:
;   - Increase vibe-current for a heavier, stronger pulse  (try 3 - 10 A)
;   - Decrease vibe-freq for a slower, more distinct thump  (try 50 - 300 Hz)
;   - Good starting point underwater: vibe-current 5, vibe-freq 200
(def vibe-current 5.0)         ; Oscillation current magnitude (A) - hard cap on feedback amps
(def vibe-freq 200.0)          ; Oscillation frequency (Hz)
(def vibe-short-cyc 40)        ; Cycles per short burst
(def vibe-long-cyc  80)        ; Cycles per long burst (critical battery level)
(def vibe-gap 0.7)             ; Pause between bursts (s)

; --- E-Clutch Alarm ---
; The e-clutch warning uses a high-pitched acoustic foc-beep, clearly distinct
; from the low-frequency vibration battery feedback.
(def beep-volt 2.0)            ; foc-beep drive voltage (0.5-2.0)

; --- Startup ---
(def startup-delay 3.0)        ; Delay before startup battery feedback (s)

; --- E-Clutch Safety (adjustable via scripting tab) ---
(def ec-stall-current 3.0)     ; Current above which stall is suspected (A)
(def ec-stall-rpm 10.0)        ; RPM below which stall is suspected
(def ec-stall-time 0.3)        ; Time stall must persist before activation (s)
(def ec-recovery-rpm 200.0)    ; RPM above which e-clutch condition clears
(def ec-warn-freq 1500.0)      ; Warning beep frequency (Hz)
(def ec-warn-dur 0.1)          ; Warning beep duration (s)
(def ec-warn-count 5)          ; Number of warning beeps

; Firmware defaults for e-clutch
(def default-ec-stall-current 3.0)
(def default-ec-stall-rpm 10.0)
(def default-ec-stall-time 0.3)
(def default-ec-recovery-rpm 200.0)

; --- EEPROM Address Layout (LispBM float slots 0-63) ---
; Slot 0:    Magic number (confirms settings have been saved)
; Slots 1-9: speeds[0] through speeds[8]
; Slot 10:   batt-75v
; Slot 11:   batt-50v
; Slot 12:   batt-25v
; Slot 13:   ec-stall-current
; Slot 14:   ec-stall-rpm
; Slot 15:   ec-stall-time
; Slot 16:   ec-recovery-rpm
(def eep-magic-addr   0)
(def eep-magic-val    12345.0)
(def eep-speed-base   1)
(def eep-b75-addr     10)
(def eep-b50-addr     11)
(def eep-b25-addr     12)
(def eep-ec-cur-addr  13)
(def eep-ec-rpm-addr  14)
(def eep-ec-tim-addr  15)
(def eep-ec-rec-addr  16)

; ============================================================================
; FSM STATE
; ============================================================================

(def state 'startup)           ; Current FSM state (see header for state list)
(def state-timer (systime))    ; Systime recorded when the current state was entered

; Motor control
(def speed-setting 0)          ; Current speed index (0 = stopped)
(def current-speed 0.0)        ; Current speed percentage
(def actual-erpm 0.0)          ; Ramped ERPM currently commanded to motor
(def trig nil)                 ; Trigger state sampled once per loop tick

; Tap counting (used in tapping / tap-held / hold-confirm states)
(def tap-count 0)              ; Number of taps counted in the current sequence

; E-Clutch stall detection
(def ec-cond nil)              ; True while stall condition is actively detected
(def ec-det-time (systime))    ; Systime when stall condition was first seen

; ============================================================================
; LIST UTILITY
; ============================================================================

; Return a new list with the element at idx replaced by val
(defun list-set (lst idx val) {
    (var result (list))
    (looprange i 0 (length lst) {
        (setq result (append result (list (if (= i idx) val (ix lst i)))))
    })
    result
})

; ============================================================================
; EEPROM PERSISTENCE FUNCTIONS
; ============================================================================

; Save all tunable settings to EEPROM
(defun eeprom-save-all () {
    (looprange i 0 num-speeds {
        (eeprom-store-f (+ eep-speed-base i) (ix speeds i))
    })
    (eeprom-store-f eep-b75-addr    batt-75v)
    (eeprom-store-f eep-b50-addr    batt-50v)
    (eeprom-store-f eep-b25-addr    batt-25v)
    (eeprom-store-f eep-ec-cur-addr ec-stall-current)
    (eeprom-store-f eep-ec-rpm-addr ec-stall-rpm)
    (eeprom-store-f eep-ec-tim-addr ec-stall-time)
    (eeprom-store-f eep-ec-rec-addr ec-recovery-rpm)
    ; Write magic last so a partial save is not mistaken for a valid save
    (eeprom-store-f eep-magic-addr eep-magic-val)
})

; Load all tunable settings from EEPROM if a valid save is present.
; Returns t if loaded from EEPROM, nil if using firmware defaults.
(defun eeprom-load-all () {
    (if (= (to-float (eeprom-read-f eep-magic-addr)) eep-magic-val)
        {
            (looprange i 0 num-speeds {
                (setq speeds (list-set speeds i (eeprom-read-f (+ eep-speed-base i))))
            })
            (setq batt-75v         (eeprom-read-f eep-b75-addr))
            (setq batt-50v         (eeprom-read-f eep-b50-addr))
            (setq batt-25v         (eeprom-read-f eep-b25-addr))
            (setq ec-stall-current (eeprom-read-f eep-ec-cur-addr))
            (setq ec-stall-rpm     (eeprom-read-f eep-ec-rpm-addr))
            (setq ec-stall-time    (eeprom-read-f eep-ec-tim-addr))
            (setq ec-recovery-rpm  (eeprom-read-f eep-ec-rec-addr))
            t
        }
        nil)
})

; ============================================================================
; CONFIGURATION DISPLAY
; ============================================================================

; Print all current tunable settings to the VESC Tool terminal
(defun print-config () {
    (print "=== DPV Configuration ===")
    (print "Speed levels (% of max ERPM):")
    (looprange i 0 num-speeds {
        (print (str-merge "  [" (to-str i) "] " (str-from-n (ix speeds i) "%.1f") "%"))
    })
    (print "Battery thresholds:")
    (print (str-merge "  75%: " (str-from-n batt-75v "%.1f") " V"))
    (print (str-merge "  50%: " (str-from-n batt-50v "%.1f") " V"))
    (print (str-merge "  25%: " (str-from-n batt-25v "%.1f") " V"))
    (print "E-Clutch:")
    (print (str-merge "  stall current:  " (str-from-n ec-stall-current "%.2f") " A"))
    (print (str-merge "  stall rpm:      " (str-from-n ec-stall-rpm "%.1f") " RPM"))
    (print (str-merge "  stall time:     " (str-from-n ec-stall-time "%.3f") " s"))
    (print (str-merge "  recovery rpm:   " (str-from-n ec-recovery-rpm "%.1f") " RPM"))
    (print "Vibration feedback:")
    (print (str-merge "  vibe-current:   " (str-from-n vibe-current "%.1f") " A"))
    (print (str-merge "  vibe-freq:      " (str-from-n vibe-freq "%.1f") " Hz"))
    (print (str-merge "  short burst:    " (to-str vibe-short-cyc) " cycles"))
    (print (str-merge "  long burst:     " (to-str vibe-long-cyc) " cycles"))
    (print (str-merge "  burst gap:      " (str-from-n vibe-gap "%.2f") " s"))
})

; ============================================================================
; TRIGGER SLEEP HELPERS
; ============================================================================

; Sleep in small steps, returning t immediately if trigger is pressed
(defun sleep-or-trig (dur) {
    (var steps (to-i (* dur 500.0)))
    (var triggered nil)
    (loopwhile (and (> steps 0) (not triggered)) {
        (sleep 0.002)
        (timeout-reset)
        (setq steps (- steps 1))
        (setq triggered (= (gpio-read trigger-pin) 1))
    })
    triggered
})

; Sleep in small steps, returning t if trigger is RELEASED then RE-PRESSED.
; Used for gap pauses during battery feedback while motor is stopped.
(defun sleep-or-retrig (dur) {
    (var steps (to-i (* dur 500.0)))
    (var aborted nil)
    (var was-released (= (gpio-read trigger-pin) 0))
    (loopwhile (and (> steps 0) (not aborted)) {
        (sleep 0.002)
        (timeout-reset)
        (setq steps (- steps 1))
        (var curr (= (gpio-read trigger-pin) 1))
        (if (not curr) (setq was-released t))
        (if (and was-released curr) (setq aborted t))
    })
    aborted
})

; ============================================================================
; VIBRATION FEEDBACK FUNCTIONS
; ============================================================================

; Play num-cycles of motor oscillation using set-current.
; Alternates between +vibe-current and -vibe-current at vibe-freq Hz.
; Using set-current (torque control) means the motor draws exactly
; vibe-current amps - it cannot spike above this value.
; Aborts immediately if trigger is PRESSED.  Returns t if aborted.
(defun play-vibe-trig (num-cycles) {
    ; half-steps: number of 2 ms ticks per half-cycle.
    ; Clamped to >= 1 so the inner loop always sleeps at least one tick,
    ; giving the motor time to respond to each current command.
    ; At vibe-freq=300, to-i(0.5/300*500)=0 without the clamp, producing
    ; no perceptible oscillation.  max is not a builtin in fw 6.05, so use if.
    (var half-steps-raw (to-i (* (/ 0.5 vibe-freq) 500.0)))
    (var half-steps (if (< half-steps-raw 1) 1 half-steps-raw))
    (var h 0)
    (var aborted nil)
    (loopwhile (and (< h (* num-cycles 2)) (not aborted)) {
        (if (= (mod h 2) 0)
            (set-current vibe-current)
            (set-current (- vibe-current)))
        (var s 0)
        (loopwhile (and (< s half-steps) (not aborted)) {
            (sleep 0.002)
            (timeout-reset)
            (setq s (+ s 1))
            (setq aborted (= (gpio-read trigger-pin) 1))
        })
        (setq h (+ h 1))
    })
    (set-current 0)
    aborted
})

; Play num-cycles of motor oscillation using set-current.
; Aborts if trigger is RELEASED then RE-PRESSED.
; Used for battery feedback from the stopped state where the trigger may
; already be held when feedback begins.  Returns t if aborted.
(defun play-vibe-retrig (num-cycles) {
    (var half-steps-raw (to-i (* (/ 0.5 vibe-freq) 500.0)))
    (var half-steps (if (< half-steps-raw 1) 1 half-steps-raw))
    (var h 0)
    (var aborted nil)
    (var was-released (= (gpio-read trigger-pin) 0))
    (loopwhile (and (< h (* num-cycles 2)) (not aborted)) {
        (if (= (mod h 2) 0)
            (set-current vibe-current)
            (set-current (- vibe-current)))
        (var s 0)
        (loopwhile (and (< s half-steps) (not aborted)) {
            (sleep 0.002)
            (timeout-reset)
            (setq s (+ s 1))
            (var curr (= (gpio-read trigger-pin) 1))
            (if (not curr) (setq was-released t))
            (if (and was-released curr) (setq aborted t))
        })
        (setq h (+ h 1))
    })
    (set-current 0)
    aborted
})

; ============================================================================
; BATTERY FEEDBACK FUNCTIONS
; ============================================================================

; Validate that battery voltage thresholds are in strictly descending order.
; If they are not, log a warning and reset all three to firmware defaults.
; Call after eeprom-load-all and whenever thresholds are changed at runtime.
(defun validate-batt-thresholds () {
    (if (not (and (> batt-75v batt-50v) (> batt-50v batt-25v)))
        {
            (print "DPV: WARNING - battery thresholds out of order, resetting to defaults")
            (setq batt-75v default-batt-75v)
            (setq batt-50v default-batt-50v)
            (setq batt-25v default-batt-25v)
        })
})

; Get battery level from voltage: 0=critical, 1=low, 2=medium, 3=good
(defun get-batt-level (v)
    (if (>= v batt-75v) 3
        (if (>= v batt-50v) 2
            (if (>= v batt-25v) 1 0))))

; Play battery status via vibration pattern (abort on trigger press).
; Critical(0): 1 long burst  Low(1): 2 short  Medium(2): 3 short  Good(3): 4 short
; Used during startup and while running.
(defun play-batt-beep (level) {
    (var cnt (if (= level 0) 1 (+ level 1)))
    (var cyc (if (= level 0) vibe-long-cyc vibe-short-cyc))
    (var i 0)
    (var aborted nil)
    (loopwhile (and (< i cnt) (not aborted)) {
        (setq aborted (play-vibe-trig cyc))
        (if (and (not aborted) (< i (- cnt 1)))
            (setq aborted (sleep-or-trig vibe-gap)))
        (setq i (+ i 1))
    })
})

; Play battery status via vibration pattern (abort on trigger release then re-press).
; Used from the stopped state where the trigger may still be held from the 3s hold.
(defun play-batt-beep-stopped (level) {
    (var cnt (if (= level 0) 1 (+ level 1)))
    (var cyc (if (= level 0) vibe-long-cyc vibe-short-cyc))
    (var i 0)
    (var aborted nil)
    (loopwhile (and (< i cnt) (not aborted)) {
        (setq aborted (play-vibe-retrig cyc))
        (if (and (not aborted) (< i (- cnt 1)))
            (setq aborted (sleep-or-retrig vibe-gap)))
        (setq i (+ i 1))
    })
})

; ============================================================================
; E-CLUTCH SAFETY FUNCTIONS
; ============================================================================

; Play e-clutch warning - high-pitched acoustic beeps via foc-beep,
; clearly distinct from the low-frequency vibration battery feedback.
(defun play-beep (freq dur) {
    (foc-beep freq dur beep-volt)
    (timeout-reset)
    (sleep-or-trig dur)
})

(defun play-ec-warning () {
    (looprange i 0 ec-warn-count {
        (play-beep ec-warn-freq ec-warn-dur)
        (sleep 0.05)
    })
})

; Check for motor stall condition. Call each tick while motor is running.
; Returns t exactly once when a stall is first confirmed (ec-stall-time elapsed).
; Returns nil otherwise (stall building, no stall, or already confirmed this event).
(defun check-stall () {
    (var current (get-current-in))
    (var erpm (abs (get-rpm)))
    (var stalling (and (> current ec-stall-current) (< erpm ec-stall-rpm)))
    (if stalling
        {
            (if (not ec-cond)
                ; First tick of stall - start the confirmation timer
                { (setq ec-cond t) (setq ec-det-time (systime)) nil }
                ; Stall ongoing - confirm if enough time has elapsed
                (if (>= (secs-since ec-det-time) ec-stall-time)
                    t    ; Confirmed: caller should transition to ec-warn
                    nil))
        }
        ; No stall - clear condition flag
        { (setq ec-cond nil) nil })
})

; ============================================================================
; SPEED CONTROL FUNCTIONS
; ============================================================================

; Increase speed setting by one step
(defun speed-up () {
    (setq speed-setting (+ speed-setting 1))
    (if (>= speed-setting num-speeds)
        (setq speed-setting (- num-speeds 1)))
    (setq current-speed (ix speeds speed-setting))
})

; Decrease speed setting by one step (minimum index 0)
(defun speed-down () {
    (setq speed-setting (- speed-setting 1))
    (if (< speed-setting 0)
        (setq speed-setting 0))
    (setq current-speed (ix speeds speed-setting))
})

; Cut motor power and clear stall state.
; set-motor-speed will coast the motor to rest on the next tick.
(defun stop-motor () {
    (setq speed-setting 0)
    (setq current-speed 0.0)
    (setq ec-cond nil)
})

; Apply current speed to motor with ramp-up.
; Called every tick regardless of state so the motor always tracks current-speed.
(defun set-motor-speed () {
    (var ramp-step (* max-erpm (/ 0.002 ramp-up-time)))
    (if (> current-speed 0.0)
        {
            (var target (* (/ current-speed 100.0) max-erpm))
            (if (> target max-erpm) (setq target max-erpm))
            (if (< actual-erpm target)
                {
                    (setq actual-erpm (+ actual-erpm ramp-step))
                    (if (> actual-erpm target) (setq actual-erpm target))
                }
                (setq actual-erpm target))
            (set-rpm actual-erpm)
        }
        {
            (setq actual-erpm 0.0)
            (set-current 0)
        })
})

; ============================================================================
; FSM TRANSITION HELPER
; ============================================================================

; Transition to a new FSM state and record the entry time.
; All state transitions go through this function.
(defun go-to (s) {
    (setq state s)
    (setq state-timer (systime))
})

; ============================================================================
; FSM STATE HANDLERS
; ============================================================================

; --- startup ---
; Initial state. Waits for the startup delay then plays battery vibration
; feedback so the diver knows the unit is armed. Then transitions to idle.
; Motor is off throughout.
; The delay uses 2 ms steps with timeout-reset so the VESC watchdog cannot
; fire during the wait regardless of how startup-delay is configured.
(defun tick-startup () {
    (looprange i 0 (to-i (* startup-delay 500.0)) {
        (sleep 0.002)
        (timeout-reset)
    })
    (play-batt-beep (get-batt-level (get-vin)))
    (go-to 'idle)
})

; --- idle ---
; Motor off, trigger up. The only way out is a trigger press.
(defun tick-idle () {
    (if trig (go-to 'idle-press))
})

; --- idle-press ---
; Trigger held from idle. Two possible outcomes:
;   Hold = start-hold-time (3s)  ?  play battery feedback  (go to batt-fb)
;   Release before 3s            ?  first tap complete      (go to idle-tap1)
(defun tick-idle-press () {
    (if trig
        ; Trigger still held - check 3s threshold
        (if (>= (secs-since state-timer) start-hold-time)
            (go-to 'batt-fb))
        ; Released before 3s - first tap is done
        { (sleep debounce-time) (go-to 'idle-tap1) })
})

; --- idle-tap1 ---
; First tap of the start sequence has been released.
; Waiting for the second press within start-tap-window.
;   Second press within window  ?  start motor  (go to running)
;   Window timeout              ?  abandon      (go to idle)
(defun tick-idle-tap1 () {
    (if trig
        ; Second press - start the motor
        {
            (setq speed-setting start-speed-idx)
            (setq current-speed (ix speeds speed-setting))
            (sleep debounce-time)
            (go-to 'running)
        }
        ; No press yet - check window expiry
        (if (>= (secs-since state-timer) start-tap-window)
            (go-to 'idle)))
})

; --- batt-fb ---
; Plays battery vibration feedback while the motor is stopped.
; play-batt-beep-stopped blocks until the pattern completes or is aborted
; by a trigger release-then-repress. After playback, waits for the trigger
; to be fully released before returning to idle.
(defun tick-batt-fb () {
    (play-batt-beep-stopped (get-batt-level (get-vin)))
    (loopwhile (= (gpio-read trigger-pin) 1) {
        (sleep 0.002)
        (timeout-reset)
    })
    (sleep debounce-time)
    (go-to 'idle)
})

; --- running ---
; Motor running at current speed. Trigger must remain held.
;   Trigger released           ?  begin tap command sequence  (go to tapping)
;   Stall confirmed            ?  e-clutch alarm              (go to ec-warn)
(defun tick-running () {
    (if (check-stall)
        ; Stall confirmed - cut motor and play warning
        { (stop-motor) (go-to 'ec-warn) }
        ; No stall - check for trigger release
        (if (not trig)
            {
                (setq tap-count 0)
                (sleep debounce-time)
                (go-to 'tapping)
            }))
})

; --- tapping ---
; Trigger UP during a tap command sequence. tap-count holds taps so far.
;   Stall confirmed                      ?  e-clutch alarm  (go to ec-warn)
;   Trigger pressed                      ?  count tap  (go to tap-held)
;   Timeout with 0 taps (no retap)       ?  stop motor (go to stopping)
;   Timeout with 1+ taps (all released)  ?  ignore taps, resume running
;
; Note: "1 tap released" means the user tapped once but didn't hold - this
; is treated as a misfire and the motor continues at its current speed.
(defun tick-tapping () {
    (if (check-stall)
        ; Stall confirmed during tap sequence - treat as running stall
        { (stop-motor) (go-to 'ec-warn) }
        (if trig
            ; Press detected - count it and move to tap-held
            {
                (setq tap-count (+ tap-count 1))
                (sleep debounce-time)
                (go-to 'tap-held)
            }
            ; Still up - check tap window timeout
            (if (>= (secs-since state-timer) tap-duration)
                (if (= tap-count 0)
                    ; Plain release with no retap = stop
                    { (stop-motor) (go-to 'stopping) }
                    ; Taps counted but trigger not held at timeout = ignore
                    (go-to 'running)))))
})

; --- tap-held ---
; Trigger HELD during a tap command sequence.
;   Stall confirmed              ?  e-clutch alarm              (go to ec-warn)
;   Trigger released             ?  possibly more taps coming  (go back to tapping)
;   Tap window expires           ?  execute command for tap-count
;   Max taps reached (4)         ?  execute command immediately
(defun tick-tap-held () {
    (if (check-stall)
        ; Stall confirmed during tap sequence
        { (stop-motor) (go-to 'ec-warn) }
        (if (not trig)
            ; Released - reset window, allow more taps
            { (sleep debounce-time) (go-to 'tapping) }
            ; Held - execute if window expired or max taps reached
            (if (or (>= tap-count max-taps)
                    (>= (secs-since state-timer) tap-duration))
                (exec-tap-cmd))))
})

; Execute the tap command for the current tap-count (trigger is held).
;   1 tap  ? speed down, back to running
;   2 taps ? speed up,   back to running
;   3 taps ? wait for 2s hold confirmation (speed 8 or abort)
;   4 taps ? wait for 2s hold confirmation (cruise or abort)
;   other  ? unexpected; resume running unchanged
(defun exec-tap-cmd () {
    (if (= tap-count 1)
        { (speed-down) (go-to 'running) }
        (if (= tap-count 2)
            { (speed-up) (go-to 'running) }
            (if (or (= tap-count 3) (= tap-count 4))
                ; 3 or 4 taps - need hold confirmation
                (go-to 'hold-confirm)
                ; Unexpected tap count - resume running unchanged
                (go-to 'running))))
})

; --- hold-confirm ---
; Trigger held after a 3-tap or 4-tap sequence. Waiting hold-confirm-time
; seconds to confirm the command.
;   Stall confirmed            ?  e-clutch alarm              (go to ec-warn)
;   Trigger released early     ?  abort, stop motor  (go to stopping)
;   Held = hold-confirm-time, 3 taps  ?  set speed 8 (100%)  (go to running)
;   Held = hold-confirm-time, 4 taps  ?  engage cruise        (go to cruise)
(defun tick-hold-confirm () {
    (if (check-stall)
        ; Stall confirmed during hold confirmation
        { (stop-motor) (go-to 'ec-warn) }
        (if (not trig)
            ; Released early - abort
            { (stop-motor) (go-to 'stopping) }
            ; Still held - check confirmation time
            (if (>= (secs-since state-timer) hold-confirm-time)
                (if (= tap-count 3)
                    ; 3-tap confirmed: jump to speed 8 (100%)
                    {
                        (setq speed-setting (- num-speeds 1))
                        (setq current-speed (ix speeds (- num-speeds 1)))
                        (go-to 'running)
                    }
                    (if (= tap-count 4)
                        ; 4-tap confirmed: enter cruise-wait (trigger still held from confirm)
                        (go-to 'cruise-wait)
                        ; Unexpected tap count - resume running unchanged
                        (go-to 'running))))))
})

; --- cruise-wait ---
; Cruise was just activated from hold-confirm, but the trigger is still held
; from the 2s confirmation press. Wait here until the trigger is released
; before cruise goes live. This prevents the activation press from being
; misread as a stop command.
;   Stall confirmed  ?  stop motor  (go to ec-warn)
;   Trigger released ?  cruise is now live  (go to cruise)
(defun tick-cruise-wait () {
    (if (check-stall)
        { (stop-motor) (go-to 'ec-warn) }
        (if (not trig)
            (go-to 'cruise)))
})

; --- cruise ---
; Cruise control is live. Motor holds speed with trigger released.
; The trigger has been cleanly released at least once since activation.
;   Stall confirmed  ?  stop motor, play warning  (go to ec-warn)
;   Trigger pressed  ?  stop motor immediately     (go to stopping)
;     stopping waits for trigger release before returning to idle,
;     so the motor cannot restart while the trigger is still held.
(defun tick-cruise () {
    (if (check-stall)
        { (stop-motor) (go-to 'ec-warn) }
        (if trig
            { (stop-motor) (go-to 'stopping) }))
})

; --- ec-warn ---
; E-clutch stall confirmed. Motor has already been cut by the caller.
; Plays the acoustic alarm then transitions to stopping.
; The diver must double-tap to restart.
(defun tick-ec-warn () {
    (play-ec-warning)
    (go-to 'stopping)
})

; --- stopping ---
; Motor is off (set-motor-speed coasts to rest each tick).
; Waits here until the trigger is fully released, then goes to idle.
; This prevents a stale trigger press from immediately starting the motor.
(defun tick-stopping () {
    (if (not trig) (go-to 'idle))
})

; ============================================================================
; INITIALIZATION
; ============================================================================

(gpio-configure trigger-pin 'pin-mode-in-pd)
(sleep 0.25)

(if (eeprom-load-all)
    (print "DPV: settings loaded from EEPROM")
    (print "DPV: using firmware defaults (no EEPROM save found)"))
(validate-batt-thresholds)

; ============================================================================
; RUNTIME CONFIGURATION HELPERS
; Note: register-terminal-cmd is not available in firmware 6.05.
; To adjust settings at runtime, paste any of the following into the
; VESC Tool LispBM scripting tab "Evaluate expression" field and press Enter.
;
; Show current config:
;   (print-config)
;
; Change a speed level (index 0-8, percent 0-100):
;   (setq speeds (list-set speeds 3 45.0)) (eeprom-save-all)
;
; Change a battery threshold (validate ordering before saving):
;   (setq batt-75v 46.2) (setq batt-50v 43.8) (setq batt-25v 40.8) (validate-batt-thresholds) (eeprom-save-all)
;
; Change an e-clutch parameter:
;   (setq ec-stall-current 3.0) (eeprom-save-all)
;   (setq ec-stall-rpm 10.0)    (eeprom-save-all)
;   (setq ec-stall-time 0.3)    (eeprom-save-all)
;   (setq ec-recovery-rpm 200.0) (eeprom-save-all)
;
; Restore all defaults:
;   (looprange i 0 num-speeds { (setq speeds (list-set speeds i (ix default-speeds i))) })
;   (setq batt-75v default-batt-75v) (setq batt-50v default-batt-50v) (setq batt-25v default-batt-25v)
;   (setq ec-stall-current default-ec-stall-current) (setq ec-stall-rpm default-ec-stall-rpm)
;   (setq ec-stall-time default-ec-stall-time) (setq ec-recovery-rpm default-ec-recovery-rpm)
;   (eeprom-save-all)
; ============================================================================

; ============================================================================
; MAIN CONTROL LOOP (~500Hz)
; ============================================================================

(loopwhile t {
    ; Sample trigger once at the top of every tick
    (setq trig (= (gpio-read trigger-pin) 1))

    ; Dispatch to the current state handler.
    ; eq is used for symbol comparison; = is numeric-only in LispBM.
    (cond
        ((eq state 'startup)      (tick-startup))
        ((eq state 'idle)         (tick-idle))
        ((eq state 'idle-press)   (tick-idle-press))
        ((eq state 'idle-tap1)    (tick-idle-tap1))
        ((eq state 'batt-fb)      (tick-batt-fb))
        ((eq state 'running)      (tick-running))
        ((eq state 'tapping)      (tick-tapping))
        ((eq state 'tap-held)     (tick-tap-held))
        ((eq state 'hold-confirm) (tick-hold-confirm))
        ((eq state 'cruise-wait)  (tick-cruise-wait))
        ((eq state 'cruise)       (tick-cruise))
        ((eq state 'ec-warn)      (tick-ec-warn))
        ((eq state 'stopping)     (tick-stopping))
    )

    ; Update motor command every tick based on current speed-setting
    (set-motor-speed)

    (sleep 0.002)
    (timeout-reset)
})
