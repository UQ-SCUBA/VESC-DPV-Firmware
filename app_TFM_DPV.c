#include "ch.h" // ChibiOS
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout

// Constants
#define MAX_ERPM 8000.0f
#define TAP_MIN_TIME_MS 50
#define DEBOUNCE_TIME_MS 20
#define COMMAND_DURATION_MS 1000
#define TAP_DURATION_MS 250
#define REVERSE_ERPM_PERCENT 0.2f
#define START_SPEED_INDEX 2
#define NUM_SPEEDS (sizeof(speeds) / sizeof(speeds[0]))
#define MAX_TAPS 4

// Global Variables
float speeds[] = {10, 15, 25, 35, 40, 55, 75, 85, 100}; // % of Max ERPM 
int speedSetting = 0;
float currentSpeed = 0; // % of full ERPM currently applied (or at least, about to be applied)
bool commandFlag = false;
bool countedTapFlag = false;
bool triggerState = false;
int commandTaps = 0; // Number of taps received in a command
systime_t commandStartTime = 0; // Ie. use with millis()
systime_t startTapTime = 0; // Ie. use with millis()
bool tapTimeoutFlag = false;
bool commandTimeoutFlag = false;
bool reverseFlag = false; // New flag to indicate reverse mode

// TFM_DPV thread
static THD_FUNCTION(TFM_DPV_thread, arg);
static THD_WORKING_AREA(TFM_DPV_thread_wa, 2048); // 2kb stack for this thread

static virtual_timer_t tapTimer;
static virtual_timer_t commandTimer;

//Timer callbacks:
static void tapTimer_cb(void *arg) {
    (void)arg; // Mark parameter as unused
    tapTimeoutFlag = true;
}
/* 
static void commandTimer_cb(void *arg) {
    (void)arg; // Mark parameter as unused
    commandTimeoutFlag = true;
}
    */

void app_TFM_DPV_init(void) {
    // Set the Servo pin as an input with pulldown
    palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLDOWN);
    chThdCreateStatic(TFM_DPV_thread_wa, sizeof(TFM_DPV_thread_wa),
        NORMALPRIO, TFM_DPV_thread, NULL);
}

void adjustSpeedSettingUp(void) {
    speedSetting++;
    while (speedSetting < (int)NUM_SPEEDS && speeds[speedSetting] == 0) {
        speedSetting++;
    }
    if (speedSetting >= (int)NUM_SPEEDS) {
        speedSetting = (int)NUM_SPEEDS - 1;
    }
    currentSpeed = speeds[speedSetting];
}

void adjustSpeedSettingDown(void) {
    speedSetting--;
    if (speedSetting < 1) {
        speedSetting = 1;
    }
    while (speedSetting > 0 && speeds[speedSetting] == 0) {
        speedSetting--;
    }
    if (speedSetting < 1) {
        speedSetting = 1;
    }
    currentSpeed = speeds[speedSetting];
}

void handleTriggerPress(void) {
    if (speedSetting == 0 && triggerState) { // We are starting!
        speedSetting = START_SPEED_INDEX;

        // Just in case start setting is set to zero!!
        if (speeds[speedSetting] == 0) {
            while (speeds[speedSetting] == 0 && speedSetting < (int)NUM_SPEEDS - 1) {
                speedSetting++;
            }
        }
        currentSpeed = speeds[speedSetting];
    }
}

void handleTriggerRelease(void) {
    if (!triggerState && commandFlag == false && speedSetting != 0) {
        commandFlag = true;
        commandTaps = 0;
        countedTapFlag = false;

        tapTimeoutFlag = false;
        chVTSet(&tapTimer, MS2ST(TAP_DURATION_MS), tapTimer_cb, NULL);

        chThdSleepMilliseconds(DEBOUNCE_TIME_MS); // Debounce
    }
}

void handleTapTimeout(void) {
    if (tapTimeoutFlag) { // We have not moved the trigger in the last tapDuration milliseconds
        tapTimeoutFlag = false;
        chVTReset(&tapTimer);
        countedTapFlag = false;
        commandFlag = false;
        if (!triggerState && commandTaps == 0) { // Stop quickly
            speedSetting = 0;
            currentSpeed = 0;
            reverseFlag = false; // Reset reverse flag
        } else if (triggerState && commandTaps == 1) {
            if (speedSetting == 0) { // Shouldn't, but just in case
                speedSetting = START_SPEED_INDEX;
            } else {
                adjustSpeedSettingDown();
            }
        } else if (commandTaps == 4) { // Reverse at % of max ERPM
            reverseFlag = true; // Set reverse flag
        } else if (triggerState && commandTaps == 2) { // Shouldn't be greater, but just in case
            if (speedSetting == 0) { // Shouldn't, but just in case
                speedSetting = START_SPEED_INDEX;
            } else {
                adjustSpeedSettingUp();
            }
        }
    }
}

void writeSpeedToMotor(void) {
    if (reverseFlag) {
        float reverseErpm = -REVERSE_ERPM_PERCENT * MAX_ERPM;
        mc_interface_set_pid_speed(reverseErpm);
    } else if (currentSpeed != 0) {
        float targetErpm = (currentSpeed / 100) * MAX_ERPM;
        if (targetErpm > MAX_ERPM) {
            targetErpm = MAX_ERPM;
        }
        mc_interface_set_pid_speed(targetErpm);
    } else {
        mc_interface_release_motor();
    }
}

static THD_FUNCTION(TFM_DPV_thread, arg) {
    (void)arg;
    chRegSetThreadName("APP_TFM_DPV");

    chThdSleepMilliseconds(250);

    for (;;) {
        // Read trigger input ("servo" pin on VESC/FOCBOX)
        triggerState = palReadPad(HW_ICU_GPIO, HW_ICU_PIN);

        handleTriggerPress();
        handleTriggerRelease();

        if (triggerState && commandFlag == true && countedTapFlag == false) { //trigger has been released and pulled again so add to the tap count
            commandTaps++;
            if (commandTaps >= MAX_TAPS) { // Already got 4, no point waiting for any more
                tapTimeoutFlag = true;
            } else { // This sets the counted tap flag and resets the timer for the next tap
                countedTapFlag = true;
                tapTimeoutFlag = false;
                chVTSet(&tapTimer, MS2ST(TAP_DURATION_MS), tapTimer_cb, NULL);
                chThdSleepMilliseconds(DEBOUNCE_TIME_MS); // Debounce
            }
        }

        if (!triggerState && commandFlag == true && countedTapFlag == true) { // Trigger has been let go set up for next tap
            countedTapFlag = false;
            tapTimeoutFlag = false;
            chVTSet(&tapTimer, MS2ST(TAP_DURATION_MS), tapTimer_cb, NULL);
            chThdSleepMilliseconds(DEBOUNCE_TIME_MS); // Debounce
        }

        handleTapTimeout();
        writeSpeedToMotor();

        // Run this loop at 500Hz
        chThdSleepMilliseconds(2);

        // Reset the timeout
        timeout_reset();
    }
}

