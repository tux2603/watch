// The frequency at which the charlieplexed LEDs are updated
// Recommended range: 96Hz - 5000Hz
// Default value: 480Hz
#define UPDATE_FREQUENCY 480

// The frequency at which the currently selected digit will be blinked
//  while the time is being set
// Recommended range: 0.25Hz - 2Hz
// Default value: 1Hz
#define BLINK_FREQUENCY 1

// The number of updates that the button state must be stable for before it is
//  accepted as an input. If UPDATE_FREQUENCY is increased dramatically, this
//  value may also need to be increased to prevent double inputs.
// Recommended range 5-500
// Default value: 50
#define DEBOUNCE_COUNT 50

// The amount of time that a button must be held down before the watch is reset
// Recommended range: 1s - 10s
// Default value: 3s
#define RESET_HOLD_TIME 3

// The amount of time that the watch will remain awake after the last button
//  press before going to sleep.
// Recommended range: 1s - 60s
// Default value: 10s
#define SLEEP_TIMEOUT 10

// The revision of the PCB being programmed. Different PCB versions have different
//  pinouts and for the LED matrix.
// Valid values: 1, 2
#define PCB_REVISION 2

// If defined, the watch will include code to enter and run calibration mode.
//  If you are using a low-memory chip, you can comment out this line to save
//  around 500-600 bytes of program memory.
#define ENABLE_CALIBRATION_MODE