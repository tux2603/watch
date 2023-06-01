// The frequency at which the charlieplexed LEDs are updated
// Recommended range: 288Hz - 5000Hz
// Default value: 720Hz
#define UPDATE_FREQUENCY 720

// The number of updates that the button state must be stable for before it is
//  accepted as an input. If UPDATE_FREQUENCY is increased dramtically, this
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