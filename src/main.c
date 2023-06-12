#define F_CPU 1000000

#include "config.h"
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>


// ##### Pin IO masks and general constants #####

#define PA1 0x02
#define PA2 0x04
#define PA3 0x08
#define PA6 0x40
#define PA7 0x80

#define RESET_COUNT (RESET_HOLD_TIME * UPDATE_FREQUENCY)
#define SET_BTN_THRESHOLD 0xC0
#define VIEW_BTN_THRESHOLD 0x40
#define BLINK_COUNT (uint16_t)(UPDATE_FREQUENCY / (2 * BLINK_FREQUENCY))

#define EEPROM_TIME_ADDR 0x00
#define EEPROM_TIME_SIZE 0x02
#define EEPROM_CAL_ADDR 0x02
#define EEPROM_CAL_SIZE 0x02
#define EEPROM_MODE_ADDR 0x04
#define EEPROM_MODE_SIZE 0x01


// ##### Charlieplexing matrix data #####

// Masks for each of the BCD nibbles
#define MM_L0_MASK 0x0001
#define MM_L1_MASK 0x0002
#define MM_L2_MASK 0x0004
#define MM_L3_MASK 0x0008
#define MM_H0_MASK 0x0010
#define MM_H1_MASK 0x0020
#define MM_H2_MASK 0x0040
#define HH_L0_MASK 0x0100
#define HH_L1_MASK 0x0200
#define HH_L2_MASK 0x0400
#define HH_L3_MASK 0x0800
#define HH_H0_MASK 0x1000

// Pins to write high for each of the bits in the BCD nibbles
#define MM_L0_HIGH PA3
#define MM_L1_HIGH PA7
#define MM_L2_HIGH PA2
#define MM_L3_HIGH PA2
#define MM_H0_HIGH PA2
#define MM_H1_HIGH PA6
#define MM_H2_HIGH PA7
#define HH_L0_HIGH PA7
#define HH_L1_HIGH PA6
#define HH_L2_HIGH PA3
#define HH_L3_HIGH PA6
#define HH_H0_HIGH PA3

// Pins to write low for each of the BCD nibbles
#define MM_L0_LOW PA2
#define MM_L1_LOW PA2
#define MM_L2_LOW PA7
#define MM_L3_LOW PA6
#define MM_H0_LOW PA3
#define MM_H1_LOW PA3
#define MM_H2_LOW PA6
#define HH_L0_LOW PA3
#define HH_L1_LOW PA2
#define HH_L2_LOW PA6
#define HH_L3_LOW PA7
#define HH_H0_LOW PA7


// ##### Various enums for data storage #####

// The display state simply displays the current time and waits for any button pressed.
// On entering any of the set states, the corresponding nibble is blinked on and off at 0.5Hz.
//  When the set button is pressed, the value in that nibble is incremented. When the view button
//  is pressed, the next nibble is made active for setting. Note that since hours are a bit odd,
//  HH_L and HH_H will be considered part of the same nibble for these purposes.
typedef enum {
    FSM_DISPLAY = 0x00,
    FSM_SET_HH = 0x01,
    FSM_SET_MM_H = 0x02,
    FSM_SET_MM_L = 0x03,
    FSM_BLANK = 0x04
} fsm_state_t;

typedef enum {
    LED_CHUNK_0 = 0x00,
    LED_CHUNK_1 = 0x01,
    LED_CHUNK_2 = 0x02,
    LED_CHUNK_3 = 0x03
} led_state_t;

typedef enum {
    BTN_NONE = 0x00,
    BTN_SET = 0x01,
    BTN_VIEW = 0x02
} btn_state_t;

typedef enum {
    MODE_STD = 0x00,
    MODE_CAL = 0x01
} mode_t;



// ##### LED data for smaller code size #####

const uint8_t led_high_pins[4] = {
    MM_L2_HIGH | MM_L3_HIGH | MM_H0_HIGH,
    MM_L0_HIGH | HH_L2_HIGH | HH_H0_HIGH,
    MM_H1_HIGH | HH_L1_HIGH | HH_L3_HIGH,
    MM_L1_HIGH | MM_H2_HIGH | HH_L0_HIGH};

const uint8_t led_low_pins[4][3] = {
    {MM_L2_LOW, MM_L3_LOW, MM_H0_LOW},
    {MM_L0_LOW, HH_L2_LOW, HH_H0_LOW},
    {MM_H1_LOW, HH_L1_LOW, HH_L3_LOW},
    {MM_L1_LOW, MM_H2_LOW, HH_L0_LOW}};

const uint16_t led_masks[4][3] = {
    {MM_L2_MASK, MM_L3_MASK, MM_H0_MASK},
    {MM_L0_MASK, HH_L2_MASK, HH_H0_MASK},
    {MM_H1_MASK, HH_L1_MASK, HH_L3_MASK},
    {MM_L1_MASK, MM_H2_MASK, HH_L0_MASK}};

const uint8_t led_blink_map[4][3] = {
    {FSM_SET_MM_L, FSM_SET_MM_L, FSM_SET_MM_H},
    {FSM_SET_MM_L, FSM_SET_HH, FSM_SET_HH},
    {FSM_SET_MM_H, FSM_SET_HH, FSM_SET_HH},
    {FSM_SET_MM_L, FSM_SET_MM_H, FSM_SET_HH}};


// ##### Global variables #####

mode_t active_mode; // The current mode of the device

volatile uint16_t current_bcd_time = 0x0000;          // The current time as a 16 bit BCD value, HH:MM
volatile uint16_t current_raw_time = 0x003b;          // The current time as minutes from 12:00
volatile led_state_t current_led_chunk = LED_CHUNK_0; // The current LED being lit
volatile fsm_state_t current_state = FSM_DISPLAY;     // The current state of the FSM
volatile btn_state_t last_btn_state = BTN_NONE;       // The current button state
volatile uint16_t debounce_cnt = 0;                   // The number of updates the button value has been stable
volatile uint16_t blink_cnt = 0;                      // The number of updates since the last blink
volatile uint8_t blink_state = 1;                     // The current state of the blinker
volatile uint16_t sar_step_size = 0x2000;             // The current SAR step size


// ##### Function declarations #####

/**
 * @brief Gets the current time as a 16 bit BCD value
 * 
 * @return The current time. The most significant byte is hours, and the least significant byte is minutes. 
 */
uint16_t get_time();


/**
 * @brief Increments the BCD time digits by a given amount. If the digit being
 *      incremented overflows, it will wrap around with no carry
 * 
 * @param hh The amount to increment the hours by
 * @param mm_h The amount to increment the tens digit of the minutes by
 * @param mm_l The amount to increment the ones digit of the minutes by
 * @return uint16_t The new BCD encoded time 
 */
uint16_t increment_time(uint8_t hh, uint8_t mm_h, uint8_t mm_l);


/**
 * @brief Saves the current time to EEPROM and resets the device
 * 
 */
void reset_device(mode_t reset_mode);


/**
 * @brief Initializes all registers, interrupts, etc, needed foe the watch to 
 *  function in standard operating mode
 */
void init_std();


/**
 * @brief Initializes all registers, interrupts, etc, needed foe the watch to 
 *  function in calibration mode
 */
void init_cal();



// ###########################################################################
// #####                          MAIN FUNCTION                          #####
// ###########################################################################


int main(void) {
    // The mode is stored at EEPROM_MODE_ADDR in the eeprom. This value
    //  is a 8 bit unsigned integer that is used to determine which mode
    //  the device should start in. If the value is MODE_STD, the device
    //  will start in standard operating mode. If the value is MODE_CAL,
    //  the device will start in calibration mode. If the value is anything
    //  else, the device will start in standard operating mode as a default.
    active_mode = eeprom_read_byte((uint8_t *)EEPROM_MODE_ADDR);

    if (active_mode == MODE_CAL)
        init_cal();
    else
        init_std();

    // Enable interrupts
    CPU_SREG |= CPU_I_bm;


    // Wait for the sleep bit to be set, then enter sleep mode
    for (;;) {
        while (!(SLPCTRL.CTRLA & SLPCTRL_SEN_bm)) continue;
        asm("SLEEP");
    }
}



// ###########################################################################
// #####                         INIT FUNCTIONS                          #####
// ###########################################################################


void init_std() {

    // ##### Configure clocks #####
    // OSC20M will operate in 16MHz mode. CLK_MAIN and CLK_PER will both use
    //  the correct prescaler option to hit F_CPU Hz
    CCP = CCP_IOREG_gc;                            // Allow access to protected registers
    CLKCTRL.MCLKCTRLA |= CLKCTRL_CLKSEL_OSC20M_gc; // Set the main clock to OSC20M
    CCP = CCP_IOREG_gc;                            // Allow access to protected registers
    CLKCTRL.MCLKCTRLB |= CLKCTRL_PDIV_16X_gc;      // Set the main clock prescaler to /16
    CCP = CCP_IOREG_gc;                            // Allow access to protected registers
    CLKCTRL.MCLKCTRLB |= CLKCTRL_PEN_bm;           // Enable the main clock prescaler
    CCP = CCP_IOREG_gc;                            // Allow access to protected registers
    CLKCTRL.MCLKLOCK |= CLKCTRL_LOCKEN_bm;         // Lock the main clock registers
    CCP = CCP_IOREG_gc;                            // Allow access to protected registers
    CLKCTRL.OSC32KCTRLA |= CLKCTRL_RUNSTDBY_bm;    // Enable the 32kHz oscillator in standby mode


    // ##### Load RTC calibration data from EEPROM #####
    // RTC period offset is stored at EEPROM_CAL_ADDR in the eeprom. This value
    //  is a 16 bit signed integer that is added to base value of the RTC
    //  period register to help account for clock drift.
    uint16_t rtc_period = eeprom_read_word((uint16_t *)EEPROM_CAL_ADDR);


    // ##### Configure RTC #####
    // The RTC will trigger an overflow every minute and trigger an interrupt
    //  that will wake the device from standby if necessary and increment the
    //  raw time value
    while (RTC.STATUS & RTC_CTRLABUSY_bm) continue;         // Wait for the RTC CTRLA register to be ready to be configured
    RTC.CTRLA |= RTC_PRESCALER_DIV128_gc | RTC_RUNSTDBY_bm; // Set the RTC prescaler to 128 and enable it in standby mode
    RTC.CLKSEL |= RTC_CLKSEL_INT32K_gc;                     // Set the RTC clock source to the 32kHz oscillator
    while (RTC.STATUS & RTC_PERBUSY_bm) continue;           // Wait for the RTC PER register to be ready to be configured
    RTC.PER = rtc_period;                                   // Set the RTC period to the calibrated value
    RTC.INTCTRL |= RTC_OVF_bm;                              // Enable the RTC overflow interrupt
    while (RTC.STATUS & RTC_CTRLABUSY_bm) continue;         // Wait for the RTC CTRLA register to be ready to be configured
    RTC.CTRLA |= RTC_RTCEN_bm;                              // Enable the RTC


    // ##### Configure CPUINT #####
    // TODO: The window comparison doesn't seem to be working
    // Five interrupts will be used:
    //  - TCA overflow will trigger the device to enter standby, and has the highest priority
    //  - TCB overflow will trigger the LEDs to update
    //  - ADC conversion will trigger the FSM to update
    //  - ADC window will reset the TCA CNT register to 0
    //  - A low level on PA1 will trigger the device to wake up
    CCP = CCP_IOREG_gc;                 // Allow access to protected registers
    CPUINT.CTRLA |= CPUINT_LVL0RR_bm;   // Enable round-robin scheduling
    CPUINT.LVL1VEC = TCA0_OVF_vect_num; // Set the TCA overflow interrupt to priority level 1


    // ##### Configure EVSYS #####
    // When TCB overflows, and event will be generated that will trigger the ADC to begin a conversion
    EVSYS.SYNCCH0 = EVSYS_SYNCCH0_TCB0_gc;          // Set the TCB overflow event to trigger sync channel 0
    EVSYS.ASYNCUSER1 = EVSYS_ASYNCUSER9_SYNCCH0_gc; // Set the ADC to be triggered by sync channel 0


    // ##### Configure TCA #####
    // TCA0 will trigger the device to enter sleep mode after SLEEP_TIMEOUT seconds of inactivity
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV1024_gc;    // Set the TCA clock source to CLK_PER/1024
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_WGMODE_NORMAL_gc;     // Set the TCA waveform generation mode to normal
    TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm;             // Enable the TCA overflow interrupt
    TCA0.SINGLE.PER = (SLEEP_TIMEOUT * F_CPU) / 1024 - 1; // Set the TCA period to SLEEP_TIMEOUT seconds
    TCA0.SINGLE.CNT = 0x0000;                             // Set the TCA count to 0
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;            // Enable the TCA


    // ##### Configure TCB #####
    // TCB will trigger the LED update interrupt at UPDATE_FREQUENCY Hz
    TCB0.INTCTRL |= TCB_CAPT_bm;              // Enable the TCB capture interrupt
    TCB0.CCMP = F_CPU / UPDATE_FREQUENCY - 1; // Set TCB's compare value to trigger at UPDATE_FREQUENCY Hz
    TCB0.CTRLA |= TCB_ENABLE_bm;              // Set the TCB clock source to TCA, and enable the TCB


    // ##### Configure ADC #####
    // The device doesn't need the full resolution, so use 8-bit mode
    // The ADC will be triggered by TCB, and will trigger an interrupt when the conversion is complete
    // The ADC will trigger another interrupt when the voltage falls below 0.75*Vdd
    ADC0.CTRLA |= ADC_RESSEL_8BIT_gc;   // Set the ADC resolution to 8-bit
    ADC0.CTRLC |= ADC_REFSEL0_bm;       // Set the ADC to use Vdd as reference
    ADC0.CTRLC |= ADC_PRESC_DIV8_gc;    // Set the ADC clock prescaler to CLK_PER/8, or 1MHz/8 = 125kHz
    ADC0.CTRLD |= ADC_INITDLY_DLY32_gc; // Set the ADC sampling delay to 32 CLK_ADC cycles after enabling
    ADC0.CTRLE |= ADC_WINCM_BELOW_gc;   // Set the ADC window mode to below
    ADC0.MUXPOS |= ADC_MUXPOS_AIN1_gc;  // Set the ADC input to PA1
    ADC0.EVCTRL |= ADC_STARTEI_bm;      // Set the ADC to be triggered by the event system
    ADC0.WINLT = SET_BTN_THRESHOLD;     // Set the ADC window comparator low threshold to ~0.75*Vdd
    ADC0.INTCTRL |= ADC_RESRDY_bm;      // Enable the ADC result ready interrupt
    ADC0.INTCTRL |= ADC_WCMP_bm;        // Enable the ADC window comparator interrupt
    ADC0.CTRLA |= ADC_ENABLE_bm;        // Enable the ADC


    // ##### Configure PORTA #####
    // The default state of all pins will be inputs with pull-ups disabled. PA2, PA3, PA6, and PA7
    //  are used to charlieplex a matrix of 12 LEDs, and will be toggled to digital outputs
    //  as appropriate in a timed interrupt.:hat is triggered on a low level.
    PORTA.DIRCLR = PA2 | PA3 | PA6 | PA7; // Set LED pins to inputs
    PORTA.DIRCLR = PA1;                   // Set the switch pin to an input
    PORTA.PIN1CTRL |= PORT_ISC_LEVEL_gc;  // Set the switch pin to trigger an interrupt on a low level


    // ##### Load time data from EEPROM #####

    // Load data from EEPROM if there was a software reset
    if (RSTCTRL.RSTFR & RSTCTRL_SWRF_bm) {
        // load the data and write it to the current time
        current_raw_time = eeprom_read_word((uint16_t *)EEPROM_TIME_ADDR);

        // parse the value into the current time variable and set the state to display
        current_bcd_time = get_time();
        current_state = FSM_DISPLAY;
    }

    // If the software reset flag wasn't set, the device most likely lost power
    //  and the time wasn't saved. in this case, the FSM should start in the
    //  FSM_SET_HH state so that the user can enter the correct time
    else {
        current_state = FSM_SET_HH;
    }
}


void init_cal() {

    // To calibrate the watch, a square wave with a period of ~468.75ms will be generated
    //  on PA2. This square wave will be measured with an external oscilloscope, and the
    //  RTC period register will be adjusted through SAR until the measured period is within
    //  10us of 468.75ms. Once the period is within tolerance, the value of the RTC period
    //  register will be saved to EEPROM and the device will be reset into standard operating
    //  mode.
    //
    // If the ULP oscillator is operating at it's nominal frequency of 32.768kHz, the resulting
    //  RTC period register value should be 0x3C00, so this value will be used as the default
    //  value to begin the SAR process. The ULP oscillator is only garunteed to be accurate to
    //  +/- 10% within the expected operational range of the watch, so realistically the RTC
    //  period register will vary between 0x3600 and 0x4200. This means that the SAR process
    //  should start with an initial value of 0x4000, initial step size of 0x1000, This will
    //  allow the SAR process to cover the entire expected range of the RTC period register
    //  in 10 steps.
    //
    // After calibration, the watch will initially have a drift of no more than 10us per
    //  468.75ms or ~20ppm. This means that the watch will gain or lose about one minute
    //  every month. Unfortunately it's likely that the frequency of the ULP oscillator will
    //  drift over time as the temperature changes or the battery discharges, so the actual
    //  accuracy of the watch will likely be significantly worse than this estimate. One
    //  method to improve the accuracy of the watch would be to calibrate the RTC period
    //  for multiple different battery voltages and use interpolation between the two
    //  nearest calibration points to determine the correct RTC period for the current
    //  battery voltage.


    // ##### Configure clocks #####
    // OSC20M will operate in 16MHz mode. At 1MHz, the maximum delay between the RTC
    //  overflowing and the interrupt being triggered will be 1us, which is well below the
    //  +/- 10us tolerance that will be used for calibration.
    CCP = CCP_IOREG_gc;                            // Allow access to protected registers
    CLKCTRL.MCLKCTRLA |= CLKCTRL_CLKSEL_OSC20M_gc; // Set the main clock to OSC20M
    CCP = CCP_IOREG_gc;                            // Allow access to protected registers
    CLKCTRL.MCLKCTRLB |= CLKCTRL_PDIV_16X_gc;      // Set the main clock prescaler to /16
    CCP = CCP_IOREG_gc;                            // Allow access to protected registers
    CLKCTRL.MCLKCTRLB |= CLKCTRL_PEN_bm;           // Enable the main clock prescaler
    CCP = CCP_IOREG_gc;                            // Allow access to protected registers
    CLKCTRL.MCLKLOCK |= CLKCTRL_LOCKEN_bm;         // Lock the main clock registers


    // ##### Configure RTC #####
    // The RTC will be used to trigger an interrupt to generate a square wave on PA2 with a
    //  period of ~1.875 seconds. The actual period of the square wave will be measured with
    //  an external oscilloscope, and the RTC period will be adjusted through SAR until the
    //  period is 1.875 seconds, +/- 10us.
    while (RTC.STATUS & RTC_CTRLABUSY_bm) continue; // Wait for the RTC CTRLA register to be ready to be configured
    RTC.CTRLA |= RTC_PRESCALER_DIV1_gc;             // Set the RTC prescaler to 1
    RTC.CLKSEL |= RTC_CLKSEL_INT32K_gc;             // Set the RTC clock source to the 32kHz oscillator
    while (RTC.STATUS & RTC_PERBUSY_bm) continue;   // Wait for the RTC PER register to be ready to be configured
    RTC.PER = 0x4000u;                              // Set the RTC period to 500ms
    while (RTC.STATUS & RTC_CMPBUSY_bm) continue;   // Wait for the RTC CMP register to be ready to be configured
    RTC.CMP = 0x2000u;                              // Set the RTC compare to 250ms
    RTC.INTCTRL |= RTC_OVF_bm;                      // Enable the RTC overflow interrupt
    RTC.INTCTRL |= RTC_CMP_bm;                      // Enable the RTC compare interrupt
    while (RTC.STATUS & RTC_CTRLABUSY_bm) continue; // Wait for the RTC CTRLA register to be ready to be configured
    RTC.CTRLA |= RTC_RTCEN_bm;                      // Enable the RTC



    // ##### Configure CPUINT #####
    // Two interrupts will be used:
    //  - ADC conversion will perform SAR updates
    //  - RTC overflow and compare will trigger the square wave generation
    CCP = CCP_IOREG_gc;                // Allow access to protected registers
    CPUINT.CTRLA |= CPUINT_LVL0RR_bm;  // Enable round-robin scheduling
    CPUINT.LVL1VEC = RTC_CNT_vect_num; // Set the RTC overflow and compare interrupt to priority level 1


    // ##### Configure EVSYS #####
    // When TCB overflows, and event will be generated that will trigger the ADC to begin a conversion
    EVSYS.SYNCCH0 = EVSYS_SYNCCH0_TCB0_gc;          // Set the TCB overflow event to trigger sync channel 0
    EVSYS.ASYNCUSER1 = EVSYS_ASYNCUSER9_SYNCCH0_gc; // Set the ADC to be triggered by sync channel 0


    // ##### Configure TCB #####
    // TCB will trigger the ADC to check the button states at UPDATE_FREQUENCY Hz
    TCB0.CCMP = F_CPU / UPDATE_FREQUENCY - 1; // Set TCB's compare value to trigger at UPDATE_FREQUENCY Hz
    TCB0.CTRLA |= TCB_ENABLE_bm;              // Set the TCB clock source to TCA, and enable the TCB


    // ##### Configure ADC #####
    // The device doesn't need the full resolution, so use 8-bit mode
    // The ADC will be triggered by TCB, and will trigger an interrupt when the conversion is complete
    ADC0.CTRLA |= ADC_RESSEL_8BIT_gc;   // Set the ADC resolution to 8-bit
    ADC0.CTRLC |= ADC_REFSEL0_bm;       // Set the ADC to use Vdd as reference
    ADC0.CTRLC |= ADC_PRESC_DIV8_gc;    // Set the ADC clock prescaler to CLK_PER/8, or 1MHz/8 = 125kHz
    ADC0.CTRLD |= ADC_INITDLY_DLY32_gc; // Set the ADC sampling delay to 32 CLK_ADC cycles after enabling
    ADC0.MUXPOS |= ADC_MUXPOS_AIN1_gc;  // Set the ADC input to PA1
    ADC0.EVCTRL |= ADC_STARTEI_bm;      // Set the ADC to be triggered by the event system
    ADC0.INTCTRL |= ADC_RESRDY_bm;      // Enable the ADC result ready interrupt
    ADC0.CTRLA |= ADC_ENABLE_bm;        // Enable the ADC


    // ##### Configure PORTA #####
    // PA1 will be used as an analog input, PA2 will be used as a digital output
    //  for the square wave, and PA3 will be a digital output with a constant value
    //  of Vdd so that the bottom right LED will light up when the square wave is active
    PORTA.DIRCLR = PA1; // Set the switch pin to an input
    PORTA.DIRSET = PA2; // Set the square wave pin to an output
    PORTA.OUTCLR = PA2; // Set the square wave pin to low
    PORTA.DIRSET = PA3; // Set the constant Vdd pin to an output
    PORTA.OUTSET = PA3; // Set the constant Vdd pin to high
}



// ###########################################################################
// #####                         HELPER FUNCTIONS                        #####
// ###########################################################################


// ##### Get the current time for display #####

uint16_t get_time() {
    // Time is stored as the number of seconds elapsed since 12:00
    // Due to hardware limitations, this value will reset every 12 hours
    uint16_t time = current_raw_time;

    // Get all the various chunks of the time
    uint8_t minutes = time % 60;
    uint8_t hours = time / 60;

    // Make sure that 12:00 is displayed as 12:00, not 00:00
    if (hours == 0) hours = 12;

    // Convert the time to BCD
    uint8_t bcd_minutes = ((minutes / 10) << 4) | (minutes % 10);
    uint8_t bcd_hours = ((hours / 10) << 4) | (hours % 10);
    uint16_t bcd_time = (bcd_hours << 8) | bcd_minutes;
    return bcd_time;
}


// ##### Increment the RTC counter #####

uint16_t increment_time(uint8_t hh, uint8_t mm_h, uint8_t mm_l) {
    // Get the BCD value of the current time
    uint16_t time = get_time();

    // Get the values for all of the individual time chunks
    uint8_t current_hh = 10 * ((time >> 12) & 0x0F) + ((time >> 8) & 0x0F);
    uint8_t current_mm_h = (time >> 4) & 0x0F;
    uint8_t current_mm_l = time & 0x0F;

    // Increment the time by the requested amount
    current_hh += hh;
    current_mm_h += mm_h;
    current_mm_l += mm_l;

    // Handle overflow, but don't carry over to the next chunk
    // Note that 12:00 is represented as 00:00 internally
    if (current_mm_l > 9) current_mm_l = 0;
    if (current_mm_h > 5) current_mm_h = 0;
    if (current_hh > 11) current_hh -= 12;

    // Write the time back to the raw time variable
    current_raw_time = current_hh * 60 + current_mm_h * 10 + current_mm_l;

    // Reset the blinker
    blink_cnt = 0;
    blink_state = 1;

    // Return the new time
    return get_time();
}


// ##### Trigger a device reset #####

void reset_device(mode_t reset_mode) {
    // Save the current time to EEPROM
    uint16_t time = current_raw_time;
    eeprom_write_word((uint16_t *)EEPROM_TIME_ADDR, time);
    while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm) continue;

    // If the mode requested to be reset to is not the same as the current mode, write the new mode to EEPROM
    if (reset_mode != active_mode) {
        eeprom_write_byte((uint8_t *)EEPROM_MODE_ADDR, reset_mode);
        while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm) continue;
    }

    // Trigger a software reset
    CCP = CCP_IOREG_gc;
    RSTCTRL.SWRR = RSTCTRL_SWRE_bm;
}



// ###########################################################################
// #####                   Interrupt Service Routines                   ######
// ###########################################################################


// ##### TCA0 Overflow Interrupt #####

ISR(TCA0_OVF_vect) {
    // When timer A overflows the device should enter sleep mode
    // To enter sleep mode, perform the following actions:
    //  - All four LED pins will need to be set to LOW and to be inputs
    //  - TCA0 will need to be disabled
    //  - TCB0 will need to be disabled
    //  - ADC0 will need to be disabled
    //  - The sleep state will need to be set to standby
    //  - The sleep bit will need to be set
    // Once the sleep bit is set and the interrupt exits, the busy loop in main will call the
    //  sleep instruction to actually enter standby mode

    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;   // Clear the interrupt flag
    if (current_state != FSM_DISPLAY) return;   // Only enter sleep if not on one of the set modes
    current_state == FSM_BLANK;                 // Set the state to blank
    PORTA.OUTCLR = PA2 | PA3 | PA6 | PA7;       // Set all four LED pins to low
    PORTA.DIRCLR = PA2 | PA3 | PA6 | PA7;       // Set all four LED pins to be inputs
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm; // Disable TCA0
    TCB0.CTRLA &= ~TCB_ENABLE_bm;               // Disable TCB0
    ADC0.CTRLA &= ~ADC_ENABLE_bm;               // Disable ADC0
    PORTA.PIN1CTRL |= PORT_ISC_LEVEL_gc;        // Set the switch pin to trigger an interrupt on low level
    SLPCTRL.CTRLA |= SLPCTRL_SMODE_STDBY_gc;    // Set the sleep mode to standby
    SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;            // Set the sleep bit
}


// ##### TCB0 Overflow Interrupt #####

ISR(TCB0_INT_vect) {
    // Clear the interrupt flag
    TCB0.INTFLAGS = TCB_CAPT_bm;

    // Set all four LED pins to low and to be inputs
    PORTA.OUTCLR = PA2 | PA3 | PA6 | PA7;
    PORTA.DIRCLR = PA2 | PA3 | PA6 | PA7;

    // Only power LEDs if the current FSM state isn't FSM_BLANK
    if (current_state == FSM_BLANK) return;

    // Figure out if we need to be blinking right now
    if (++blink_cnt >= BLINK_COUNT) {
        blink_cnt = 0;
        blink_state = !blink_state;
    }

    // LEDs are updated in chunks of three. Set the high pin for the current
    //  chunk to be an output and write a 1 to the set bit. Then set all of the low
    //  pins for the current chunk to be outputs only if the corresponding bit is set
    //  in the current time. Finally, set the current LED to the next chunk
    PORTA.DIRSET = led_high_pins[current_led_chunk];
    PORTA.OUTSET = led_high_pins[current_led_chunk];

    if (current_bcd_time & led_masks[current_led_chunk][0] && (blink_state || current_state != led_blink_map[current_led_chunk][0]))
        PORTA.DIRSET = led_low_pins[current_led_chunk][0];

    if (current_bcd_time & led_masks[current_led_chunk][1] && (blink_state || current_state != led_blink_map[current_led_chunk][1]))
        PORTA.DIRSET = led_low_pins[current_led_chunk][1];

    if (current_bcd_time & led_masks[current_led_chunk][2] && (blink_state || current_state != led_blink_map[current_led_chunk][2]))
        PORTA.DIRSET = led_low_pins[current_led_chunk][2];

    switch (current_led_chunk) {
        case LED_CHUNK_0:
            current_led_chunk = LED_CHUNK_1;
            break;
        case LED_CHUNK_1:
            current_led_chunk = LED_CHUNK_2;
            break;
        case LED_CHUNK_2:
            current_led_chunk = LED_CHUNK_3;
            break;
        case LED_CHUNK_3:
            current_led_chunk = LED_CHUNK_0;
            current_bcd_time = get_time();
            break;
    }
}


// ##### ADC Conversion Ready Interrupt #####

ISR(ADC0_RESRDY_vect) {
    // Clear the interrupt flag
    ADC0.INTFLAGS = ADC_RESRDY_bm;

    // Figure out what button if any is currently pressed
    uint16_t adc_result = ADC0.RES;
    btn_state_t btn_state = BTN_NONE;

    if (adc_result < VIEW_BTN_THRESHOLD)
        btn_state = BTN_VIEW;
    else if (adc_result < SET_BTN_THRESHOLD)
        btn_state = BTN_SET;

    // If no button is pressed, reset the debounce counter and exit
    if (btn_state == BTN_NONE) {
        last_btn_state = BTN_NONE;
        debounce_cnt = 0;
        return;
    }

    // If the button state has changed, reset the debounce counter and exit
    if (btn_state != last_btn_state) {
        last_btn_state = btn_state;
        debounce_cnt = 0;
        return;
    }

    // If the button state is stable, increment the debounce counter
    debounce_cnt++;

    // If the debounce counter has reached the debounce threshold, we have a valid input.
    //  update the FSM based on said input
    if (debounce_cnt == DEBOUNCE_COUNT && active_mode == MODE_STD) {
        switch (current_state) {
            // If the set button is pressed in the display state, enter the set hour state
            case FSM_DISPLAY:
            case FSM_BLANK:
                if (btn_state == BTN_SET)
                    current_state = FSM_SET_HH;
                else if (btn_state == BTN_VIEW)
                    current_state = FSM_DISPLAY;
                break;

            // If the set button is pressed in the hour state, enter the first set minute state
            // If the view button is pressed in the set hour state, increment the time by one hour
            case FSM_SET_HH:
                if (btn_state == BTN_SET)
                    current_state = FSM_SET_MM_H;
                else if (btn_state == BTN_VIEW)
                    increment_time(1, 0, 0);
                break;

            // If the set button is pressed in the high minute state, enter the low minute state
            // If the view button is pressed in the low minute state, increment the time by ten minutes
            case FSM_SET_MM_H:
                if (btn_state == BTN_SET)
                    current_state = FSM_SET_MM_L;
                else if (btn_state == BTN_VIEW)
                    increment_time(0, 1, 0);
                break;

            // If the set button is pressed in the low minute state, enter the display state
            // If the view button is pressed in the low minute state, increment the time by one minute
            case FSM_SET_MM_L:
                if (btn_state == BTN_SET) {
                    // Start counting from the beginning of the current minute and then re-enter display mode
                    while (RTC.STATUS & RTC_CNTBUSY_bm) continue;
                    RTC.CNT = 0x0000;
                    current_state = FSM_DISPLAY;
                }

                else if (btn_state == BTN_VIEW)
                    increment_time(0, 0, 1);
                break;
        }
    }

    // If the debounce counter has reached the debounce threshold, we have a valid input.
    //  Perform the next SAR step based on said input
    if (debounce_cnt == DEBOUNCE_COUNT && active_mode == MODE_CAL) {
        if (btn_state == BTN_VIEW) {
            uint16_t rtc_period = RTC.PER - sar_step_size;
            while (RTC.STATUS & RTC_PERBUSY_bm) continue;
            RTC.PER = rtc_period;
            while (RTC.STATUS & RTC_CMPBUSY_bm) continue;
            RTC.CMP = rtc_period >> 1;
        }

        else if (btn_state == BTN_SET) {
            uint16_t rtc_period = RTC.PER + sar_step_size;
            while (RTC.STATUS & RTC_PERBUSY_bm) continue;
            RTC.PER = rtc_period;
            while (RTC.STATUS & RTC_CMPBUSY_bm) continue;
            RTC.CMP = rtc_period >> 1;
        }

        sar_step_size >>= 1;

        if (sar_step_size == 0) {
            // Save the new RTC period to EEPROM
            uint16_t rtc_period = RTC.PER;
            eeprom_write_word((uint16_t *)EEPROM_CAL_ADDR, rtc_period);
            while (NVMCTRL.STATUS & NVMCTRL_EEBUSY_bm) continue;

            // Reset the device into standard mode
            reset_device(MODE_STD);
        }
    }

    // Reset functionality works the same no matter what mode the device is in
    if (debounce_cnt == RESET_COUNT) {
        if (btn_state == BTN_VIEW)
            reset_device(MODE_STD);
        if (btn_state == BTN_SET)
            reset_device(MODE_CAL);
    }
}


// ##### ADC Window Comparator Interrupt #####

ISR(ADC0_WCOMP_vect) {
    // Whenever the window comparator triggers, we know that one of the
    //  buttons was pressed. We don't really care about which one, since
    //  either button press will be used to reset the standby timer.
    ADC0.INTFLAGS = ADC_WCMP_bm;                       // Clear the interrupt flag
    TCA0.SINGLE.CTRLESET |= TCA_SINGLE_CMD_RESTART_gc; // Reset the TCA0 counter
}


// ##### PORTA Low Level Interrupt #####

ISR(PORTA_PORT_vect) {
    // Clear the interrupt flag
    PORTA.INTFLAGS = PORT_INT1_bm;

    ADC0.CTRLA |= ADC_ENABLE_bm;               // Enable ADC0
    TCB0.CTRLA |= TCB_ENABLE_bm;               // Enable TCB0
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm; // Enable TCA0
    PORTA.PIN1CTRL &= ~PORT_ISC_gm;            // Set the switch pin to not trigger an interrupt
    SLPCTRL.CTRLA &= ~SLPCTRL_SEN_bm;          // Clear the sleep bit
    current_state = FSM_DISPLAY;               // Initialize the FSM
    current_led_chunk = LED_CHUNK_0;           // Initialize the LED
    current_bcd_time = get_time();             // Initialize the time
    last_btn_state = BTN_NONE;                 // Initialize the last button
    debounce_cnt = 0;                          // Initialize the button counter
}


// ##### RTC Overflow and Compare Interrupt #####

ISR(RTC_CNT_vect) {
    // Clear the interrupt flag
    RTC.INTFLAGS = RTC_OVF_bm | RTC_CMP_bm;

    if (active_mode == MODE_CAL) {
        // Toggle the square wave pin
        PORTA.OUTTGL = PA2;
    }

    else {
        current_raw_time++;                                   // Increment the raw time value
        if (current_raw_time >= 0x02D0) current_raw_time = 0; // Reset the raw time value if it's greater than 720
        current_bcd_time = get_time();                        // Update the BCD time value
    }
}