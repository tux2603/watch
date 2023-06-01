#define F_CPU 1000000

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include "config.h"


// ##### Pin IO masks and general constants #####

#define PA1 0x20
#define PA2 0x40
#define PA3 0x80
#define PA6 0x40
#define PA7 0x80

#define RESET_COUNT (RESET_HOLD_TIME * 3)
#define SET_BTN_THRESHOLD 0xC0
#define VIEW_BTN_THRESHOLD 0x40

#define EEPROM_TIME_ADDR 0x00


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
// TODO: Check the visibility/usability of the 0.5Hz blink. Maybe some PWM style duty cycle would be more appropriate?
typedef enum {
    FSM_DISPLAY = 0x00,
    FSM_SET_HH = 0x01,
    FSM_SET_MM_H = 0x02,
    FSM_SET_MM_L = 0x03
} fsm_state_t;

typedef enum {
    LED_MM_L0 = 0x00,
    LED_MM_L1 = 0x01,
    LED_MM_L2 = 0x02,
    LED_MM_L3 = 0x03,
    LED_MM_H0 = 0x04,
    LED_MM_H1 = 0x05,
    LED_MM_H2 = 0x06,
    LED_HH_L0 = 0x07,
    LED_HH_L1 = 0x08,
    LED_HH_L2 = 0x09,
    LED_HH_L3 = 0x0A,
    LED_HH_H0 = 0x0B
} led_state_t;

typedef enum {
    BTN_NONE = 0x00,
    BTN_SET = 0x01,
    BTN_VIEW = 0x02
} btn_state_t;


// ##### Volatile global variables #####

volatile uint16_t current_time = 0x0000;          // The current time as a 16 bit BCD value, HH:MM
volatile led_state_t current_led = LED_HH_L0;     // The current LED being lit
volatile fsm_state_t current_state = FSM_DISPLAY; // The current state of the FSM
volatile btn_state_t last_btn_state = BTN_NONE;   // The current button state
volatile uint16_t debounce_cnt = 0;               // The number of updates the button value has been stable


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
void reset_device();



// ###########################################################################
// #####                          MAIN FUNCTION                          #####
// ###########################################################################


int main(void) {

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


    // ##### Configure RTC #####
    // The RTC will be used to keep track of the elapsed time in seconds since 12:00.
    //  Once the time hits 12:00, it will reset to 0. This will be used to keep track
    //  of the time even when the device is in sleep mode.
    while (RTC.STATUS & RTC_CTRLABUSY_bm) continue;           // Wait for the RTC CTRLA register to be ready to be configured
    RTC.CTRLA |= RTC_PRESCALER_DIV32768_gc | RTC_RUNSTDBY_bm; // Set the RTC prescaler to 32768 and enable it in standby mode
    RTC.CLKSEL |= RTC_CLKSEL_INT32K_gc;                       // Set the RTC clock source to the 32kHz oscillator
    while (RTC.STATUS & RTC_CNTBUSY_bm) continue;             // Wait for the RTC CNT register to be ready to be configured
    RTC.CNT = 0x0000;                                         // Set the RTC count to 0
    while (RTC.STATUS & RTC_PERBUSY_bm) continue;             // Wait for the RTC PER register to be ready to be configured
    RTC.PER = 43199u;                                         // Set the RTC period to 12 hours
    while (RTC.STATUS & RTC_CTRLABUSY_bm) continue;           // Wait for the RTC CTRLA register to be ready to be configured
    RTC.CTRLA |= RTC_RTCEN_bm;                                // Enable the RTC


    // ##### Configure CPUINT #####
    // Five interrupts will be used:
    //  - TCA overflow will trigger the device to enter standby, and has the highest priority
    //  - TCB overflow will trigger the LEDs to update
    //  - ADC conversion will trigger the FSM to update
    //  - ADC window will reset the TCA CNT register to 0
    //  - Falling edge on PA1 will trigger the device to wake up
    CCP = CCP_IOREG_gc;                 // Allow access to protected registers
    CPUINT.CTRLA |= CPUINT_LVL0RR_bm;   // Enable round-robin scheduling
    CPUINT.LVL1VEC = TCA0_OVF_vect_num; // Set the TCA overflow interrupt to priority level 1


    // ##### Configure EVSYS #####
    // When TCB overflows, and event will be generated that will trigger the ADC to begin a conversion
    EVSYS.SYNCCH0 = EVSYS_SYNCCH0_TCB0_gc;          // Set the TCB overflow event to trigger sync channel 0
    EVSYS.ASYNCUSER1 = EVSYS_ASYNCUSER9_SYNCCH0_gc; // Set the ADC to be triggered by sync channel 0


    // ##### Configure TCA #####
    // TCA0 will trigger the device to enter sleep mode after SLEEP_TIMEOUT seconds of inactivity
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV1024_gc; // Set the TCA clock source to CLK_PER/1024
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_WGMODE_NORMAL_gc;  // Set the TCA waveform generation mode to normal
    TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm;          // Enable the TCA overflow interrupt
    TCA0.SINGLE.PER = (SLEEP_TIMEOUT * F_CPU) / 1024 - 1;  // Set the TCA period to SLEEP_TIMEOUT seconds
    TCA0.SINGLE.CNT = 0x0000;                          // Set the TCA count to 0
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;         // Enable the TCA


    // ##### Configure TCB #####
    // TCB will trigger the LED update interrupt at UPDATE_FREQUENCY Hz
    TCB0.INTCTRL |= TCB_CAPT_bm;         // Enable the TCB capture interrupt
    TCB0.CCMP = F_CPU / UPDATE_FREQUENCY - 1; // Set TCB's compare value to trigger at UPDATE_FREQUENCY Hz
    TCB0.CTRLA |= TCB_ENABLE_bm;         // Set the TCB clock source to TCA, and enable the TCB


    // ##### Configure ADC #####
    // The device doesn't need the full resolution, so use 8-bit mode
    // The ADC will be triggered by TCB, and will trigger an interrupt when the conversion is complete
    // The ADC will trigger another interrupt when the voltage falls below 0.75*Vdd
    ADC0.CTRLA |= ADC_RESSEL_8BIT_gc;   // Set the ADC resolution to 8-bit
    ADC0.CTRLC |= ADC_REFSEL0_bm;       // Set the ADC to use Vdd as reference
    ADC0.CTRLC |= ADC_PRESC_DIV2_gc;    // Set the ADC clock prescaler to CLK_PER/2, or 250kHz/2 = 125kHz
    ADC0.CTRLD |= ADC_INITDLY_DLY32_gc; // Set the ADC sampling delay to 32 CLK_ADC cycles after enabling
    ADC0.CTRLE |= ADC_WINCM_BELOW_gc;   // Set the ADC window mode to below
    ADC0.MUXPOS |= ADC_MUXPOS_AIN1_gc;  // Set the ADC input to PA1
    ADC0.EVCTRL |= ADC_STARTEI_bm;      // Set the ADC to be triggered by the event system
    ADC0.INTCTRL |= ADC_RESRDY_bm;      // Enable the ADC result ready interrupt
    ADC0.INTCTRL |= ADC_WCMP_bm;        // Enable the ADC window comparator interrupt
    ADC0.WINLT = SET_BTN_THRESHOLD;     // Set the ADC window comparator low threshold to ~0.75*Vdd
    ADC0.CTRLA |= ADC_ENABLE_bm;        // Enable the ADC


    // ##### Configure PORTA #####
    // The default state of all pins will be inputs with pull-ups disabled. PA2, PA3, PA6, and PA7
    //  are used to charlieplex a matrix of 12 LEDs, and will be toggled to digital outputs
    //  as appropriate in a timed interrupt.
    // PA1 will be a digital input with an attached interrupt that is triggered on its falling edge.
    PORTA.DIRCLR = PA2 | PA3 | PA6 | PA7;  // Set LED pins to inputs
    PORTA.DIRCLR = PA1;                    // Set the switch pin to an input
    PORTA.PIN1CTRL |= PORT_ISC_FALLING_gc; // Set the switch pin to trigger an interrupt on a falling edge


    // ##### Load time data from EEPROM #####

    // Load data from EEPROM if there was a software reset
    if (RSTCTRL.RSTFR & RSTCTRL_SWRF_bm) {
        // load the data and write it to the current time
        uint16_t saved_time = eeprom_read_word((uint16_t *)EEPROM_TIME_ADDR);

        // re-initialize the RTC with the time loaded from EEPROM
        while (RTC.STATUS & RTC_CNTBUSY_bm) continue;
        RTC.CNT = saved_time;

        // load the value into the current time variable and set the state to display
        current_time = get_time();
        current_state = FSM_DISPLAY;
    }

    // If the software reset flag wasn't set, the device most likely lost power
    //  and the time wasn't saved. in this case, the FSM should start in the
    //  FSM_SET_HH state so that the user can enter the correct time
    else {
        current_state = FSM_SET_HH;
    }


    // ##### Ready to go! Turn everything on #####

    // Enable interrupts
    SREG |= SREG_I;


    for (;;) {
        // Wait for the enter sleep bit to be set
        while (!(SLPCTRL.CTRLA & SLPCTRL_SEN_bm)) continue;
        asm("SLEEP"); // Go to sleep
    }
}



// ###########################################################################
// #####                         HELPER FUNCTIONS                        #####
// ###########################################################################


// ##### Get the current time for display #####

uint16_t get_time() {
    // Time is stored as the number of seconds elapsed since 12:00
    // Due to hardware limitations, this value will reset every 12 hours
    uint16_t time = RTC.CNT;

    // Get all the various chunks of the time
    uint8_t seconds = time % 60;
    uint8_t minutes = (time / 60) % 60;
    uint8_t hours = (time / 3600);

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
    if (current_hh > 11) current_hh = 0;

    // Convert the time back to seconds
    time = (current_hh * 3600) + (current_mm_h * 600) + (current_mm_l * 60);

    // Save the new time to the RTC
    while(RTC.STATUS & RTC_CNTBUSY_bm) continue;
    RTC.CNT = time;

    // Convert the new time back to BCD and return it
    if (current_hh == 0) current_hh = 12;
    time = (current_hh / 10) << 12 | (current_hh % 10) << 8;
    time |= (current_mm_h << 4) | current_mm_l;
    return time;
}


// ##### Trigger a device reset #####

void reset_device() {
    // Save the current time to EEPROM
    uint16_t time = RTC.CNT;
    eeprom_write_word((uint16_t *)EEPROM_TIME_ADDR, time);

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
    PORTA.OUTCLR = PA2 | PA3 | PA6 | PA7;       // Set all four LED pins to low
    PORTA.DIRCLR = PA2 | PA3 | PA6 | PA7;       // Set all four LED pins to be inputs
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm; // Disable TCA0
    TCB0.CTRLA &= ~TCB_ENABLE_bm;               // Disable TCB0
    ADC0.CTRLA &= ~ADC_ENABLE_bm;               // Disable ADC0
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


    // Figure out if we need to be blinking right now
    uint8_t blink = current_time & 0x00001;

    // For each LED, set the two required pins to outputs, then write the High and Low values.
    // On the final LED, update the current time
    switch (current_led) {
        case LED_MM_L0:
            if ((current_time & MM_L0_MASK) && (blink || (current_state != FSM_SET_MM_L))) {
                PORTA.DIRSET = MM_L0_LOW | MM_L0_HIGH;
                PORTA.OUTSET = MM_L0_HIGH;
                PORTA.OUTCLR = MM_L0_LOW;
            }
            current_led = LED_MM_L1;
            break;

        case LED_MM_L1:
            if ((current_time & MM_L1_MASK) && (blink || (current_state != FSM_SET_MM_L))) {
                PORTA.DIRSET = MM_L1_LOW | MM_L1_HIGH;
                PORTA.OUTSET = MM_L1_HIGH;
                PORTA.OUTCLR = MM_L1_LOW;
            }
            current_led = LED_MM_L2;
            break;

        case LED_MM_L2:
            if ((current_time & MM_L2_MASK) && (blink || (current_state != FSM_SET_MM_L))) {
                PORTA.DIRSET = MM_L2_LOW | MM_L2_HIGH;
                PORTA.OUTSET = MM_L2_HIGH;
                PORTA.OUTCLR = MM_L2_LOW;
            }
            current_led = LED_MM_L3;
            break;

        case LED_MM_L3:
            if ((current_time & MM_L3_MASK) && (blink || (current_state != FSM_SET_MM_L))) {
                PORTA.DIRSET = MM_L3_LOW | MM_L3_HIGH;
                PORTA.OUTSET = MM_L3_HIGH;
                PORTA.OUTCLR = MM_L3_LOW;
            }
            current_led = LED_MM_H0;
            break;

        case LED_MM_H0:
            if ((current_time & MM_H0_MASK) && (blink || (current_state != FSM_SET_MM_H))) {
                PORTA.DIRSET = MM_H0_LOW | MM_H0_HIGH;
                PORTA.OUTSET = MM_H0_HIGH;
                PORTA.OUTCLR = MM_H0_LOW;
            }
            current_led = LED_MM_H1;
            break;

        case LED_MM_H1:
            if ((current_time & MM_H1_MASK) && (blink || (current_state != FSM_SET_MM_H))) {
                PORTA.DIRSET = MM_H1_LOW | MM_H1_HIGH;
                PORTA.OUTSET = MM_H1_HIGH;
                PORTA.OUTCLR = MM_H1_LOW;
            }
            current_led = LED_MM_H2;
            break;

        case LED_MM_H2:
            if ((current_time & MM_H2_MASK) && (blink || (current_state != FSM_SET_MM_H))) {
                PORTA.DIRSET = MM_H2_LOW | MM_H2_HIGH;
                PORTA.OUTSET = MM_H2_HIGH;
                PORTA.OUTCLR = MM_H2_LOW;
            }
            current_led = LED_HH_L0;
            break;

        case LED_HH_L0:
            if ((current_time & HH_L0_MASK) && (blink || (current_state != FSM_SET_HH))) {
                PORTA.DIRSET = HH_L0_LOW | HH_L0_HIGH;
                PORTA.OUTSET = HH_L0_HIGH;
                PORTA.OUTCLR = HH_L0_LOW;
            }
            current_led = LED_HH_L1;
            break;

        case LED_HH_L1:
            if ((current_time & HH_L1_MASK) && (blink || (current_state != FSM_SET_HH))) {
                PORTA.DIRSET = HH_L1_LOW | HH_L1_HIGH;
                PORTA.OUTSET = HH_L1_HIGH;
                PORTA.OUTCLR = HH_L1_LOW;
            }
            current_led = LED_HH_L2;
            break;

        case LED_HH_L2:
            if ((current_time & HH_L2_MASK) && (blink || (current_state != FSM_SET_HH))) {
                PORTA.DIRSET = HH_L2_LOW | HH_L2_HIGH;
                PORTA.OUTSET = HH_L2_HIGH;
                PORTA.OUTCLR = HH_L2_LOW;
            }
            current_led = LED_HH_L3;
            break;

        case LED_HH_L3:
            if ((current_time & HH_L3_MASK) && (blink || (current_state != FSM_SET_HH))) {
                PORTA.DIRSET = HH_L3_LOW | HH_L3_HIGH;
                PORTA.OUTSET = HH_L3_HIGH;
                PORTA.OUTCLR = HH_L3_LOW;
            }
            current_led = LED_HH_H0;
            break;

        case LED_HH_H0:
            if ((current_time & HH_H0_MASK) && (blink || (current_state != FSM_SET_HH))) {
                PORTA.DIRSET = HH_H0_LOW | HH_H0_HIGH;
                PORTA.OUTSET = HH_H0_HIGH;
                PORTA.OUTCLR = HH_H0_LOW;
            }
            current_led = LED_MM_L0;
            current_time = get_time();
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
    if (debounce_cnt == DEBOUNCE_COUNT) {
        switch (current_state) {
            // If the set button is pressed in the display state, enter the set hour state
            case FSM_DISPLAY:
                if (btn_state == BTN_SET)
                    current_state = FSM_SET_HH;
                break;

            // If the set button is pressed in the hour state, enter the first set minute state
            // If the view button is pressed in the set hour state, increment the time by one hour
            case FSM_SET_HH:
                if (btn_state == BTN_SET)
                    current_state = FSM_SET_MM_L;
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
                if (btn_state == BTN_SET)
                    current_state = FSM_DISPLAY;
                else if (btn_state == BTN_VIEW)
                    increment_time(0, 0, 1);
                break;
        }
    }

    // If the debounce counter has reached the reset threshold, reset the device.
    if (debounce_cnt == RESET_HOLD_TIME)
        reset_device();
}


// ##### ADC Window Comparator Interrupt #####

ISR(ADC0_WCOMP_vect) {
    // Whenever the window comparator triggers, we know that one of the
    //  buttons was pressed. We don't really care about which one, since
    //  either button press will be used to reset the standby timer.
    ADC0.INTFLAGS = ADC_WCMP_bm; // Clear the interrupt flag
    TCA0.SINGLE.CNT = 0;         // Reset the standby timer
}


// ##### PORTA Falling Edge Interrupt #####

ISR(PORTA_PORT_vect) {
    // Clear the interrupt flag
    PORTA.INTFLAGS = PORT_INT1_bm;

    ADC0.CTRLA &= ~ADC_ENABLE_bm;               // Enable ADC0
    TCB0.CTRLA &= ~TCB_ENABLE_bm;               // Enable TCB0
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm; // Enable TCA0
    current_state = FSM_DISPLAY;                // Initialize the FSM
    current_led = LED_MM_L0;                    // Initialize the LED
    current_time = get_time();                  // Initialize the time
    last_btn_state = BTN_NONE;                  // Initialize the last button
    debounce_cnt = 0;                           // Initialize the button counter
}