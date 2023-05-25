#define F_CPU 250000
#include <avr/interrupt.h>
#include <avr/io.h>

#define PA1 0b00000010
#define PA2 0b00000100
#define PA3 0b00001000
#define PA6 0b01000000
#define PA7 0b10000000
// PA6 and PA7 will be used as outputs to blink a pair of LEDs

// 32 bit values. Each byte will have a single bit set to 1. This bit will be the
//  corresponding bit in the PORTA register that must be set or cleared to display
//  the desired pattern on the LED matrix.
#define MM0_HIGH (PA2 << 24) | (PA2 << 16) | (PA7 << 8) | (PA3 << 0)
#define MM1_HIGH (0x0 << 24) | (PA7 << 16) | (PA6 << 8) | (PA2 << 0)
#define HH0_HIGH (PA6 << 24) | (PA3 << 16) | (PA6 << 8) | (PA7 << 0)
#define HH1_HIGH (0x0 << 24) | (0x0 << 16) | (0x0 << 8) | (PA3 << 0)
#define MM0_LOW (PA6 << 24) | (PA7 << 16) | (PA2 << 8) | (PA2 << 0)
#define MM1_LOW (0x0 << 24) | (PA6 << 16) | (PA3 << 8) | (PA3 << 0)
#define HH0_LOW (PA7 << 24) | (PA6 << 16) | (PA2 << 8) | (PA3 << 0)
#define HH1_LOW (0x0 << 24) | (0x0 << 16) | (0x0 << 8) | (PA7 << 0)

/**
 * @brief Gets the current time as a 16 bit BCD value
 * 
 * @return The current time. The most significant byte is hours, and the least significant byte is minutes. 
 */
uint16_t get_time();

/**
 * @brief Prepares the device to enter a low power sleep state
 */
void prep_sleep();

int main(void) {

    // ##### Configure clocks #####
    // OSC20M will operate in 16MHz mode. CLK_MAIN and CLK_PER will both use the /64 prescaler
    //  option to get a 250kHz clock
    // TODO: Should this use a higher frequency clock?
    CCP = CCP_IOREG_gc;                                        // Allow access to protected registers
    CLKCTRL.MCLKCTRLA |= CLKCTRL_CLKSEL_OSC20M_gc;             // Set the main clock to OSC20M
    CCP = CCP_IOREG_gc;                                        // Allow access to protected registers
    CLKCTRL.MCLKCTRLB |= CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm; // Set the main clock prescaler to /64
    CCP = CCP_IOREG_gc;                                        // Allow access to protected registers
    CLKCTRL.MCLKLOCK |= CLKCTRL_LOCKEN_bm;                     // Lock the main clock registers
    CCP = CCP_IOREG_gc;                                        // Allow access to protected registers
    CLKCTRL.OSC32KCTRLA |= CLKCTRL_RUNSTDBY_bm;                // Enable the 32kHz oscillator in standby mode


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
    RTC.PER = 43200u;                                         // Set the RTC period to 12 hours
    while (RTC.STATUS & RTC_CTRLABUSY_bm) continue;           // Wait for the RTC CTRLA register to be ready to be configured
    RTC.CTRLA |= RTC_RTCEN_bm;                                // Enable the RTC


    // ##### Configure PORTA #####
    // The default state of all pins will be inputs with pullups disabled. PA2, PA3, PA6, and PA7
    //  are used to charlieplex a matrix of 12 LEDs, and will be toggled to digital outputs
    //  as appropriate in a timed interrupt.
    PORTA.DIRCLR = PA1 | PA2 | PA3 | PA6 | PA7; // Set all pins to inputs


    // ##### Configure CPUINT #####
    // Five interrupts will be used:
    //  - TCA overflow will trigger the device to enter standby, and has the highest priority
    //  - TCB overflow will trigger the LEDs to update
    //  - ADC conversion will trigger the FSM to update
    //  - ADC window will reset TCA's count
    //  - Falling edge on PA1 will trigger the device to wake up
    CCP = CCP_IOREG_gc;                 // Allow access to protected registers
    CPUINT.CTRLA |= CPUINT_LVL0RR_bm;   // Enable round-robin scheduling
    CPUINT.LVL1VEC = TCA0_OVF_vect_num; // Set the TCA overflow interrupt to priority level 1
    SREG |= SREG_I;                     // Enable interrupts


    // ##### Configure EVSYS #####
    // When TCB overflows, and event will be generated that will trigger the ADC to begin a conversion
    EVSYS.SYNCCH0 = EVSYS_SYNCCH0_TCB0_gc;          // Set the TCB overflow event to trigger sync channel 0
    EVSYS.ASYNCUSER1 = EVSYS_ASYNCUSER9_SYNCCH0_gc; // Set the ADC to be triggered by sync channel 0


    // ##### Configure TCA #####
    // TCA0 will trigger the device to enter sleep mode after 10 seconds of inactivity
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV64_gc;  // Set the TCA clock source to CLK_PER/64, or 250kHz/64 = 3906.25Hz
    TCA0.SINGLE.CTRLB |= TCA_SINGLE_WGMODE_NORMAL_gc; // Set the TCA waveform generation mode to normal
    TCA0.SINGLE.INTCTRL |= TCA_SINGLE_OVF_bm;         // Enable the TCA overflow interrupt
    TCA0.SINGLE.PER = 39062u;                         // Set the TCA period to 10 seconds
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;        // Enable the TCA


    // ##### Configure TCB #####
    // TCB will trigger the LED update interrupt at 720Hz
    TCB0.INTCTRL |= TCB_CAPT_bm; // Enable the TCB capture interrupt
    TCB0.CCMP = 347u;            // Set the TCB compare value to 54
    TCB0.CTRLA |= TCB_ENABLE_bm; // Set the TCB clock source to TCA, and enable the TCB


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
    ADC0.WINLT = 0xC0;                  // Set the ADC window comparator low threshold to ~0.75*Vdd


    for (;;) {
        // TODO: See how much power you'll save if you go into idle mode between updates
        // TODO:    instead of using this busy wait. This could give longer battery life
        continue;
    }
}

// Gets the time as 16-bit BCD value in HH:MM format
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

// Prepares the device to enter a low power sleep state. To do this, the
//  following tasks will be performed:
//  - PA1 is set to a digital input
//  - An interrupt is set to wake the device from sleep when PA1 is pulled low
//  - PA2, PA3, PA6, and PA7 are set to digital inputs
// TODO If you end up using any other peripherals, make sure to disable them here
// TODO the ADC requires a clock between 50kHz and 1.5MHz. This clock will probably also have to be disabled?
void prep_sleep() {
    PORTA.DIRCLR = PA7; // Set PA7 as an input
}

ISR(TCA0_OVF_vect) {
    // Clear the interrupt flag
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;

    // TODO: Put the device to sleep
}

ISR(TCB0_INT_vect) {
    // Clear the interrupt flag
    TCB0.INTFLAGS = TCB_CAPT_bm;


    // TODO: Update the LEDs
}

ISR(ADC0_RESRDY_vect) {
    // Clear the interrupt flag
    ADC0.INTFLAGS = ADC_RESRDY_bm;

    // TODO: Update the FSM
}

ISR(ADC0_WCMP_vect) {
    // Clear the interrupt flag
    ADC0.INTFLAGS = ADC_WCMP_bm;

    // TODO: Reset TCA's count
}

ISR(PORTA_PORT_vect) {
    // Clear the interrupt flag
    PORTA.INTFLAGS = PORT_INT1_bm;

    // TODO: prep the device after waking up
}