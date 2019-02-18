/* The following code will output a steady PWM wave with duty cycle equal
 * to the constant DUTY_CYCLE (10% below) and frequency equal to 
 * the constant PWM_FREQ (16kHz below)
 * 
 * RB7 is the output pin
 */
 
// Include Header Files
#include <p32xxxx.h>
#include <plib.h>
 
// Configuration Bits
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
                                    // see figure 8.1 in datasheet for more info
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config ICESEL = ICS_PGx1    // ICE/ICD Comm Channel Select
#pragma config JTAGEN = OFF         // Disable JTAG
#pragma config FSOSCEN = OFF        // Disable Secondary Oscillator
 
// Defines
#define SYSCLK  40000000L
 
// Note that 612Hz (PR2=0xffff) is the lowest pwm frequency with our configuration
// : To get lower, use a timer prescaler or use the 32-bit timer mode
#define PWM_FREQ    16000
#define DUTY_CYCLE  10
 
int main(void)
{
    SYSTEMConfigPerformance(SYSCLK);
 
    // Set OC1 to pin RB7 with peripheral pin select
    RPB15Rbits.RPB15R = 0x0005;
 
    // Configure standard PWM mode for output compare module 1
    OC1CON = 0x0006; 
 
    // A write to PRy configures the PWM frequency
    // PR = [FPB / (PWM Frequency * TMR Prescale Value)] – 1
    // : note the TMR Prescaler is 1 and is thus ignored
    PR2 = (SYSCLK / PWM_FREQ) - 1;
 
    // A write to OCxRS configures the duty cycle
    // : OCxRS / PRy = duty cycle
    OC1RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100);
 
    T2CONSET = 0x8000;      // Enable Timer2, prescaler 1:1
    OC1CONSET = 0x8000;     // Enable Output Compare Module 1
 
    // loop indefinitely
    while( 1);
 
    return 1;
}
