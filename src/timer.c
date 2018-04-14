#include <p33FJ128MC802.h>
#include "timer.h"

void timer_deinit(void)
{
    IEC0bits.T1IE = 0; // Disable the Timer1 interrupt
    T1CONbits.TON = 0; // Disable timer1
    IFS0bits.T1IF = 0; // Clear Timer interrupt flag
}

void timer_init(void)
{ 
	IEC0bits.T1IE = 0; // Disable the Timer1 interrupt
	T1CONbits.TON = 0; // Disable timer1
	IFS0bits.T1IF = 0; // Clear Timer interrupt flag

	T1CONbits.TGATE = 0;
	T1CONbits.TCKPS = 0;
	T1CONbits.TCS = 0;

	TMR1 = 0;
	PR1 = 29479; // 1ms interrupts

	IPC0bits.T1IP = 2; // interrupt priority == 2
	IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
	IEC0bits.T1IE = 1; // Enable Timer1 interrupt
	T1CONbits.TON = 1;
}
