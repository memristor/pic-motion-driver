#include <p33FJ128MC802.h>
#include "timer.h"

/********************************************************************
*    Function Name:  CloseTimer1                                    *
*    Description:    This routine disables the Timer1 and its       *
*                    interrupt enable and flag bits.                *
*    Parameters:     None                                           *
*    Return Value:   None                                           *
********************************************************************/
void CloseTimer1(void)
{
    IEC0bits.T1IE = 0;      /* Disable the Timer1 interrupt */
    T1CONbits.TON = 0;      /* Disable timer1 */
    IFS0bits.T1IF = 0;      /* Clear Timer interrupt flag */
}

/*******************************************************************
*    Function Name: ConfigIntTimer1                                *
*    Description:    This Function Configures Interrupt and sets   *
*                    Interrupt Priority                            *
*    Return Value:  None                                           *
*******************************************************************/



void TimerInit(void)
{ 
	IEC0bits.T1IE = 0;      /* Disable the Timer1 interrupt */
	T1CONbits.TON = 0;      /* Disable timer1 */
	IFS0bits.T1IF = 0;      /* Clear Timer interrupt flag */

	T1CONbits.TGATE = 0;
	T1CONbits.TCKPS = 0;
	T1CONbits.TCS = 0;

	TMR1 = 0;
	PR1 = 29479;        // ide na 1ms

	IPC0bits.T1IP = 2; // prioritet prekida == 2
	IFS0bits.T1IF = 0; // Clear Timer1 Interrupt Flag
	IEC0bits.T1IE = 1; // Enable Timer1 interrupt
	T1CONbits.TON = 1;
}
