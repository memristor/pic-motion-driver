#include "init.h"
#include "pwm.h"
#include <p33FJ128MC802.h>

void TimerInit(void)
{ 
	IEC0bits.T1IE = 0;      /* Disable the Timer1 interrupt */
	T1CONbits.TON = 0;      /* Disable timer1 */
	IFS0bits.T1IF = 0;      /* Clear Timer interrupt flag */

	T1CONbits.TGATE = 0;
	T1CONbits.TCKPS = 0;
	T1CONbits.TCS = 0;

	TMR1 = 0;
	PR1 = 29479;    // ide na 1ms

	IPC0bits.T1IP = 2; // prioritet prekida == 2
	IFS0bits.T1IF = 0;// Clear Timer1 Interrupt Flag
	IEC0bits.T1IE = 1;// Enable Timer1 interrupt
	T1CONbits.TON = 1;
}

void PWMinit(void)
{
	CloseMCPWM();

	P1TCONbits.PTEN = 0;
	PWM1CON1bits.PMOD1 = 1;
	PWM1CON1bits.PEN1H = 1;
	PWM1CON1bits.PEN1L = 0;
	//P1DTCON1bits.DTBPS = 0;
	//P1DTCON1bits.DTB = 5;
	//P1DTCON1bits.DTAPS = 0;
	//P1DTCON1bits.DTA = 5;
	P1TMR = 1;
	P1TPER = 1599;
	P1DC1 = 0;
	P1TCONbits.PTEN = 1;

	
	// init PWM2
	P2TCONbits.PTEN = 0;
	PWM2CON1bits.PMOD1 = 1;
	PWM2CON1bits.PEN1H = 1;
	PWM2CON1bits.PEN1L = 0;
	//P2DTCON1bits.DTBPS = 0;
	//P2DTCON1bits.DTB = 5;
	//P2DTCON1bits.DTAPS = 0;
	//P2DTCON1bits.DTA = 5;
	P2TPER = 1599;
	P2DC1 = 0;
	P2TCONbits.PTEN = 1;
	
	
	//ukljuci mostove
	LATBbits.LATB11 = 1;
	LATBbits.LATB12 = 1;
}

void QEIinit()
{
	//konfigurisi registre:
	QEI1CONbits.POSRES=0;       //index impuls ne resetuje brojac
	QEI1CONbits.TQCS=1;         //brojac broji impulse sa QEA ulaza
	QEI1CONbits.UPDN_SRC=1;     //za to vreme QEB odredjuje smer brojanja
	QEI1CONbits.QEIM=6;         //Quadrature Encoder Interface enabled (x4 mode) with index pulse reset of position counter
	QEI1CONbits.TQCKPS=0;

	MAX1CNT=0000;
	POS1CNT=0;

	//konfigurisi registre:
	QEI2CONbits.POSRES=0;       //index impuls ne resetuje brojac
	QEI2CONbits.TQCS=1;         //brojac broji impulse sa QEA ulaza
	QEI2CONbits.UPDN_SRC=1;     //za to vreme QEB odredjuje smer brojanja
	QEI2CONbits.QEIM=6;         //Quadrature Encoder Interface enabled (x4 mode) with index pulse reset of position counter
	QEI2CONbits.TQCKPS=0;

	MAX2CNT=0000;
	POS2CNT=0;
}

void PortInit()
{

	TRISAbits.TRISA4=0;

	TRISBbits.TRISB8=0;
	TRISBbits.TRISB9=0;
	TRISBbits.TRISB10=0;
	TRISBbits.TRISB11=0;
	TRISBbits.TRISB12=0;
	TRISBbits.TRISB13=0;
	TRISBbits.TRISB14=0;
	TRISBbits.TRISB15=0;

	LATAbits.LATA4 = 0;

	LATBbits.LATB8 = 0;
	LATBbits.LATB9 = 0;
	LATBbits.LATB10 = 0;
	LATBbits.LATB11 = 1;
	LATBbits.LATB12 = 1;
	LATBbits.LATB13 = 0;
	LATBbits.LATB14 = 0;
	LATBbits.LATB15 = 0;
}

