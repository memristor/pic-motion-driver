#include <p33FJ128MC802.h>
#include "pwm.h"

void PWMinit(void)
{
	CloseMCPWM();

	P1TCONbits.PTEN = 0;
	PWM1CON1bits.PMOD1 = 1;
	PWM1CON1bits.PEN1H = 1;
	PWM1CON1bits.PEN1L = 0;
	
	/*
	P1DTCON1bits.DTBPS = 0;
	P1DTCON1bits.DTB = 5;
	P1DTCON1bits.DTAPS = 0;
	P1DTCON1bits.DTA = 5;
	*/
	
	P1TMR = 1;
	P1TPER = 1599;
	P1DC1 = 0;
	P1TCONbits.PTEN = 1;

	
	// init PWM2
	P2TCONbits.PTEN = 0;
	PWM2CON1bits.PMOD1 = 1;
	PWM2CON1bits.PEN1H = 1;
	PWM2CON1bits.PEN1L = 0;
	
	/*
	P2DTCON1bits.DTBPS = 0;
	P2DTCON1bits.DTB = 5;
	P2DTCON1bits.DTAPS = 0;
	P2DTCON1bits.DTA = 5;
	*/
	
	P2TPER = 1599;
	P2DC1 = 0;
	P2TCONbits.PTEN = 1;
	
	// turn on bridges
	LATBbits.LATB11 = 1;
	LATBbits.LATB12 = 1;
}

inline void left_pwm(unsigned int PWM)
{
	P2DC1 = PWM;
}

inline void right_pwm(unsigned int PWM)
{
	P1DC1 = PWM;
}


/***********************************************************************
 * Function Name     : CloseMCPWM
 * Description       : This function Clears the Interrupt enable ,flag
 *                     bits, PTCON, PWMCON1 and PWMCON2 registers.
 * Parameters        : void
 * Return Value      : void
 **********************************************************************/

void CloseMCPWM(void)
{
	/* clear the Interrupt enables */
	IEC3bits.PWM1IE = 0;
	IEC3bits.FLTA1IE = 0;

	/* clear the Interrupt flags */
	IFS3bits.PWM1IF = 0;
	IFS3bits.FLTA1IF = 0;

	/* clear the PWM control registers */
	PTCON       =       0;
	PWMCON1     =       0;
	PWMCON2     =       0;

	P1TCONbits.PTEN = P2TCONbits.PTEN = 0;
	
	// turn off bridges
	LATBbits.LATB11 = 0;
	LATBbits.LATB12 = 0;
}

/*********************************************************************
 * Function Name     : OpenMCPWM
 * Description       : This function configures PWM module for the
 *                     following parameters:
 *                     period, sptime, PWM Mode, Clock Prescale,
 *                     Output Postscale, high res mode, I/O pair mode,
 *                     I/O pair mode,I/O pair enable, Special event
 *                     postscale, Special event direction, override
 *                     synchronization.
 * Parameters        : unsigned int period
 *                     unsigned int sptime
 *                     unsigned int config1
 *                     unsigned int config2,
 *                     unsigned int config3
 * Return Value      : None
 **********************************************************************/

void OpenMCPWM(unsigned int period, unsigned int sptime, unsigned int
		config1, unsigned int config2, unsigned int config3)
{
	PTPER   = period;
	SEVTCMP = sptime;
	PWMCON1 = config2;
	PWMCON2 = config3;
	PTCON   = config1;
}

/*********************************************************************
 * Function Name     : OverrideMCPWM
 * Description       : This function set OVDCON register includes PWM
 *                     output override bits and PWM Manual Output Bits
 * Parameters        : unsigned int config
 * Return Value      : None
 *********************************************************************/

void OverrideMCPWM(unsigned int config)
{
	OVDCON = config;
}

/*********************************************************************
 * Function Name     : SetDCMCPWM
 * Description       : This function updates the dutycycle register and
 *                     updatedisable bit.
 * Parameters        : unsigned int dutycyclereg for selection of reg
 *                      (ie PDC1, PDC2...)
 *                     unsigned int dutycycle
 *                     char updatedisable
 * Return Value      : None
 **********************************************************************/

void SetDCMCPWM(unsigned int dutycyclereg, unsigned int dutycycle,
		char updatedisable)
{
	PWMCON2bits.UDIS = updatedisable & 0x1;

	/* Assign dutycycle to the duty cycle register */
	*(&PDC1+dutycyclereg -1) = dutycycle;
}

/*************************************************************************
 * Function Name     : SetMCPWMDeadTimeAssignment
 * Description       : This function configures the assignment of dead time
 *                     units to PWM output pairs
 * Parameters        : unsigned int config
 * Return Value      : None
 **************************************************************************/

void SetMCPWMDeadTimeAssignment(unsigned int config)
{
	DTCON2 = config ;
}

/***********************************************************************
 * Function Name     : SetMCPWMDeadTimeSourceGeneration
 * Description       : This function configures dead time values and clock
 *					  prescalers.
 * Parameters        : unsigned int config
 * Return Value      : None
 ************************************************************************/
void SetMCPWMDeadTimeGeneration (unsigned int config)
{
	DTCON1 = config;
}


/**********************************************************************
 * Function Name     : SetMCPWMFaultA
 * Description       : This function sets Fault A override and enables
 *                     for pins of PWM
 * Parameters        : unsigned int config includes the FAULT A override
 *                     value,
 *                     Fault A mode, Fault A Pairs Enable
 * Return Value      : None
 **********************************************************************/
void SetMCPWMFaultA(unsigned int config)
{
	FLTACON = config;
}

