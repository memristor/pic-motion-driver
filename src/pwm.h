#ifndef PWM_H
#define	PWM_H

// void PWMinit(void);

/***********************************************************************
 * Function Name     : CloseMCPWM
 * Description       : This function Clears the Interrupt enable ,flag
 *                     bits, PTCON, PWMCON1 and PWMCON2 registers.
 * Parameters        : void
 * Return Value      : void
 **********************************************************************/
void CloseMCPWM(void);

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
		config1, unsigned int config2, unsigned int config3);

/*********************************************************************
 * Function Name     : OverrideMCPWM
 * Description       : This function set OVDCON register includes PWM
 *                     output override bits and PWM Manual Output Bits
 * Parameters        : unsigned int config
 * Return Value      : None
 *********************************************************************/
void OverrideMCPWM(unsigned int config);

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
		char updatedisable);

/*************************************************************************
 * Function Name     : SetMCPWMDeadTimeAssignment
 * Description       : This function configures the assignment of dead time
 *                     units to PWM output pairs
 * Parameters        : unsigned int config
 * Return Value      : None
 **************************************************************************/
void SetMCPWMDeadTimeAssignment(unsigned int config);

/***********************************************************************
 * Function Name     : SetMCPWMDeadTimeSourceGeneration
 * Description       : This function configures dead time values and clock
 *					  prescalers.
 * Parameters        : unsigned int config
 * Return Value      : None
 ************************************************************************/
void SetMCPWMDeadTimeGeneration (unsigned int config);

/**********************************************************************
 * Function Name     : SetMCPWMFaultA
 * Description       : This function sets Fault A override and enables
 *                     for pins of PWM
 * Parameters        : unsigned int config includes the FAULT A override
 *                     value,
 *                     Fault A mode, Fault A Pairs Enable
 * Return Value      : None
 **********************************************************************/
void SetMCPWMFaultA(unsigned int config);

#endif	/* PWM_H */
