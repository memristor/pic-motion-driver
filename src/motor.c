#include <p33FJ128MC802.h>
#include "motor.h"
#include "math.h"
#include "config.h"

void on_motor_speed_limit_change() {
	// c_motor_speed_limit = clip(-MOTOR_MAX_SPEED, MOTOR_MAX_SPEED, c_motor_speed_limit);
}

void motor_init(void)
{
	motor_turn_off();

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
	// P1TPER = 1599;
	P1TPER = MOTOR_MAX_SPEED/2;
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
	
	// P2TPER = 1599;
	P2TPER = MOTOR_MAX_SPEED/2;
	P2DC1 = 0;
	P2TCONbits.PTEN = 1;
	
	// turn on bridges
	LATBbits.LATB11 = 1;
	LATBbits.LATB12 = 1;
	
	config_on_change(CONF_MOTOR_SPEED_LIMIT, on_motor_speed_limit_change);
}

static inline void motor_left_pwm(unsigned int PWM)
{
	P2DC1 = PWM;
}

static inline void motor_right_pwm(unsigned int PWM)
{
	P1DC1 = PWM;
}

int left_motor_pwm = 0;
int right_motor_pwm = 0;

void motor_left_set_power(int power) {
	// power = clip(-c_motor_speed_limit, c_motor_speed_limit, power);
	
	// power = clip(left_motor_pwm-c_motor_rate_of_change, left_motor_pwm+c_motor_rate_of_change, power);
	
	left_motor_pwm = power;
	
	// apply left pwm
	if(power >= 0)
	{
		LATBbits.LATB9 = 0;
		motor_left_pwm(power);
	}
	else
	{
		LATBbits.LATB9 = 1;
		motor_left_pwm(MOTOR_MAX_SPEED + power);
	}
}

void motor_right_set_power(int power) {
	// power = clip(-c_motor_speed_limit, c_motor_speed_limit, power);
	
	// power = clip(right_motor_pwm-c_motor_rate_of_change, right_motor_pwm+c_motor_rate_of_change, power);

	right_motor_pwm = power;
	
	// apply right pwm
	if (power >= 0)
	{
		LATBbits.LATB15 = 0;
		motor_right_pwm(power);
	}
	else
	{
		LATBbits.LATB15 = 1;
		motor_right_pwm(MOTOR_MAX_SPEED + power);
	}
}


void motor_turn_off(void)
{
	/* clear the Interrupt enables */
	IEC3bits.PWM1IE = 0;
	IEC3bits.FLTA1IE = 0;

	/* clear the Interrupt flags */
	IFS3bits.PWM1IF = 0;
	IFS3bits.FLTA1IF = 0;

	/* clear the PWM control registers */
	PTCON   = 0;
	PWMCON1 = 0;
	PWMCON2 = 0;

	P1TCONbits.PTEN = P2TCONbits.PTEN = 0;
	
	// turn off bridges
	LATBbits.LATB11 = 0;
	LATBbits.LATB12 = 0;
}
