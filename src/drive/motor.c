#include <p33FJ128MC802.h>
#include "motor.h"
#include "../util/math.h"
#include "../config.h"

#ifdef BOARD_OLD
// old board
#define LEFT_INH  LATBbits.LATB15
#define RIGHT_INH LATBbits.LATB9

#define RIGHT_IN1 LATBbits.LATB14
#define RIGHT_IN2 LATBbits.LATB12

#define LEFT_IN1 LATBbits.LATB11
#define LEFT_IN2 LATBbits.LATB8
#endif

#ifdef BOARD_NEW
// new board
#define LEFT_INH  LATBbits.LATB12
#define RIGHT_INH LATBbits.LATB11

#define RIGHT_IN1 LATBbits.LATB14
#define RIGHT_IN2 LATBbits.LATB15
	
#define LEFT_IN1 LATBbits.LATB9
#define LEFT_IN2 LATBbits.LATB8
#endif
  
void on_motor_speed_limit_change() {
	// c_motor_speed_limit = clip(-MOTOR_MAX_SPEED, MOTOR_MAX_SPEED, c_motor_speed_limit);
}

void motor_init(void) {
	motor_turn_off();

	P1TCONbits.PTEN = 0;
	PWM1CON1bits.PMOD1 = 1;
	PWM1CON1bits.PEN1H = 1;
	PWM1CON1bits.PEN1L = 0;
	
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
	
	// P2TPER = 1599;
	P2TPER = MOTOR_MAX_SPEED/2;
	P2DC1 = 0;
	P2TCONbits.PTEN = 1;
	
	// turn on bridges
	LEFT_INH = 1;
	RIGHT_INH = 1;
	
	config_on_change(CONF_MOTOR_SPEED_LIMIT, on_motor_speed_limit_change);
}

static inline void motor_left_pwm(unsigned int PWM) {
	P2DC1 = PWM;
}

static inline void motor_right_pwm(unsigned int PWM) {
	P1DC1 = PWM;
}

int left_motor_pwm = 0;
int right_motor_pwm = 0;

#define LEFT_PIN_SW LEFT_IN1 
#define RIGHT_PIN_SW RIGHT_IN2
void motor_left_set_power(int power) {
	power = clip(-c_motor_speed_limit, min(c_motor_speed_limit, c_left_motor_speed_limit), power);
	power = clip(left_motor_pwm-c_motor_rate_of_change, left_motor_pwm+c_motor_rate_of_change, power);
	
	left_motor_pwm = power;
	
	// apply left pwm
	if(power >= 0) {
		LEFT_PIN_SW = 0;
		motor_left_pwm(power);
	} else {
		LEFT_PIN_SW = 1;
		motor_left_pwm(MOTOR_MAX_SPEED + power);
	}
}

int get_left_motor_power(void) {
	return left_motor_pwm;
}
int get_right_motor_power(void) {
	return right_motor_pwm;
}

void motor_right_set_power(int power) {
	power = clip(-c_motor_speed_limit, min(c_motor_speed_limit, c_right_motor_speed_limit), power);
	power = clip(right_motor_pwm-c_motor_rate_of_change, right_motor_pwm+c_motor_rate_of_change, power);

	right_motor_pwm = power;
	
	// apply right pwm
	if (power >= 0) {
		RIGHT_PIN_SW = 0;
		motor_right_pwm(power);
	} else {
		RIGHT_PIN_SW = 1;
		motor_right_pwm(MOTOR_MAX_SPEED + power);
	}
}


void motor_turn_off(void)
{
	left_motor_pwm = 0;
	right_motor_pwm = 0;
	
	// clear the Interrupt enables
	IEC3bits.PWM1IE = 0;
	IEC3bits.FLTA1IE = 0;

	// clear the Interrupt flags
	IFS3bits.PWM1IF = 0;
	IFS3bits.FLTA1IF = 0;

	// clear the PWM control registers
	PTCON   = 0;
	PWMCON1 = 0;
	PWMCON2 = 0;

	P1TCONbits.PTEN = P2TCONbits.PTEN = 0;
	
	// turn off bridges
	LEFT_INH = 0;
	RIGHT_INH = 0;
}
