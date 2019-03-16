#include "../motor.h"


#undef MOTOR_MAX_SPEED
#undef MOTOR_MAX_POWER
#define MOTOR_MAX_SPEED 3200
#define MOTOR_MAX_POWER 3200


#ifdef BOARD_V2
	#define LEFT_INH
	#define RIGHT_INH 

	#define LEFT_IN1 
	#define LEFT_IN2 LATBbits.LATB15
	
	#define RIGHT_IN1  
	#define RIGHT_IN2 LATBbits.LATB13
#endif

#define LEFT_PIN_SW LEFT_IN2
#define RIGHT_PIN_SW RIGHT_IN2


int left_motor_pwm = 0;
int right_motor_pwm = 0;
  
void on_motor_speed_limit_change() {
	c_motor_speed_limit = clip2(MOTOR_MAX_SPEED, c_motor_speed_limit);
}


void try_timer_pwm() {
		
	PLIB_DEVCON_SystemUnlock(DEVCON_ID_0);
	PLIB_DEVCON_DeviceRegistersUnlock(DEVCON_ID_0, DEVCON_ALL_REGISTERS);
	PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_OC7, OUTPUT_PIN_RPA0);
	PLIB_DEVCON_DeviceRegistersLock(DEVCON_ID_0, DEVCON_ALL_REGISTERS);
	PLIB_DEVCON_SystemLock(DEVCON_ID_0);	
	/* Configure Timer2 in 16-bit mode. Refer to Timer Peripheral Library for the API */
	#define MY_OC_ID OC_ID_7
	
	/* Disable OC module */
	PLIB_OC_Disable(MY_OC_ID);

	/* Select Timer2 as time base */
	PLIB_OC_TimerSelect(MY_OC_ID, OC_TIMER_16BIT_TMR2);

	/* Set period of time base. Refer to Timer Peripheral Library for the API */

	/* Select compare mode. PWM with fault protection mode is selected ,
	   fault selection is preset in the hardware*/

	PLIB_OC_ModeSelect(MY_OC_ID, OC_COMPARE_PWM_MODE_WITH_FAULT_PROTECTION );
	/*or use PLIB_OC_FaultInputSelect(MY_OC_ID, OC_FAULT_PRESET) to achieve the same*/

	/* Set buffer size to 16-bits. Refer to Timer Peripheral Library for the API */
	PLIB_OC_BufferSizeSelect(MY_OC_ID, OC_BUFFER_SIZE_16BIT);

	/* Set buffer (initial duty cycle) Value */
	PLIB_OC_Buffer16BitSet(MY_OC_ID, 0x00FF);

	/*Set pulse width (Duty cycle) value */
	PLIB_OC_PulseWidth16BitSet(MY_OC_ID, 0x0FFF);

	/* Configure interrupts associated with Output Compare module. Refer to
	Interrupts Peripheral Library for the API */


	/* Enable Timer2. Refer to Timer Peripheral Library for the API */
	PLIB_TMR_Start(TMR_ID_2);

	/* Enable OC module */
	PLIB_OC_Enable(MY_OC_ID);



	/* Check for PWM Fault */
	while(!PLIB_OC_FaultHasOccurred(MY_OC_ID))
	{
		/* If no PWM fault, continue normal operation*/
	}
	while(1){}
}

#define SYSCLK 120000000
#define PWM_FREQ    17000
#define DUTY_CYCLE  10

static void init_timer_pwm1() {
	#define OC OC_ID_3
	#define MOTOR1 OC_ID_3
    PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_OC1, OUTPUT_PIN_RPB14);
	PLIB_OC_TimerSelect(OC, OC_TIMER_16BIT_TMR2);
	PLIB_OC_ModeSelect(OC, OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION );
	PLIB_TMR_Period32BitSet(TMR_ID_2, SYSCLK/PWM_FREQ/2 - 1);
    PLIB_TMR_PrescaleSelect(TMR_ID_2, TMR_PRESCALE_VALUE_1);
    PLIB_OC_PulseWidth32BitSet(OC, 0.0*PR2); 
    PLIB_TMR_Start(TMR_ID_2);
    PLIB_OC_Enable(OC);
}
static void init_timer_pwm2() {
	#undef OC
	#define OC OC_ID_2
	#define MOTOR2 OC_ID_2
    PLIB_PORTS_RemapOutput(PORTS_ID_0, OUTPUT_FUNC_OC1, OUTPUT_PIN_RPB12);
	PLIB_OC_TimerSelect(OC, OC_TIMER_16BIT_TMR3);
	PLIB_OC_ModeSelect(OC, OC_COMPARE_PWM_MODE_WITHOUT_FAULT_PROTECTION );
	PLIB_TMR_Period32BitSet(TMR_ID_3, SYSCLK/PWM_FREQ/2 - 1);
    PLIB_TMR_PrescaleSelect(TMR_ID_3, TMR_PRESCALE_VALUE_1);
    PLIB_OC_PulseWidth32BitSet(OC, 0.0*PR3); 
    PLIB_TMR_Start(TMR_ID_3);
    PLIB_OC_Enable(OC);
}

int motor_active = 0;

void motor_init(void) {
	// return;
	if(motor_active) return;
	motor_active = 1;
	// motor_turn_off();
	left_motor_pwm=0;
	right_motor_pwm=0;
	
	TRISA &= ~0x480;
	LATA |= 0x480;
	
	TRISB &= ~((1<<13) | (1<<15));
	
	static int first_time = 1;
	if(first_time) {
		init_timer_pwm1();
		init_timer_pwm2();
		first_time=0;
	}
	
	long unsigned cnt = 0;
	c_motor_speed_limit = 3200;
	c_motor_rate_of_change=3200;
}

void motor_turn_off(void) {
	motor_active = 0;
	// PLIB_OC_Disable(OC_ID_2);
	// PLIB_OC_Disable(OC_ID_3);
	// PLIB_TMR_Stop(TMR_ID_2);
	// PLIB_TMR_Stop(TMR_ID_3);
	LATA &= ~0x480;
	left_motor_pwm = 0;
	right_motor_pwm = 0;
}


static inline void motor_left_pwm(int pwm) {
	// apply left pwm
	// pwm = -pwm;
	
	if(pwm >= 0) {
		LEFT_PIN_SW = 0;
	} else {
		LEFT_PIN_SW = 1;
		pwm += MOTOR_MAX_SPEED;
	}
	
	PLIB_OC_PulseWidth32BitSet(OC_ID_3, (PR2-1) * clip2(MOTOR_MAX_POWER, pwm)/MOTOR_MAX_POWER);
}

static inline void motor_right_pwm(int pwm) {
	pwm = -pwm;
	
	if(pwm >= 0) {
		RIGHT_PIN_SW = 0;
	} else {
		RIGHT_PIN_SW = 1;
		pwm += MOTOR_MAX_SPEED;
	}
	
	PLIB_OC_PulseWidth32BitSet(OC_ID_2, (PR3-1) * clip2(MOTOR_MAX_POWER, pwm)/MOTOR_MAX_POWER);
}


int ramp_power(int power) {
	int ramp_start = 300;
	int ramp_end = 800;
	int ramp_width = ramp_end - ramp_start;
	int apower = abs(power);
	int spower = sign(power);
	if (apower < ramp_end) {
		if (apower < ramp_start) {
			power = 0;
		} else {
			power = spower * (ramp_end * (apower-ramp_start)) / ramp_width;
		}
	}
	return power;
}

// hadrware independent
void motor_left_set_power(int power) {
	power = clip(-c_motor_speed_limit, min(c_motor_speed_limit, c_left_motor_speed_limit), power);
	power = clip(left_motor_pwm-c_motor_rate_of_change, left_motor_pwm+c_motor_rate_of_change, power);
	// power = ramp_power(power);
	left_motor_pwm = power;
	motor_left_pwm(power);
}

void motor_right_set_power(int power) {
	power = clip(-c_motor_speed_limit, min(c_motor_speed_limit, c_right_motor_speed_limit), power);
	power = clip(right_motor_pwm-c_motor_rate_of_change, right_motor_pwm+c_motor_rate_of_change, power);

	// power = ramp_power(power);
	right_motor_pwm = power;
	motor_right_pwm(power);
}

int motor_left_get_power(void) {
	return left_motor_pwm;
}

int motor_right_get_power(void) {
	return right_motor_pwm;
}

