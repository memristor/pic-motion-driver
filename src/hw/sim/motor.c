#include "../motor.h"

void on_motor_speed_limit_change() {
	// c_motor_speed_limit = clip(-MOTOR_MAX_SPEED, MOTOR_MAX_SPEED, c_motor_speed_limit);
}

void motor_init(void) {
	motor_turn_off();
	config_on_change(CONF_MOTOR_SPEED_LIMIT, on_motor_speed_limit_change);
}

int left_motor_pwm = 0;
int right_motor_pwm = 0;

static inline void motor_left_pwm(unsigned int PWM) {
	// left_motor_pwm = PWM;
}

static inline void motor_right_pwm(unsigned int PWM) {
	// right_motor_pwm = PWM;
}

void motor_left_set_power(int power) {
	power = clip(-c_motor_speed_limit, min(c_motor_speed_limit, c_left_motor_speed_limit), power);
	power = clip(left_motor_pwm-c_motor_rate_of_change, left_motor_pwm+c_motor_rate_of_change, power);
	
	left_motor_pwm = power;
	
	// apply left pwm
	if(power >= 0) {
		motor_left_pwm(power);
	} else {
		motor_left_pwm(MOTOR_MAX_SPEED + power);
	}
}

int motor_left_get_power(void) {
	return left_motor_pwm;
}

int motor_right_get_power(void) {
	return right_motor_pwm;
}

void motor_right_set_power(int power) {
	power = clip(-c_motor_speed_limit, min(c_motor_speed_limit, c_right_motor_speed_limit), power);
	power = clip(right_motor_pwm-c_motor_rate_of_change, right_motor_pwm+c_motor_rate_of_change, power);

	right_motor_pwm = power;
	
	// apply right pwm
	if (power >= 0) {
		motor_right_pwm(power);
	} else {
		motor_right_pwm(MOTOR_MAX_SPEED + power);
	}
}


void motor_turn_off(void) {
	left_motor_pwm = 0;
	right_motor_pwm = 0;
}
