void encoder_init(void)
{

}


int left_motor_pwm;
int right_motor_pwm;

void encoder_init_pins(void) {
}

#include <stdio.h>
#include "../util/math.h"
int encoder_left_get_count(void) {
	// printf("enc: %d\n", left_motor_pwm);
	return left_motor_pwm * 1;
}

int encoder_right_get_count(void) {
	return right_motor_pwm * 1;
}

int motor_encoder_left_get_count(void){return 0;}
int motor_encoder_right_get_count(void){return 0;}
