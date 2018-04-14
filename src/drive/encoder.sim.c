void encoder_init(void)
{

}


int left_motor_pwm;
int right_motor_pwm;

void encoder_init_pins(void) {
}

#include <stdio.h>
int encoder_left_get_count(void) {
	// printf("enc: %d\n", left_motor_pwm);
	return left_motor_pwm;
}

int encoder_right_get_count(void) {
	return right_motor_pwm;
}
