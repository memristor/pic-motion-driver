#ifndef PWM_H
#define	PWM_H
#include "../config.h"
#include "../math.h"
#define MOTOR_MAX_SPEED 3200
#define MOTOR_MAX_POWER 3200

void motor_init(void);
void motor_turn_off(void);

void motor_set_rate_of_change(int change);
void motor_left_set_power(int power);
void motor_right_set_power(int power);

int motor_left_get_power(void);
int motor_right_get_power(void);

#endif	/* PWM_H */
