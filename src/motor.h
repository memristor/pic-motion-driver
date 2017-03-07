#ifndef PWM_H
#define	PWM_H

#define MOTOR_MAX_SPEED 3200

void motor_init(void);
void motor_turn_off(void);

void motor_set_rate_of_change(int change);
void motor_left_set_power(int power);
void motor_right_set_power(int power);

#endif	/* PWM_H */
