#ifndef PWM_H
#define	PWM_H

#define MOTOR_MAX_SPEED 3200

void motor_init(void);
void motor_turn_off(void);

void motor_left_set_power(long power);
void motor_right_set_power(long power);

#endif	/* PWM_H */
