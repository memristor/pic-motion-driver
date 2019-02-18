#ifndef ENCODER_H
#define ENCODER_H
#include "../config.h"
void encoder_init(void);

int encoder_odometry_left_get_count(void);
int encoder_odometry_right_get_count(void);
int encoder_motor_left_get_count(void);
int encoder_motor_right_get_count(void);

int encoder_odometry_left_get_velocity(void);
int encoder_odometry_right_get_velocity(void);
int encoder_motor_left_get_velocity(void);
int encoder_motor_right_get_velocity(void);

void encoder_init_pins(void);

#endif
