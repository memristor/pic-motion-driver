#ifndef ENCODER_H
#define ENCODER_H

void encoder_init(void);
int encoder_left_get_count(void);
int encoder_right_get_count(void);

int motor_encoder_left_get_count(void);
int motor_encoder_right_get_count(void);

void encoder_init_pins(void);

#endif
