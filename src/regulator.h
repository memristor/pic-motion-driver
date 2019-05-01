#ifndef REGULATOR_H
#define	REGULATOR_H
#include <stdint.h>


#define PI	3.1415926535897932384626433832795

// odometry encoder diameter
//#define R_wheel	90.9

// odometry encoder wheel distance
//#define wheel_distance 330.0

/*
	number of increments per full circle represents circumference of encoder wheel
	it is here converted into arc length of circle with radius of wheel distance
	
	used for converting encoder wheel increments to rotation around an wheel, which means
	translates to robot orientation
	= 57666L
*/

//#define K1	(long)(4.0*2048.0 * wheel_distance / (R_wheel / 2.0))

/*
	8192 - number of increments for encoder wheel to spin full circle (convertible to degrees)
	2*r*pi - wheel circumference in milimeters
	8192 [inc] = (r [mm] * 2*pi[rad])*const
	const = 8192 [inc] / (2*r*pi) [mm] => number of increments per milimeter
	= 32.05f
*/
//#define K2	/*[inc/mm]*/	(float)(4.0*2048.0 / (R_wheel * PI))

/*
	Encoder used: Part no:MA5D1N4FBK1SA0; Type no.: sca24-5000-N-04-09-64-01-S-00
*/

#define VMAX K2

// error codes
#define ERROR 0
#define BREAK 0
#define OK 1
//

#define REGULATOR_POSITION 0
#define REGULATOR_LINEAR 1
#define REGULATOR_SPEED 3

// unit conversions
#define MM_TO_DINC(x) 					(((long long)(x) << 1)*K2)
#define MM_TO_INC(x) 					((long)(x)*K2)
#define INC_TO_MM(x) 					((x) / K2)
#define DINC_TO_MM(x) 					(((x) / 2) / K2)

#define DEG_TO_INC_ANGLE(x) 			((long)(x)*K1/360)
#define DEG_TO_RAD_ANGLE(x) 			((float)(x)*3.141592f/180.0f)
#define INC_TO_DEG_ANGLE(x) 			((x)*360/K1)
#define INC_TO_RAD_ANGLE(x) 			((float)(x)*6.283184f/(float)K1)
#define RAD_TO_INC_ANGLE(x) 			((x)*K1/(2.0*PI))
#define RAD_TO_DEG_ANGLE(x) 			((x)*180.0/PI)

#define RUN_EACH_NTH_CYCLES(counter_type, nth, run) { static counter_type _cycle_ = 0; if(nth > 0 && ++_cycle_ >= nth) { _cycle_ = 0; run; } }

// stuck detection variables
#define STUCK_DISTANCE_JUMP_ERROR_THRESHOLD MM_TO_INC(400)
#define STUCK_ROTATION_JUMP_ERROR_THRESHOLD DEG_TO_INC_ANGLE(180)

#define STUCK_DISTANCE_MAX_FAIL_COUNT 200
#define STUCK_ROTATION_MAX_FAIL_COUNT 200

// --- regulator PD (proportional, differential) components, trial & error ---
// depends on timer interrupt period



// ----------------------------------------------------------------------------

enum State
{
	STATUS_IDLE = 'I',
	STATUS_MOVING = 'M',
	STATUS_ROTATING = 'R',
	STATUS_STUCK = 'S',
	STATUS_ERROR = 'E'
};

void regulator_init(void);
void reset_driver(void);
void reset_stuck(void);

// standard commands
void turn_and_go(int Xd, int Yd, char direction);
void forward(int length);
void forward_lazy(int length, uint8_t speed);
void rotate_absolute_angle(int angle);
void rotate_absolute_angle_inc(int32_t angle);
char turn(int angle);
char turn_inc(int32_t angle);
void arc(long Xc, long Yc, int Fi, char direction);
void arc_relative(int R, int Fi);
void move_to(long x, long y, int radius, char direction);
void stop(void);
void motor_const(int a, int b);
void speed_const(int a, int b);
void set_speed_accel(float v);
void set_speed(unsigned char tmp);
void set_rotation_speed(unsigned char max_speed, unsigned char max_accel);
void set_position(int X, int Y, int orientation);
void diff_drive(int x,int y, int fi);
void send_status_and_position(void);
void report_status(int status);
void on_status_changed(void);
enum State get_status(void);
void force_status(enum State);

void smooth_stop(void);
void soft_stop(void);

void cmd_pwm_opto();
void reset_packet_count();
void regulator_interrupt();
#endif	/* REGULACIJA_H */

