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


// unit conversions
#define MILIMETER_TO_2INC(x) 			(((long long)(x) << 1)*K2)
#define MILIMETER_TO_INC(x) 			((long)(x)*K2)
#define INC_TO_MILIMETER(x) 			((x) / K2)
#define DOUBLED_INC_TO_MILIMETER(x) 	(((x) / 2) / K2)
#define DEG_TO_INC_ANGLE(x) 			((long)(x)*K1/360)
#define INC_TO_DEG_ANGLE(x) 			((x)*360/K1)
#define RAD_TO_INC_ANGLE(x) 			((x)/(2.0*PI)*K1)
#define RAD_TO_DEG_ANGLE(x) 			((x)*180.0/PI)



// stuck detection variables
#define STUCK_DISTANCE_JUMP_ERROR_THRESHOLD MILIMETER_TO_INC(400)
#define STUCK_ROTATION_JUMP_ERROR_THRESHOLD DEG_TO_INC_ANGLE(180)

#define STUCK_DISTANCE_MAX_FAIL_COUNT 200
#define STUCK_ROTATION_MAX_FAIL_COUNT 200

// --- regulator PD (proportional, differential) components, trial & error ---
// depends on timer interrupt period

// for distance regulator
#define Gp_D	5.5
#define Gd_D	200

// for rotation regulator
#define Gp_T	3
#define Gd_T	90


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

void turn_and_go(int Xd, int Yd, unsigned char end_speed, char direction);
void forward(int length, unsigned char end_speed);
void rotate_absolute_angle(int angle);
char turn(int angle);
void arc(long Xc, long Yc, int Fi, char direction_angle, char direction);
void move_to(long x, long y, char direction, int radius);
void stop(void);

void set_speed_accel(float v);
void set_speed(unsigned char tmp);
void set_rotation_speed(unsigned char max_speed, unsigned char max_accel);
void set_position(int X, int Y, int orientation);

void send_status_and_position(void);
void report_status(void);
enum State get_status(void);
void force_status(enum State);

void smooth_stop(void);
void calculate_K();
#endif	/* REGULACIJA_H */

