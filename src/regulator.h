#ifndef REGULATOR_H
#define	REGULATOR_H


#define PI	3.1415926535897932384626433832795

// Precnik odometrijskog tocka (u milimetrima)
#define R_wheel	81.4

// Rastojanje izmedju odometrijskih tockova (u milimetrima)
#define wheel_distance 286.5

/*
	Broj inkremenata jednog tocka da se obrne pun krug poluprecnika rastojanja izmedju tockova,
	recimo da drugi tocak stoji u mestu, enkoder drugog tocka treba da napravi K1 broj inkrementa
	brojaca da bi taj tocak napravio pun krug oko stojeceg tocka u centru.
	
	increments = 8192 = 4*2048 = 8192 inkremenata po punom obrnutom krugu jednog tocka
	ugao = 8192 => 2*pi*const (const - neka konstanta koja povecava rezoluciju ugla, tj. u ovom slucaju ako se 
					 izracuna 8192/(2*pi) = 1303/rad dobije se 1303 inkrementa po 1 radijanu, odnosno najmanji
					 radijan koji enkoder moze predstaviti je ~0.00077 radijana tj. 0.044 stepeni)
	d - rastojanje izmedju tockova
	
	increments * d/r = increments*d/(R/2) = increments * 2 * d/R => (broj inkremenata jednog tocka da se obrne pun krug poluprecnika d tj. poluprecnika rastojanja izmedju tockova)
	
	korisna je za racunanje orijentacije na osnovu razlike inkrementa 2 tocka
*/
#define K1	(long)(4*2048.0f * 2 * wheel_distance / R_wheel)

/*
	Broj inkremenata po 1 milimetru
	8192 - broj inkrementa za obrtaj 360 stepeni tj. pun krug
	2*r*pi - obim tocka tj. put koji tocak predje za 1 pun obrtaj
	8192 [inc] = (r [mm] * 2*pi[rad])*const
	const = 8192 [inc] / (2*r*pi) [mm] => broj inkrementa po milimetru
*/
#define K2	(float)(4*2048.0f / (R_wheel * PI))

/*
	Encoder used: Part no:MA5D1N4FBK1SA0; Type no.: sca24-5000-N-04-09-64-01-S-00
*/

#define VMAX K2

// error codes
#define ERROR 0
#define BREAK 0
#define OK 1

#define PWM_MAX_SPEED 3200

// --- regulator PD (proportional, differential) components, trial & error ---
// depends on timer interrupt period

// for distance regulator
#define Gp_D	5.5
#define Gd_D	200

// for rotation regulator
#define Gp_T	3
#define Gd_T	90
// ----------------------------------------------------------------------------


// unit conversions
#define MILIMETER_TO_DOUBLED_INC(x) (((long long)(x) >> 1)*K2)
#define MILIMETER_TO_INC(x) ((long)(x)*K2)
#define INC_TO_MILIMETER(x) ((x) / K2)
#define DOUBLED_INC_TO_MILIMETER(x) (((x) / 2) / K2)
#define DEG_TO_INC_ANGLE(x) ((x)*K1/360)
#define INC_TO_DEG_ANGLE(x) ((x)*360/K1)
#define RAD_TO_INC_ANGLE(x) ((x)/(2.0*PI)*K1)
#define RAD_TO_DEG_ANGLE(x) ((x)*180.0/PI)


// debug flags
#define DBG_MESSAGES 1
#define DBG_NO_DISTANCE_REGULATOR 2
#define DBG_NO_ROTATION_REGULATOR 4


enum State
{
	STATUS_IDLE = 'I',
	STATUS_MOVING = 'M',
	STATUS_ROTATING = 'R',
	STATUS_STUCK = 'S',
	STATUS_ERROR = 'E'
};

void debug_flags(int level);

void reset_driver(void);

void set_position(int X, int Y, int orientation);

void send_status_and_position(void);

void set_speed_accel(float v);

void turn_and_go(int Xd, int Yd, unsigned char end_speed, char direction);

void forward(int duzina, unsigned char end_speed);

void rotate_absolute_angle(int angle);

char turn(int angle);

void start_command();

void arc(long Xc, long Yc, int Fi, char direction_angle, char direction);

void move_to(long x, long y, char direction);

void stop(void);

void set_speed(unsigned char tmp);

void set_rotation_speed(unsigned char max_speed, unsigned char max_accel);

enum State get_status(void);

void force_status(enum State);

void set_stuck_on(void);

void set_stuck_off(void);

#endif	/* REGULACIJA_H */

