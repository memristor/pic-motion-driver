#ifndef REGULACIJA_H
#define	REGULACIJA_H


#define PI	3.1415926535897932384626433832795

// Precnik odometrijskog tocka (u milimetrima)
#define R_tocka	81.4

// Rastojanje izmedju odometrijskih tockova (u milimetrima)
#define rastojanje_tockova 286.5

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
#define K1	(long)(4*2048.0f * 2 * rastojanje_tockova / R_tocka)

/*
	Broj inkremenata po 1 milimetru
	8192 - broj inkrementa za obrtaj 360 stepeni tj. pun krug
	2*r*pi - obim tocka tj. put koji tocak predje za 1 pun obrtaj
	8192 [inc] = (r [mm] * 2*pi[rad])*const
	const = 8192 [inc] / (2*r*pi) [mm] => broj inkrementa po milimetru
*/
#define K2	(float)(4*2048.0f / (R_tocka * PI))

#define VMAX K2

#define ERROR 0
#define BREAK 0
#define OK 1

#define PWM_MAX_SPEED 3200

// ovo su neke eksperimentalne konstante, trial & error
#define Gp_D	5.5
#define Gd_D	200

#define Gp_T	3
#define Gd_T	90


#define MILIMETER_TO_DOUBLED_INC(x) ((long long)(x)*K2*2)
#define DEG_TO_INC_ANGLE(x) ((x)*K1/360)
#define INC_TO_DEG_ANGLE(x) ((x)*360/K1)
#define RAD_TO_INC_ANGLE(x) ((x)/(2.0*PI)*K1)
#define RAD_TO_DEG_ANGLE(x) ((x)*180.0/PI)


#define DBG_NO_DISTANCE_REGULATOR 2
#define DBG_NO_ROTATION_REGULATOR 4
/*
	PROVERI ENKODER: Part no:MA5D1N4FBK1SA0; Type no.: sca24-5000-N-04-09-64-01-S-00
*/


enum States
{
	STATUS_IDLE = 0,
	STATUS_MOVING,
	STATUS_ROTATING,
	STATUS_STUCK,
	STATUS_ERROR
};

void debug_level(int level);

void resetDriver(void);

// zadavanje pozicije
void setPosition(int X, int Y, int orientation);

void sendStatusAndPosition(void);

// zadavanje brzine i ubrzanja
void setSpeedAccel(float v);

// funkcija za stizanje u tacku (Xd, Yd)
void gotoXY(int Xd, int Yd, unsigned char krajnja_brzina, char smer);

// funkcija za kretanje pravo s trapezoidnim profilom brzine
void kretanje_pravo(int duzina, unsigned char krajnja_brzina);

// funkcija za dovodjenje robota u zeljenu apsolutnu orjentaciju
void rotiraj_robota_apsolutni_ugao(int ugao);

// funkcija za okretanje oko svoje ose s trapezoidnim profilom brzine
char okret(int ugao);

void testing();

void start_command();

// funkcija za kretanje po kruznoj putanji
void luk(long Xc, long Yc, int Fi, char smer_ugla, char smer);

void move_to(long x, long y, char direction);

void stop(void);

void setSpeed(unsigned char tmp);

void setRotationSpeed(unsigned char max_speed, unsigned char max_accel);

enum States getStatus(void);

void forceStatus(enum States);

void iskljuciZaglavljivanje(void);

void ukljuciZaglavljivanje(void);

#endif	/* REGULACIJA_H */

