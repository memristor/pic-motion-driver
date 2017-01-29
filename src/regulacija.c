#include "regulacija.h"
#include "sinus.h"
#include "uart.h"
#include "pwm.h"
#include "init.h"
#include <stdint.h>
#include <p33FJ128MC802.h>
#include <libpic30.h>
#include <math.h>

//POMOCNE PROMENLJIVE:
static int stuck_off=0;

static const int stuck_wait1 = 600, stuck_wait2 = 200;

// setuje se u f-ji setSpeed
static unsigned char brzinaL = 0x80;
static float vmax, accel;

// menjaju se sa setSpeedAccel()
/*
	omega - put koji jedan tocak predje vise od drugog po milisekundi
	tj. length luka = wheel_distance * ugao
	alfa - current_speed promene omege
*/
static float omega, alfa;

// menjaju se u interruptu (treba volatile)
static volatile long double Xlong = 0, Ylong = 0;
static volatile long X = 0, Y = 0;
static volatile unsigned long sys_time = 0;
static volatile float orientation = 0;
static volatile long prev_orientation = 0;
static volatile long double positionL;
static volatile long double positionR;
static volatile long double L = 0;      //Milos: predjena ukupna distanca 
//PROMENLJIVE POTREBNE ZA REGULACIJU
static volatile long t_ref = 0, d_ref = 0;
static volatile long keep_rotation = 0, keep_speed = 0, keep_count = 0;


static const long K1_p = K1;
static long prevPositionL,prevPositionR;
static volatile signed long current_speed, angular_speed;

static int debug = 0xff;

static enum States currentStatus = STATUS_IDLE;

// ---------------- POMOCNE FUNKCIJE ------------

static inline void LeviPWM(unsigned int PWM)
{
	P2DC1 = PWM;
}

static inline void DesniPWM(unsigned int PWM)
{
	P1DC1 = PWM;
}

inline long absl(long a)
{
	return a >= 0 ? a : -a;
}

inline long long clipll(long long a, long long b, long long value) {
	if(value <= a)
		value = a;
	else if(value > b)
		value = b;
	return value;
}

int angle_range_fix(int angle) {
	while(angle > 180)
		angle -= 360;
	while(angle < -180)
		angle += 360;
	return angle;
}

float rad_angle_range_fix(float angle) {
	while(angle > PI)
		angle -= 2*PI;
	while(angle < -PI)
		angle += 2*PI;
	return angle;
}

long inc_angle_range_fix(long angle) {
	while(angle > K1/2)
		angle -= K1;
	while(angle < -K1/2)
		angle += K1;
	return angle;
}

long sign(long x) {
	return x >= 0L ? 1 : -1;
}

float minf(float a, float b) {
	return a < b ? a : b;
}
float maxf(float a, float b) {
	return a > b ? a : b;
}
long minl(long a, long b) {
	return a < b ? a : b;
}
long maxl(long a, long b) {
	return a > b ? a : b;
}

static float getDistanceTo(long x, long y) {
	return sqrt((x-X)*(x-X) + (y-Y)*(y-Y));
}

void sin_cos(long theta, long *sint, long *cost) {
	if(theta < SINUS_MAX)
	{
		theta = theta;
		*sint = sinus[theta];
		*cost = sinus[SINUS_MAX-1 - theta];
	}
	else 
	{
		if(theta < 2*SINUS_MAX) // drugi kvadrant
		{
			theta = theta - SINUS_MAX;
			*sint = sinus[SINUS_MAX-1 - theta];
			*cost = -sinus[theta];
		}
		else
			if(theta < 3*SINUS_MAX) // treci kvadrant
			{
				theta = theta - 2*SINUS_MAX;
				*sint = -sinus[theta];
				*cost = -sinus[SINUS_MAX-1 - theta];
			}
			else    // 4. kvadrant
			{
				theta = theta - 3*SINUS_MAX;
				*sint = -sinus[SINUS_MAX-1 - theta];
				*cost = sinus[theta];
			}
	}
}

// --------------------------------------------------------------

// **********************************************************************
// ODOMETRIJA  I REGULACIJA (ide na 1ms)
//pidovanje zavisi od vremena trajanja timerskog interapta
// *********************************************************************
void __attribute__((interrupt(auto_psv))) _T1Interrupt(void)
{//ulazi ovde svakih 1ms,i traje 30000 ciklusa cpu
	sys_time++;
	IFS0bits.T1IF = 0;    // Clear Timer interrupt flag 
	// return;

	static float vR;
	static float vL;
	
	static long sint, cost;
	
	static long long PWML, PWMD;
	
	static signed long commande_distance, commande_rotation;
	
	static long greska;
	static volatile float x, y;
	static volatile float d;
	
	static volatile long theta = 0;

	// ODOMETRIJA, citanje sa enkodera i obrada te informacije
	//************************************************************************
	//CITANJE PODATAKA O POZICIJAMA OBA MOTORA:
	
	vL = -(int)POS1CNT;
	POS1CNT = 0;	//resetujemo brojac inkremenata
	positionL += vL;	//azuriramo trenutnu poziciju

	vR = +(int)POS2CNT;	//ocitavamo broj inkremenata u zadnjoj periodi
	POS2CNT = 0;//resetujemo brojac inkremenata
	positionR += vR;	//azuriramo trenutnu poziciju //MNOZENJE SA KONSTANTOM CM/CL

	//ovde izbaciti PostiionR/L pa proveriti precnik tockova (razliku) NOTE
	L = (positionR + positionL) / 2;//predjena ukupna distanca

	orientation = (positionR - positionL); //% K1;//stvarna ukupna orijentacija robota-ovo treba izbaciti
	
	if (orientation > 0)
	{
		while(orientation > K1_p)
		{
			orientation -= K1_p;
		}
	}
	else
	{
		while( orientation < -K1_p )
		{
			orientation = orientation + K1_p;
		}

	}
	
	if(orientation > K1_p/2)
		orientation -= K1_p;
	if(orientation < -K1_p/2)
		orientation += K1_p;

	//ovde je izbegnuta prva long long operacija
	//PRE// theta = (long long) orientation * 32768 / K1;
	theta = (orientation * 2*SINUS_MAX) / (K1_p / 2);//trans iz ink u ugao

	
	if(theta < 0)
		theta += 4*SINUS_MAX;

	// tabela sinusa ima 8192 elemenata -> prvi kvadrant
	d = vR + vL;	//ovo se kasnije deli sa 2, predjeni put u zadnjih 1ms
	
	//sin_cos(theta, &cost, &sint);
	if(theta < SINUS_MAX)
	{
		theta = theta;
		sint = sinus[theta];
		cost = sinus[SINUS_MAX-1 - theta];
	}
	else 
	{
		if(theta < 2*SINUS_MAX) // drugi kvadrant
		{
			theta = theta - SINUS_MAX;
			sint = sinus[SINUS_MAX-1 - theta];
			cost = -sinus[theta];
		}
		else
			if(theta < 3*SINUS_MAX) // treci kvadrant
			{
				theta = theta - 2*SINUS_MAX;
				sint = -sinus[theta];
				cost = -sinus[SINUS_MAX-1 - theta];
			}
			else    // 4. kvadrant
			{
				theta = theta - 3*SINUS_MAX;
				sint = -sinus[SINUS_MAX-1 - theta];
				cost = sinus[theta];
			}
	}
	// x, y -> predjeno delta po x i y koordinati u zadnjoj periodi,tj 1ms (deltax, deltay)
	x = d * cost;
	y = d * sint;

	Xlong += (x/SINUS_AMPLITUDE);
	Ylong += (y/SINUS_AMPLITUDE);

	//izacunavanje trenutne pozicije [mm]
	// pretvaranje u milimetre,ova X i Y su ono sto dobijamo kad serijskom trazimo poziciju
	X = (((long long)Xlong / 2));
	X /= K2; // pretvaranje u milimetre
	
	Y = (((long long)Ylong / 2));
	Y /= K2; // pretvaranje u milimetre

	//Milos: UPRAVLJANJE: do sada samo odometrija, sada ide upravljanje
	//*************************************************************************
	//regulator rastojanja
	current_speed = (vL + vR) / 2; // srednja vrednost predjenih inkremenata u zadnjoj periodi -> v = s/t, current_speed
	
	if(keep_count > 0) {
		t_ref += keep_rotation;
		d_ref += keep_speed;
		keep_count--;
	}
	
	greska = d_ref - L;
	commande_distance = greska * Gp_D - Gd_D * current_speed; //PD blok


	//regulator orjentacije
	angular_speed = vL - vR; //ugao u zanjih 1ms u inkre

	greska = ((long)orientation - t_ref) % K1_p;

	if(greska > K1_p/2) {
		greska -= K1_p;
	} else if(greska < -K1_p/2) {
		greska += K1_p;
	}

	commande_rotation = greska * Gp_T - angular_speed * Gd_T; //PD blok

	// ukupan PWM dobija se superpozicijom uzimajuci u obzir oba regulatora
	// commande_distance = regulator rastojanja
	// commande_rotation = regulator orijentacije
	if(debug & DBG_NO_DISTANCE_REGULATOR) {
		commande_distance = 0;
		d_ref = L;
	}
	if(debug & DBG_NO_ROTATION_REGULATOR) {
		commande_rotation = 0;
		t_ref = orientation;
	}

	PWML = commande_distance - commande_rotation;
	PWMD = commande_distance + commande_rotation;

	// saturacija,ogranicava max brzinu kretanja
	// if(PWMD <= -PWM_MAX_SPEED)
		// PWMD = -PWM_MAX_SPEED;
	// else if(PWMD >= PWM_MAX_SPEED)
		// PWMD = PWM_MAX_SPEED;
		
	PWMD = clipll(-PWM_MAX_SPEED, PWM_MAX_SPEED, PWMD);

	// if(PWML <= -PWM_MAX_SPEED)
		// PWML = -PWM_MAX_SPEED;
	// else if(PWML >= PWM_MAX_SPEED)
		// PWML = PWM_MAX_SPEED;
		
	PWML = clipll(-PWM_MAX_SPEED, PWM_MAX_SPEED, PWML);
	//izbor directiona,i kontrola hardvera:

	
	if (PWMD >= 0)
	{
		LATBbits.LATB15 = 0;
		DesniPWM(PWMD);
	}
	else
	{
		LATBbits.LATB15 = 1;
		DesniPWM(PWM_MAX_SPEED + PWMD);
	}

	if(PWML >= 0)
	{
		LATBbits.LATB9 = 0;
		LeviPWM(PWML);
	}
	else
	{
		LATBbits.LATB9 = 1;
		LeviPWM(PWM_MAX_SPEED + PWML);
	}

	IFS0bits.T1IF = 0;    // Clear Timer interrupt flag 
}

void resetDriver(void)
{
	// inicijalizacija parametara:
	positionR = positionL = 0;
	L = orientation = 0;

	PWMinit();
	setSpeed(0x32); //setSpeed(0x80);
	// setSpeedAccel(K2);	//K2 je za 1m/s /bilo je 2
	// da li ima smisla?
	setPosition(0, 0, 0);
	currentStatus = STATUS_IDLE;
}

void wait_for_regulator_timer_interrupt() {
	unsigned long t = sys_time;
	while(sys_time == t);
}

// zadavanje X koordinate
static void setX(int tmp)
{
	Xlong = (long long)tmp * 2 * K2;
	d_ref = L;
	t_ref = orientation;

	wait_for_regulator_timer_interrupt();
}

// zadavanje Y koordinate
static void setY(int tmp)
{
	Ylong = (long long)tmp * 2 * K2;
	d_ref = L;
	t_ref = orientation;

	wait_for_regulator_timer_interrupt();
}


// zadavanje orientacije
static void setO(int tmp)
{
	positionL = -(tmp * K1 / 360) / 2;
	positionR = (tmp * K1 / 360) / 2;

	// ovo se pokrati pa je L = 0
	// L = (positionR + positionL) / 2;
	L = 0;
	orientation = (long int)(positionR - positionL) % K1;

	d_ref = L;
	t_ref = orientation;

	wait_for_regulator_timer_interrupt();
}

void setPosition(int X, int Y, int orientation)
{
	setX(X);
	setY(Y);
	setO(orientation);
	currentStatus = STATUS_IDLE;
}


// konverzija stanja u ASCII za slanje
char state_to_ascii(enum States state) {
	const char * state_to_ascii_table = "IMRSE";
	return state_to_ascii_table[state];
}

void sendStatusAndPosition(void)
{
	start_packet('P');
		put_byte(state_to_ascii(currentStatus));
		put_word(X);
		put_word(Y);
		put_word(orientation * 360 / K1 + 0.5);
		put_word(current_speed);
	end_packet();
	
	return;
	int tmpO;
	putch(SYNC_BYTE);
	putch(SYNC_BYTE);
	putch(SYNC_BYTE);
	putch(state_to_ascii(currentStatus));
	putint16(X);
	putint16(Y);
	tmpO = orientation * 360 / K1 + 0.5;
	putint16(tmpO);
	putint16(current_speed);
}

void setSpeedAccel(float v)
{
	vmax = v;
	omega = 2 * vmax;
	if(vmax < (VMAX * 161 / 256))
		// za 500 ms ubrza do vmax
		accel = vmax / (500 /*[ms]*/);
	else
		// za 375 ms ubrza do vmax
		accel = vmax / (375 /*[ms]*/);
	alfa = 2 * accel;
}


enum States getStatus(void)
{
	return currentStatus;
}

void forceStatus(enum States newStatus)
{
	currentStatus = newStatus;
}

void start_command() {
	keep_count = 1;
}

// citanje serijskog porta tokom kretanja
static char getCommand(void)
{
	char command;
	int i;
	// U1STAbits.OERR = 0;
	if(UART_CheckRX()) // proverava jel stigao karakter preko serijskog porta
	{
		// command = getch();
		uint8_t len;
		if(!try_read_packet((uint8_t*)&command, &len)) return OK;

		switch(command)
		{           
			case 'P':
				sendStatusAndPosition();
				break;

			case 'S':
				//ukopaj se u mestu
				for(i=0;i<20;i++){
					d_ref = L;
					t_ref = orientation;
					__delay_ms(10);
				}


				PWMinit();
				currentStatus = STATUS_IDLE;
				__delay_ms(10);

				return ERROR;

			case 's':
				// stani i ugasi PWM
				d_ref = L;
				t_ref = orientation;

				CloseMCPWM();
				currentStatus = STATUS_IDLE;
				__delay_ms(10);

				return ERROR;

			case 'V':
				// setSpeed(getch());
				setSpeed(get_byte());
				break;
			
			case 'i':
				// currentStatus = STATUS_MOVING;
				keep_count = 5;
				keep_speed = current_speed;
				keep_rotation = angular_speed;
				return BREAK;

			case 'G' : case 'D' : case 'T' : case 'A' : case 'Q':
			default:
				// TODO: queue command
				// primljena komanda za kretanje u toku kretanja

				// stop, status ostaje MOVING

				// d_ref = L;
				// t_ref = orientation;

				// __delay_ms(120);

				// d_ref = L;
				// t_ref = orientation;

				// __delay_ms(20);
				return OK;

		}// end of switch(command)
	}

	return OK;
}

static char check_stuck_condition(void)
{
	static int zaglav=0;
	static unsigned long prev_sys_time=0;
	// otprlike svakih 10ms
	if((sys_time-prev_sys_time)>=10)
	{
		if((absl((long)positionL - prevPositionL) < (15+brzinaL*0.75) ) || (absl((long)positionR - prevPositionR) < (15+brzinaL*0.75) )) 
		{
			if(!stuck_off)
				zaglav++;
			if(zaglav == 5)
			{
				zaglav=0;
				d_ref = L;
				t_ref = orientation;
				stop();
				currentStatus = STATUS_STUCK;
				__delay_ms(50);
				return STATUS_STUCK;
			} 
		}
		else
		{
			zaglav = 0;
			prevPositionL = positionL;
			prevPositionR = positionR;
		}
		prev_sys_time=sys_time;
	}

	return OK;
}

static char check_stuck_condition_angle(void)
{
	static int stuck=0;
	static unsigned long prev_sys_time=0;
	// 10ms periodical
	if( sys_time - prev_sys_time >= 10 )
	{
		if( absl((long)orientation-prev_orientation) < 20+1.4*brzinaL)
		{
			if(!stuck_off)
				stuck++;
				
			if(stuck == 5)
			{
				stuck=0;
				d_ref = L;
				t_ref = orientation;
				stop();
				currentStatus = STATUS_STUCK;
				__delay_ms(50);
				return STATUS_STUCK;
			}
		}
		else
		{
			// not stuck
			stuck = 0;
			// pamti trenutnu poziciju
			prev_orientation = orientation;
			prevPositionL = positionL;
			prevPositionR = positionR;
		}
		prev_sys_time=sys_time;
	}
	return OK;
}


// gadjaj tacku (Xd, Yd)
void turn_and_go(int Xd, int Yd, unsigned char end_speed, char direction)
{
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long L_dist, L0, L1, L2, L3;
	long D0, D1, D2;
	float v_vrh, v_end, v0;
	int length, ugao;
	long long int Xdlong, Ydlong;
	
	Xdlong = (long long)Xd * K2 * 2;
	Ydlong = (long long)Yd * K2 * 2;

	d_ref=L;
	// v_ref=0;
	v0 = current_speed;
	direction = (direction >= 0 ? 1 : -1);

	//okreni se prema krajnjoj tacki, nadji ugao gde treba srenuti
	ugao = atan2(Ydlong-Ylong, Xdlong-Xlong) * (180 / PI) - orientation * 360 / K1;
	
	if(direction < 0)
		ugao += 180;
	
	ugao = angle_range_fix(ugao);

	if(turn(ugao) == ERROR)
		return;

	// length = sqrt((X - Xd) * (X - Xd) + (Y - Yd) * (Y - Yd)); //put koji treba preci
	length = getDistanceTo(Xd, Yd); //put koji treba preci

	if((length < 100) && (vmax > K2/32)) {
		setSpeedAccel(VMAX/3);// OVO JE DODATO
	}

	//forward(length, end_speed);
	v_end = vmax * end_speed / 256;
	
	//pretvaranje u inkrem
	L_dist = (long)(length * K2);

	// racunanje koliko ce trajati svaka faza
	T1 = (vmax - current_speed) / accel;
	L0 = L;
	L1 = current_speed * T1 + accel * T1 * T1 / 2;

	T3 = (vmax - v_end) / accel;
	L3 = vmax * T3 - accel * T3 * T3 / 2;


	if( (L1 + L3) < L_dist)
	{
		//moze da dostigne vmax
		L2 = L_dist - L1 - L3;
		T2 = L2 / vmax;
	}
	else
	{
		//ne moze da dostigne vmax
		T2 = 0;
		v_vrh = sqrt(accel * L_dist + (current_speed * current_speed + v_end * v_end) / 2);
		if( (v_vrh < current_speed) || (v_vrh < v_end) )
		{
			currentStatus = STATUS_ERROR;
			return; //mission impossible
		}

		T1 = (v_vrh - current_speed) / accel;
		T3 = (v_vrh - v_end) / accel;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	D0 = d_ref;


	currentStatus = STATUS_MOVING;
	while(t < t3) {

		if(t == sys_time) continue;
		
		t = sys_time;
			
		if(getCommand() == ERROR) {
			return;
		}
			
		if( t > (t0+stuck_wait1) && t < (t3-stuck_wait2) )
		{
			if (check_stuck_condition() == STATUS_STUCK)
				return;
		}
		
		if(t <= t2) // obnavlja referencu ugla,tj proverava da li je dodatno skrenuo
		{
			if(direction > 0)
				t_ref = (atan2(Ydlong-Ylong, Xdlong-Xlong) / (2 * PI)) * K1;
			// ovo ispod je matematicki ekvivalentno ovom iznad
			else
				t_ref = (atan2(Ylong-Ydlong, Xlong-Xdlong) / (2 * PI)) * K1;
		}

		if(t <= t1) // faza ubrzanja
		{
			// v_ref = v0 + accel * (t-t0);
			D1 = D2 = d_ref = D0 + direction * (v0 * (t-t0) + accel * (t-t0)*(t-t0)/2);
		}
		else if(t <= t2) // faza konstantne brzine
		{
			// v_ref = vmax;
			D2 = d_ref = D1 + direction * vmax * (t-t1);
		}
		else if(t <= t3) // faza usporenja
		{
			// v_ref = vmax - accel * (t-t2);
			d_ref = D2 + direction * (vmax * (t-t2) - accel * (t-t2) * (t-t2) / 2);
		}
	}
	currentStatus = STATUS_IDLE;
}


// funkcija za kretanje pravo s trapezoidnim profilom brzine
void forward(int length, unsigned char end_speed)
{
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long L_dist, L0, L1, L2, L3;
	long D0, D1, D2;
	float v_vrh, v_end, v0, v_ref;
	char predznak;

	//Ne moze na manjim rastojanjima od 50cm da ide brzimom koju zelis, ograniceno je na /3 brzine
	//    if((length < 700) && (length > -700))
	//      setSpeedAccel(K2 / 3);//OVO je bilo zakoment.
	d_ref=L;
	v_ref=0;// PROVERITI DA LI ZBOG OVOGA LUDI!!!
	v0 = v_ref;
	v_end = vmax * end_speed / 256;
	predznak = (length >= 0) ? 1 : -1;
	L_dist = (long)length * K2; // konverzija u inkremente

	T1 = (vmax - v_ref) / accel;
	L0 = L;
	L1 = v_ref * T1 + accel * T1 * (T1 / 2);

	T3 = (vmax - v_end) / accel;
	L3 = vmax * T3 - accel * T3 * (T3 / 2);

	if((L1 + L3) < (long)predznak * L_dist)
	{
		// can reach
		L2 = predznak * L_dist - L1 - L3;
		T2 = L2 / vmax;
	}
	else
	{
		// can't reach vmax
		T2 = 0;
		v_vrh = sqrt(accel * predznak * L_dist + (v_ref * v_ref + v_end * v_end) / 2);
		if((v_vrh < v_ref) || (v_vrh < v_end))
		{
			currentStatus = STATUS_ERROR;
			return; //mission impossible
		}

		T1 = (v_vrh - v_ref) / accel;
		T3 = (v_vrh - v_end) / accel;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	D0 = d_ref;

	currentStatus = STATUS_MOVING;
	while(t < t3)  {
		
		if(t == sys_time) continue;
		
		t = sys_time;
		if(getCommand() == ERROR)
			return;

		if( t > (t0+stuck_wait1) && t < (t3-stuck_wait2) )   
		{
			if (check_stuck_condition() == STATUS_STUCK)
				return;
		}
		
		if(t <= t1)
		{
			// v_ref = v0 + accel * (t-t0);
			D1 = D2 = d_ref = D0 + predznak * (v0 * (t-t0) + accel * (t-t0)*(t-t0)/2);
		}
		else if(t <= t2)
		{
			// v_ref = vmax;
			D2 = d_ref = D1 + predznak * vmax * (t-t1);
		}
		else if(t <= t3)
		{
			// v_ref = vmax - accel * (t-t2);
			d_ref = D2 + predznak * (vmax * (t-t2) - accel * (t-t2) * (t-t2) / 2);
		}
	}
	
	currentStatus = STATUS_IDLE;
}

// funkcija za dovodjenje robota u zeljenu apsolutnu orientaciju
void rotate_absolute_angle(int ugao)
{
	int tmp = ugao - orientation * 360 / K1;
	tmp = angle_range_fix(tmp);

	turn(tmp);
}

// funkcija za okretanje oko svoje ose s trapezoidnim profilom brzine
char turn(int ugao)
{
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long Fi_total, Fi1;
	float ugao_ref, w_ref = 0;
	char predznak;

	predznak = (ugao >= 0 ? 1 : -1);

	Fi_total = (long)ugao * K1 / 360;

	T1 = T3 = omega / alfa;
	Fi1 = alfa * T1 * T1 / 2;
	if(Fi1 > (predznak * Fi_total / 2))
	{
		//trougaoni profil grafika brzine (od 0 ubrzava do max, pa usporava do 0)
		Fi1 = predznak  * Fi_total / 2;
		T1 = T3 = sqrt(2 * Fi1 / alfa);
		T2 = 0;
	}
	else
	{
		//trapezni profil grafika brzine (postoji period konstantne maksimalne brzine)
		T2 = (predznak * Fi_total - 2 * Fi1) / omega;
	}

	ugao_ref = t_ref;
	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;

	currentStatus = STATUS_ROTATING;
	while(t < t3)
		if(t != sys_time) // svakih 1 ms
		{
			t = sys_time;
			if(getCommand() == ERROR)
				return ERROR;

			if( t > (t0+stuck_wait1) && t < (t3-stuck_wait2) )
			{
				if (check_stuck_condition_angle() == STATUS_STUCK)
					return ERROR;
			}
			if(t <= t1)
			{
				w_ref += alfa;
				ugao_ref += predznak * (w_ref - alfa / 2);
				t_ref = ugao_ref;
			}
			else if(t <= t2)
			{
				w_ref = omega;
				ugao_ref += predznak * omega;
				t_ref = ugao_ref;
			}
			else if(t <= t3)
			{
				w_ref -= alfa;
				ugao_ref += predznak * (w_ref + alfa / 2);
				t_ref = ugao_ref;
			}
		}

	currentStatus = STATUS_IDLE;

	return OK;
}



void luk(long Xc, long Yc, int Fi, char direction_ugla, char direction)
{
	float R, Fi_pocetno, delta, luk;
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long Fi_total, Fi1;
	float v_poc, dist_ref, ugao_ref, w_ref = 0, v_ref = 0;
	char sign;
	int angle;


	if (direction_ugla)
		sign = 1;
	else 
		sign = -1;

	// R = sqrt(((X-Xc) * (X-Xc) + (Y-Yc) * (Y-Yc)));
	R = getDistanceTo(Xc, Yc);
	Fi_pocetno = atan2(((int)Y-(int)Yc), ((int)X-(int)Xc));
	angle = Fi_pocetno * 180 / PI;
	direction = (direction >= 0 ? 1 : -1);

	angle = (angle + direction * sign * 90) % 360;
	angle = angle_range_fix(angle);

	angle -= orientation * 360 / K1;
	angle %= 360;

	angle = angle_range_fix(angle);
	
	if(turn(angle))
		return;

	v_poc = vmax;

	Fi_total = (long)Fi * K1 / 360;

	T1 = T3 = omega / alfa;
	Fi1 = alfa * T1 * T1 / 2;
	if(Fi1 > (sign * Fi_total / 2))
	{
		//trougaoni profil grafika brzine
		Fi1 = sign  * Fi_total / 2;
		T1 = T3 = sqrt(2 * Fi1 / alfa);
		T2 = 0;
	}
	else
	{
		//trapezni profil grafika brzine
		T2 = (sign * Fi_total - 2 * Fi1) / omega;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	ugao_ref = t_ref;
	dist_ref = d_ref;

	currentStatus = STATUS_MOVING;
	while(t < t3) 
	{
		if(t == sys_time) continue;
		
		t = sys_time;
		if(getCommand() == ERROR)
		{
			setSpeedAccel(v_poc);
			return;
		}

		if( t > (t0+stuck_wait1) && t < (t3-stuck_wait2) )
		{
			if(check_stuck_condition() == STATUS_STUCK)
			{
				setSpeedAccel(v_poc);
				return;
			}
		}

		if(t <= t1)
		{
			w_ref += alfa;
			v_ref += accel;
			delta = sign * (w_ref - alfa / 2);
			luk = sign * delta * R / wheel_distance;
			ugao_ref += delta;
			dist_ref += direction * luk;
			t_ref = ugao_ref;
			d_ref = dist_ref;
		}
		else if(t <= t2)
		{
			w_ref = omega;
			v_ref = vmax;
			delta = sign * omega;
			luk = sign * delta * R / wheel_distance;
			ugao_ref += delta;
			dist_ref += direction * luk;
			t_ref = ugao_ref;
			d_ref = dist_ref;
		}
		else if(t <= t3)
		{
			w_ref -= alfa;
			v_ref -= accel;
			delta = sign * (w_ref + alfa / 2);
			luk = sign * delta * R / wheel_distance;
			ugao_ref += delta;
			dist_ref += direction * luk;
			t_ref = ugao_ref;
			d_ref = dist_ref;
		}
	}
	setSpeedAccel(v_poc);
	currentStatus = STATUS_IDLE;
}

long get_inc_angle_diff(long a, long b) {
	long d = a - b;
	if(d > K1/2)
		d -= K1;
	else if(d < -K1/2)
		d += K1;
	return d;
}

int get_deg_angle_diff(int a, int b) {
	long d = a - b;
	if(d > 180)
		d -= 360;
	else if(d < -180)
		d += 360;
	return d;
}


void debug_level(int level) {
	debug = level;
}

/*
	@function: move_to
	direction:
		0 - pick smallest rotation
		1 - forward
		-1 - backward
*/
void move_to(long x, long y, char direction) {
	float speed = current_speed;
	float rotation_speed = angular_speed; /* [inc/ms] */
	// float acceleration = accel; /* [inc/ms] */
	
	// long min_angle = DEG_TO_INC_ANGLE(30);
	long long int Xdlong, Ydlong;
	
	float dist = getDistanceTo(x,y);
	Ydlong = MILIMETER_TO_DOUBLED_INC(y);
	Xdlong = MILIMETER_TO_DOUBLED_INC(x);
	long goal_angle;
	long angle_diff;
	goal_angle = RAD_TO_INC_ANGLE(atan2(y-Y, x-X));
	angle_diff = get_inc_angle_diff(goal_angle, orientation);
	
	if(direction > 0) direction = 1;
	else if(direction < 0) direction = -1;
	
	if(direction == 0) {
		// determine direction automatically
		if(absl(angle_diff) > K1/4) {
			direction = 1;
		} else {
			direction = -1;
		}
	}
	
	currentStatus = STATUS_MOVING;
	
	long D = d_ref;
	long R = t_ref;
	float min_speed = VMAX * 0x10 / 255;
	int e = 0;
	if(debug & 1) {
		putch('\r');
		dbg(0xff,0);
	}
	
	// long t0 = sys_time, t;
	start_command();
	while(1)
	{
		if(getCommand() == ERROR)
			return;
			
		wait_for_regulator_timer_interrupt();
		
		// t = sys_time;
		/*
		if( t > t0 + stuck_wait1 )
		{
			if(check_stuck_condition() == STATUS_STUCK)
			{
				// setSpeedAccel(v_poc);
				return;
			}
		}
		*/
		
		goal_angle = RAD_TO_INC_ANGLE( atan2(y-Y, x-X) );
		long orient = orientation;
		
		if(direction == -1) {
			orient = inc_angle_range_fix( orient + DEG_TO_INC_ANGLE( 180 ) );
		}
		
		angle_diff = get_inc_angle_diff(goal_angle, orient);
		
		long abs_angle_diff = absl(angle_diff);
		
		if(abs_angle_diff > DEG_TO_INC_ANGLE(20)) {
			rotation_speed = minf( rotation_speed + alfa/2, omega/2 );
		} else if(abs_angle_diff > DEG_TO_INC_ANGLE(3)){
			rotation_speed = maxf( rotation_speed - alfa/2, min_speed );
		} else {
			R = orientation;
		}
		
		dist = getDistanceTo(x,y);
		
		if(dist > 200.0f && abs_angle_diff < DEG_TO_INC_ANGLE(65)) {
			if(speed < vmax) {
				speed = maxf( min_speed, speed+accel );
			}
		} else if(dist > 1.0f && dist <= 200.0f) {
			speed = maxf( min_speed, speed-accel );
		} else if(dist < 1.0f) {
			d_ref = L;
			return;
		}
		
		
		if( (debug & 1) && (++e > 25) ) {
			putch('\r');
			dbg(0, speed);
			dbg(1, angle_diff);
			dbg(2, rotation_speed);
			dbg(3, (int)dist);
			dbg(4, orientation);
			dbg(5, goal_angle);
			dbg(6, x);
			dbg(7, y);
			dbg(8, RAD_TO_DEG_ANGLE( atan2(y-Y, x-X) ));
			dbg(25, X);
			dbg(26, Y);
			e = 0;
		}
		
		D += direction * speed;
		R += sign(angle_diff) * rotation_speed;
		
		d_ref = D;
		t_ref = R;
	}
}


void stop(void)
{
	d_ref = L;
	t_ref = orientation;

	__delay_ms(120);

	d_ref = L;
	t_ref = orientation;

	__delay_ms(20);

	currentStatus = STATUS_IDLE;
}

void setRotationSpeed(unsigned char max_speed, unsigned char max_accel) {
	omega = VMAX * (unsigned char)max_speed / 256;
	if(omega < (VMAX * 161 / 256))
		// za 500 ms ubrza do vmax
		alfa = omega / (500 /*[ms]*/);
	else
		// za 375 ms ubrza do vmax
		alfa = omega / (375 /*[ms]*/);
}

void setSpeed(unsigned char tmp)
{
	brzinaL = tmp;
	/*
		K2 (broj inkrementa / mm) i current_speed (mm/ms) nemaju veze, ovo je lupljeno.
		Ali ovo radi jer daje otprilike dobre brojeve sa maksimalnom brzinom oko 1 m/s
		(naravno ukoliko motori zaista podrzavaju ovu brzinu).
		K2 je otprilike 32 inkrementa po milimetru, ili 32000 po sekundi, a to je
		32000/8192 = 3.9 punih obrtaja tocka koji je obima 81.4*pi = 255.725, i to 
		pomnozeno sa 3.9 daje 997.3275 sto je 1 metar, znaci maksimalna brzina je 1 m/s
	*/
	setSpeedAccel(VMAX * (unsigned char)brzinaL / 256);
}


void iskljuciZaglavljivanje(void) {
	stuck_off=1;
}

void ukljuciZaglavljivanje(void) {
	stuck_off=0;
}
