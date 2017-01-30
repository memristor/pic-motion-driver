#include "regulator.h"
#include "sinus.h"
#include "uart.h"
#include "pwm.h"
#include "init.h"
#include <stdint.h>
#include <p33FJ128MC802.h>
#include <libpic30.h>
#include <math.h>

// stuck check
// TODO: remove (replaced with general stuck condition)
/*
static int stuck_off=0;
static const int stuck_wait1 = 600, stuck_wait2 = 200;
static long prev_orientation = 0;
static long prev_positionL, prev_positionR;
static unsigned char speedL = 0x80;
*/
//


static float vmax, accel;

/*
	omega - arc length / ms = wheel_distance * angle
	alpha - omega change / ms
*/
static float omega, alpha;

// changes in interrupt, so here are all volatiles
static volatile long double Xlong = 0, Ylong = 0;
static volatile long X = 0, Y = 0;
static volatile signed long current_speed, angular_speed;

static volatile unsigned long sys_time = 0;

static volatile long double positionL;
static volatile long double positionR;

// ----- variables for controlling robot -----
static volatile float orientation = 0;
static volatile long double L = 0; // total distance
static volatile long t_ref = 0, d_ref = 0;

// stuck detection in regulator
static long prev_orientation = 0;
static long prev_L = 0;
static int t_ref_fail_count = 0;
static int d_ref_fail_count = 0;
//

static volatile long keep_rotation = 0, keep_speed = 0, keep_count = 0;
// -------------------------------

static int16_t send_status_counter = 0;
static int16_t send_status_interval;

#define K1_p ((long)K1)


static uint8_t control_flags = 0xff & ~CONTROL_FLAG_NO_STUCK;

static enum State current_status = STATUS_IDLE;

// ---------------- HELPER FUNCTIONS ------------

static inline void left_pwm(unsigned int PWM)
{
	P2DC1 = PWM;
}

static inline void right_pwm(unsigned int PWM)
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

int sign(int x) {
	return x >= 0L ? 1 : -1;
}
long signl(long x) {
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

static float get_distance_to(long x, long y) {
	return sqrt((x-X)*(x-X) + (y-Y)*(y-Y));
}

void sin_cos(long theta, long *sint, long *cost) {
	if(theta < SINUS_MAX) // 1st quadrant
	{
		theta = theta;
		*sint = sinus[theta];
		*cost = sinus[SINUS_MAX-1 - theta];
	}
	else 
	{
		if(theta < 2*SINUS_MAX) // 2nd quadrant
		{
			theta = theta - SINUS_MAX;
			*sint = sinus[SINUS_MAX-1 - theta];
			*cost = -sinus[theta];
		}
		else
			if(theta < 3*SINUS_MAX) // 3rd quadrant
			{
				theta = theta - 2*SINUS_MAX;
				*sint = -sinus[theta];
				*cost = -sinus[SINUS_MAX-1 - theta];
			}
			else    // 4th quadrant
			{
				theta = theta - 3*SINUS_MAX;
				*sint = -sinus[SINUS_MAX-1 - theta];
				*cost = sinus[theta];
			}
	}
}

// --------------------------------------------------------------

// **********************************************************************
// ODOMETRY AND REGULATION (every 1ms)
// enters periodically every 1ms, and can last 30000 cpu cycles max (but recommended to last at most half of that, 15000 cycles)
// *********************************************************************
void __attribute__((interrupt(auto_psv))) _T1Interrupt(void)
{
	sys_time++;
	IFS0bits.T1IF = 0;    // Clear Timer interrupt flag 

	static float vR;
	static float vL;
	
	static long sint, cost;
	
	static long long PWML, PWMD;
	
	static signed long regulator_distance, regulator_rotation;
	
	static long error;
	static volatile float x, y;
	static volatile float d;
	
	static volatile long theta = 0;

	// ODOMETRY, reading encoders and processing them
	//************************************************************************
	
	// read left encoder
	vL = -(int)POS1CNT;
	POS1CNT = 0;
	positionL += vL;

	// read right encoder
	vR = +(int)POS2CNT;
	POS2CNT = 0;
	positionR += vR;

	/* FIXME: moving too much can cause overflow when passed distance is too big
	 		  can be fixed by resetting values when crosses some value, but maintain references
			  and orientation
	*/
	L = (positionR + positionL) / 2;

	// FIXME: rotating too much can cause overflow
	orientation = (positionR - positionL);
	
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
	
	if(orientation > K1_p/2) {
		orientation -= K1_p;
	}
	if(orientation < -K1_p/2) {
		orientation += K1_p;
	}


	theta = (orientation * 2*SINUS_MAX) / (K1_p / 2);

	
	if(theta < 0)
		theta += 4*SINUS_MAX;

	d = vR + vL;
	
	// TODO: check if sin_cos function works well, and use it instead of this bloat
	// sin_cos(theta, &cost, &sint);
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
	
	x = d * cost;
	y = d * sint;

	Xlong += (x/SINUS_AMPLITUDE);
	Ylong += (y/SINUS_AMPLITUDE);

	// TODO: check if correct
	X = DOUBLED_INC_TO_MILIMETER(Xlong);
	Y = DOUBLED_INC_TO_MILIMETER(Ylong);

	// REGULATOR
	//*************************************************************************
	
	current_speed = (vL + vR) / 2; // v = s/t, current_speed [inc/ms]
	
	if(keep_count > 0) {
		t_ref += keep_rotation;
		d_ref += keep_speed;
		keep_count--;
		if(keep_count == 0) {
			current_status = STATUS_IDLE;
		}
	}
	
	error = d_ref - L;
	regulator_distance = error * Gp_D - Gd_D * current_speed; // PD (proportional, differential) regulator

	// stuck detection for distance
	if( !(control_flags & CONTROL_FLAG_NO_STUCK) ) {
		if(absl(error) > STUCK_DISTANCE_JUMP_ERROR_THRESHOLD) {
			current_status = STATUS_STUCK;
		}
		if(absl(L - prev_L) < absl(error) * STUCK_DISTANCE_ERROR_RATIO) {
			if(++d_ref_fail_count > STUCK_DISTANCE_MAX_FAIL_COUNT) {
				current_status = STATUS_STUCK;
				d_ref_fail_count = 0;
			}
		} else {
			d_ref_fail_count = 0;
		}
	}
	//

	// ------------ rotation regulator -------------
	angular_speed = vL - vR; // angular speed [inc/ms]

	error = ((long)orientation - t_ref) % K1_p;

	if(error > K1_p/2) {
		error -= K1_p;
	} else if(error < -K1_p/2) {
		error += K1_p;
	}

	regulator_rotation = error * Gp_T - angular_speed * Gd_T; // PD (proportional, differential) regulator
	
	// stuck detection for rotation
	
	
	if( !(control_flags & CONTROL_FLAG_NO_STUCK) ) {
		if(absl(error) > STUCK_ROTATION_JUMP_ERROR_THRESHOLD) {
			current_status = STATUS_STUCK;
		}
		if(absl(orientation - prev_orientation) < absl(error) * STUCK_ROTATION_ERROR_RATIO) {
			if(++t_ref_fail_count > STUCK_ROTATION_MAX_FAIL_COUNT) {
				current_status = STATUS_STUCK;
				t_ref_fail_count = 0;
			}
		} else {
			t_ref_fail_count = 0;
		}
		
		// if robot stuck, then shut down engines
		if(current_status == STATUS_STUCK) {
			control_flags |= CONTROL_FLAG_NO_DISTANCE_REGULATOR | CONTROL_FLAG_NO_ROTATION_REGULATOR;
		}
	}
	//
	
	prev_orientation = orientation;
	prev_L = L;
	
	
	if(control_flags & CONTROL_FLAG_NO_DISTANCE_REGULATOR) {
		regulator_distance = 0;
		d_ref = L;
	}
	if(control_flags & CONTROL_FLAG_NO_ROTATION_REGULATOR) {
		regulator_rotation = 0;
		t_ref = orientation;
	}

	// ------ final PWM is result of superposition of 2 regulators --------
	PWML = regulator_distance - regulator_rotation;
	PWMD = regulator_distance + regulator_rotation;

	// TODO: check if correct
	PWMD = clipll(-PWM_MAX_SPEED, PWM_MAX_SPEED, PWMD);
	PWML = clipll(-PWM_MAX_SPEED, PWM_MAX_SPEED, PWML);

	// apply right pwm
	if (PWMD >= 0)
	{
		LATBbits.LATB15 = 0;
		right_pwm(PWMD);
	}
	else
	{
		LATBbits.LATB15 = 1;
		right_pwm(PWM_MAX_SPEED + PWMD);
	}

	// apply left pwm
	if(PWML >= 0)
	{
		LATBbits.LATB9 = 0;
		left_pwm(PWML);
	}
	else
	{
		LATBbits.LATB9 = 1;
		left_pwm(PWM_MAX_SPEED + PWML);
	}

	IFS0bits.T1IF = 0;    // Clear Timer interrupt flag 
}

void reset_driver(void)
{
	positionR = positionL = 0;
	L = orientation = 0;

	PWMinit();
	set_speed(0x32);
	set_position(0, 0, 0);
	current_status = STATUS_IDLE;
}

void wait_for_regulator() {
	unsigned long t = sys_time;
	while(sys_time == t);
}


static void setX(int tmp)
{
	Xlong = (long long)tmp * 2 * K2;
	d_ref = L;
	t_ref = orientation;

	wait_for_regulator();
}

static void setY(int tmp)
{
	Ylong = (long long)tmp * 2 * K2;
	d_ref = L;
	t_ref = orientation;

	wait_for_regulator();
}


static void setO(int tmp)
{
	positionL = -(tmp * K1 / 360) / 2;
	positionR = (tmp * K1 / 360) / 2;

	L = 0;
	orientation = (long int)(positionR - positionL) % K1;

	d_ref = L;
	t_ref = orientation;

	wait_for_regulator();
}

void set_position(int X, int Y, int orientation)
{
	setX(X);
	setY(Y);
	setO(orientation);
	current_status = STATUS_IDLE;
}

void send_status_and_position(void)
{
	start_packet('P');
		put_byte(current_status);
		put_word(X);
		put_word(Y);
		put_word(orientation * 360 / K1 + 0.5);
		put_word(current_speed);
	end_packet();
}

void set_speed_accel(float v)
{
	vmax = v;
	omega = 2 * vmax;
	if(vmax < (VMAX * 161 / 256))
		// in 500 ms speeds up to vmax
		accel = vmax / (500 /*[ms]*/);
	else
		// in 375 ms speeds up to vmax
		accel = vmax / (375 /*[ms]*/);
	alpha = 2 * accel;
}


enum State get_status(void)
{
	return current_status;
}

void force_status(enum State newStatus)
{
	current_status = newStatus;
}

void start_command() {
	keep_count = 1;
}

/*
	read UART while moving
	return - OK (continue command) or BREAK/ERROR (break current command)
*/
static char get_command(void)
{
	// should be called every 1ms, otherwise interval is not in [ms] unit
	static unsigned long time;
	if(send_status_interval > 0 && time != sys_time) {
		if(--send_status_counter <= 0) {
			send_status_counter = send_status_interval;
			send_status_and_position();
		}
		time = sys_time;
	}
	
	if(current_status == STATUS_STUCK) return ERROR;
	
	char command;
	if(UART_CheckRX()) // if any input in serial port
	{
		uint8_t packet_length;
		if(!try_read_packet((uint8_t*)&command, &packet_length)) return OK;

		switch(command)
		{           
			case 'P':
				send_status_and_position();
				break;
				
			case 'p':
				set_status_update_interval(get_word());
				break;

			case 'S':
				// stop and become idle
				stop();

				PWMinit();
				current_status = STATUS_IDLE;
				__delay_ms(10);

				return BREAK;

			case 's':
				// stop and turn off PWM
				stop();

				CloseMCPWM();
				current_status = STATUS_IDLE;
				__delay_ms(10);

				return BREAK;
				
			case 'c':
				set_control_flags(get_byte());
				break;
				
			case 'H':
				// turn off regulator (PWM can active, but regulator not setting PWM), must be turned on
				// explicitly with control_flags operator
				control_flags |= CONTROL_FLAG_NO_DISTANCE_REGULATOR | CONTROL_FLAG_NO_ROTATION_REGULATOR;
				break;

			case 'V':
				set_speed(get_byte());
				break;
			
			// break current command, status remains for 5ms
			case 'i':
				keep_count = 5;
				keep_speed = current_speed;
				keep_rotation = angular_speed;
				return BREAK;

			default:
				// received unknown packet, ignore
				return OK;

		}// end of switch(command)
	}

	return OK;
}


// ------------- STUCK CHECK -----------------
// TODO: remove (replaced with general stuck condition)
/*
static char check_stuck_condition(void)
{
	static int stuck=0;
	static unsigned long prev_sys_time=0;
	
	// every 10ms
	if((sys_time-prev_sys_time)>=10)
	{
		if((absl((long)positionL - prev_positionL) < (15+speedL*0.75) ) || (absl((long)positionR - prev_positionR) < (15+speedL*0.75) )) 
		{
			if(!stuck_off)
				stuck++;
				
			if(stuck == 5)
			{
				stuck=0;
				d_ref = L;
				t_ref = orientation;
				stop();
				current_status = STATUS_STUCK;
				__delay_ms(50);
				return STATUS_STUCK;
			} 
		}
		else
		{
			stuck = 0;
			prev_positionL = positionL;
			prev_positionR = positionR;
		}
		prev_sys_time=sys_time;
	}

	return OK;
}

static char check_stuck_condition_angle(void)
{
	static int stuck=0;
	static unsigned long prev_sys_time=0;
	
	// every 10ms
	if( sys_time - prev_sys_time >= 10 )
	{
		if( absl((long)orientation-prev_orientation) < 20+1.4*speedL)
		{
			if(!stuck_off)
				stuck++;
				
			if(stuck == 5)
			{
				stuck=0;
				d_ref = L;
				t_ref = orientation;
				stop();
				current_status = STATUS_STUCK;
				__delay_ms(50);
				return STATUS_STUCK;
			}
		}
		else
		{
			// not stuck
			stuck = 0;
			
			// save current position
			prev_orientation = orientation;
			prev_positionL = positionL;
			prev_positionR = positionR;
		}
		prev_sys_time=sys_time;
	}
	return OK;
}
*/
// ------------------------------------------------------------

// turn to point and move to it (Xd, Yd)
void turn_and_go(int Xd, int Yd, unsigned char end_speed, char direction)
{
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long L_dist, L0, L1, L2, L3;
	long D0, D1, D2;
	float v_vrh, v_end, v0;
	int length, angle;
	long long int Xdlong, Ydlong;
	
	Xdlong = MILIMETER_TO_DOUBLED_INC(Xd);
	Ydlong = MILIMETER_TO_DOUBLED_INC(Yd);

	d_ref=L;
	v0 = current_speed;
	direction = (direction >= 0 ? 1 : -1);

	// turn to end point, find angle to turn
	angle = atan2(Ydlong-Ylong, Xdlong-Xlong) * (180 / PI) - orientation * 360 / K1;
	
	if(direction < 0)
		angle += 180;
	
	angle = angle_range_fix(angle);

	if(turn(angle) == ERROR)
		return;

	length = get_distance_to(Xd, Yd);

	if((length < 100) && (vmax > K2/32)) {
		set_speed_accel(VMAX/3);
	}

	v_end = vmax * end_speed / 256;
	
	L_dist = MILIMETER_TO_INC(length);

	// calculate phase durations
	T1 = (vmax - current_speed) / accel;
	L0 = L;
	L1 = current_speed * T1 + accel * T1 * T1 / 2;

	T3 = (vmax - v_end) / accel;
	L3 = vmax * T3 - accel * T3 * T3 / 2;


	if( (L1 + L3) < L_dist)
	{
		// can reach vmax
		L2 = L_dist - L1 - L3;
		T2 = L2 / vmax;
	}
	else
	{
		// can't reach vmax
		T2 = 0;
		v_vrh = sqrt(accel * L_dist + (current_speed * current_speed + v_end * v_end) / 2);
		if( (v_vrh < current_speed) || (v_vrh < v_end) )
		{
			current_status = STATUS_ERROR;
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


	current_status = STATUS_MOVING;
	while(t < t3) {

		if(t == sys_time) continue;
		
		t = sys_time;
			
		if(get_command() == ERROR) {
			return;
		}
		
		/*
		// TODO: remove (replaced with general stuck condition)
		if( t > (t0+stuck_wait1) && t < (t3-stuck_wait2) )
		{
			if (check_stuck_condition() == STATUS_STUCK)
				return;
		}
		*/
		
		if(t <= t2) // refresh angle reference
		{
			if(direction > 0)
				t_ref = (atan2(Ydlong-Ylong, Xdlong-Xlong) / (2 * PI)) * K1;
			else
				t_ref = (atan2(Ylong-Ydlong, Xlong-Xdlong) / (2 * PI)) * K1;
		}

		if(t <= t1) // acceleration phase
		{
			D1 = D2 = d_ref = D0 + direction * (v0 * (t-t0) + accel * (t-t0)*(t-t0)/2);
		}
		else if(t <= t2) // constant speed phase
		{
			D2 = d_ref = D1 + direction * vmax * (t-t1);
		}
		else if(t <= t3) // decceleration phase
		{
			d_ref = D2 + direction * (vmax * (t-t2) - accel * (t-t2) * (t-t2) / 2);
		}
	}
	current_status = STATUS_IDLE;
}

// move robot forward in direction its facing
void forward(int length, unsigned char end_speed)
{
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long L_dist, L0, L1, L2, L3;
	long D0, D1, D2;
	float v_vrh, v_end, v0, v_ref;
	char sign;

	d_ref=L;
	v_ref=current_speed;
	v0 = v_ref;
	v_end = vmax * end_speed / 256;
	sign = (length >= 0) ? 1 : -1;
	L_dist = MILIMETER_TO_INC(length);

	T1 = (vmax - v_ref) / accel;
	L0 = L;
	L1 = v_ref * T1 + accel * T1 * (T1 / 2);

	T3 = (vmax - v_end) / accel;
	L3 = vmax * T3 - accel * T3 * (T3 / 2);

	if((L1 + L3) < (long)sign * L_dist)
	{
		// can reach
		L2 = sign * L_dist - L1 - L3;
		T2 = L2 / vmax;
	}
	else
	{
		// can't reach vmax
		T2 = 0;
		v_vrh = sqrt(accel * sign * L_dist + (v_ref * v_ref + v_end * v_end) / 2);
		if((v_vrh < v_ref) || (v_vrh < v_end))
		{
			current_status = STATUS_ERROR;
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

	current_status = STATUS_MOVING;
	while(t < t3)  {
		
		if(t == sys_time) continue;
		t = sys_time;
		
		if(get_command() == ERROR)
			return;

		/*
		// TODO: remove (replaced with general stuck condition)
		if( t > (t0+stuck_wait1) && t < (t3-stuck_wait2) )   
		{
			if (check_stuck_condition() == STATUS_STUCK)
				return;
		}
		*/
		
		if(t <= t1)
		{
			// v_ref = v0 + accel * (t-t0);
			D1 = D2 = d_ref = D0 + sign * (v0 * (t-t0) + accel * (t-t0)*(t-t0)/2);
		}
		else if(t <= t2)
		{
			// v_ref = vmax;
			D2 = d_ref = D1 + sign * vmax * (t-t1);
		}
		else if(t <= t3)
		{
			// v_ref = vmax - accel * (t-t2);
			d_ref = D2 + sign * (vmax * (t-t2) - accel * (t-t2) * (t-t2) / 2);
		}
	}
	
	current_status = STATUS_IDLE;
}

// funkcija za dovodjenje robota u zeljenu apsolutnu orientaciju
void rotate_absolute_angle(int ugao)
{
	int tmp = ugao - orientation * 360 / K1;
	tmp = angle_range_fix(tmp);
	turn(tmp);
}

char turn(int ugao)
{
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long Fi_total, Fi1;
	float ugao_ref, w_ref = 0;
	char sign;

	sign = (ugao >= 0 ? 1 : -1);

	Fi_total = (long)ugao * K1 / 360;

	T1 = T3 = omega / alpha;
	Fi1 = alpha * T1 * T1 / 2;
	if(Fi1 > (sign * Fi_total / 2))
	{
		// triangle profile (speeds up to some speed and then slows down to 0)
		Fi1 = sign  * Fi_total / 2;
		T1 = T3 = sqrt(2 * Fi1 / alpha);
		T2 = 0;
	}
	else
	{
		// trapezoid profile of speed graph (similar like triangle profile except there is period of constant speed in between)
		T2 = (sign * Fi_total - 2 * Fi1) / omega;
	}

	ugao_ref = t_ref;
	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;

	current_status = STATUS_ROTATING;
	while(t < t3) {
		if(t != sys_time) // every 1ms
		{
			t = sys_time;
			if(get_command() == ERROR)
				return ERROR;

			/*
			// TODO: remove (replaced with general stuck condition)
			if( t > (t0+stuck_wait1) && t < (t3-stuck_wait2) )
			{
				if (check_stuck_condition_angle() == STATUS_STUCK)
					return ERROR;
			}
			*/
			
			if(t <= t1)
			{
				w_ref += alpha;
				ugao_ref += sign * (w_ref - alpha / 2);
				t_ref = ugao_ref;
			}
			else if(t <= t2)
			{
				w_ref = omega;
				ugao_ref += sign * omega;
				t_ref = ugao_ref;
			}
			else if(t <= t3)
			{
				w_ref -= alpha;
				ugao_ref += sign * (w_ref + alpha / 2);
				t_ref = ugao_ref;
			}
		}
	}
	current_status = STATUS_IDLE;

	return OK;
}



void arc(long Xc, long Yc, int Fi, char direction_angle, char direction)
{
	float R, Fi_pocetno, delta, luk;
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long Fi_total, Fi1;
	float v_poc, dist_ref, ugao_ref, w_ref = 0, v_ref = 0;
	char sign;
	int angle;


	if (direction_angle) {
		sign = 1;
	} else {
		sign = -1;
	}
	
	R = get_distance_to(Xc, Yc);
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

	T1 = T3 = omega / alpha;
	Fi1 = alpha * T1 * T1 / 2;
	if(Fi1 > (sign * Fi_total / 2))
	{
		// triangle profile
		Fi1 = sign  * Fi_total / 2;
		T1 = T3 = sqrt(2 * Fi1 / alpha);
		T2 = 0;
	}
	else
	{
		// trapezoid profile
		T2 = (sign * Fi_total - 2 * Fi1) / omega;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	ugao_ref = t_ref;
	dist_ref = d_ref;

	current_status = STATUS_MOVING;
	while(t < t3) 
	{
		if(t == sys_time) continue;
		
		t = sys_time;
		if(get_command() == ERROR)
		{
			set_speed_accel(v_poc);
			return;
		}

		/*
		// TODO: remove (replaced with general stuck condition)
		if( t > (t0+stuck_wait1) && t < (t3-stuck_wait2) )
		{
			if(check_stuck_condition() == STATUS_STUCK)
			{
				set_speed_accel(v_poc);
				return;
			}
		}
		*/

		if(t <= t1)
		{
			w_ref += alpha;
			v_ref += accel;
			delta = sign * (w_ref - alpha / 2);
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
			w_ref -= alpha;
			v_ref -= accel;
			delta = sign * (w_ref + alpha / 2);
			luk = sign * delta * R / wheel_distance;
			ugao_ref += delta;
			dist_ref += direction * luk;
			t_ref = ugao_ref;
			d_ref = dist_ref;
		}
	}
	set_speed_accel(v_poc);
	current_status = STATUS_IDLE;
}

long inc_angle_diff(long a, long b) {
	long d = a - b;
	if(d > K1/2)
		d -= K1;
	else if(d < -K1/2)
		d += K1;
	return d;
}

int deg_angle_diff(int a, int b) {
	long d = a - b;
	if(d > 180)
		d -= 360;
	else if(d < -180)
		d += 360;
	return d;
}

void set_control_flags(uint8_t flags) {
	control_flags = flags;
}

/*
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
	
	float dist = get_distance_to(x,y);
	Ydlong = MILIMETER_TO_DOUBLED_INC(y);
	Xdlong = MILIMETER_TO_DOUBLED_INC(x);
	long goal_angle;
	long angle_diff;
	goal_angle = RAD_TO_INC_ANGLE(atan2(y-Y, x-X));
	angle_diff = inc_angle_diff(goal_angle, orientation);
	
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
	
	current_status = STATUS_MOVING;
	
	long D = d_ref;
	long R = t_ref;
	float min_speed = VMAX * 0x10 / 255;
	int e = 0;
	if(control_flags & CONTROL_FLAG_DEBUG) {
		start_packet('d');
			put_byte_word(0xff,0);
		end_packet();
	}
	
	start_command();
	while(1)
	{
		if(get_command() == ERROR)
			return;
			
		wait_for_regulator();
		
		goal_angle = RAD_TO_INC_ANGLE( atan2(y-Y, x-X) );
		long orient = orientation;
		
		if(direction == -1) {
			orient = inc_angle_range_fix( orient + DEG_TO_INC_ANGLE( 180 ) );
		}
		
		angle_diff = inc_angle_diff(goal_angle, orient);
		
		long abs_angle_diff = absl(angle_diff);
		
		if(abs_angle_diff > DEG_TO_INC_ANGLE(20)) {
			rotation_speed = minf( rotation_speed + alpha/2, omega/2 );
		} else if(abs_angle_diff > DEG_TO_INC_ANGLE(3)){
			rotation_speed = maxf( rotation_speed - alpha/2, min_speed );
		} else {
			R = orientation;
		}
		
		dist = get_distance_to(x,y);
		
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
		
		
		
		if( (control_flags & CONTROL_FLAG_DEBUG) && (++e > 25) ) {
			start_packet('d');
				put_byte_word(0, speed);
				put_byte_word(1, angle_diff);
				put_byte_word(2, rotation_speed);
				put_byte_word(3, (int)dist);
				put_byte_word(4, orientation);
				put_byte_word(5, goal_angle);
				put_byte_word(6, x);
				put_byte_word(7, y);
				put_byte_word(8, RAD_TO_DEG_ANGLE( atan2(y-Y, x-X) ));
				put_byte_word(25, X);
				put_byte_word(26, Y);
			end_packet();
			
			e = 0;
		}
		
		D += direction * speed;
		R += rotation_speed * signl(angle_diff);
		
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

	current_status = STATUS_IDLE;
}

void set_rotation_speed(unsigned char max_speed, unsigned char max_accel) {
	omega = VMAX * (unsigned char)max_speed / 256;
	if(omega < (VMAX * 161 / 256)) {
		// in 500 ms speeds up to vmax
		alpha = omega / (500 /*[ms]*/);
	} else {
		// in 375 ms speeds up to vmax
		alpha = omega / (375 /*[ms]*/);
	}
}

void set_speed(unsigned char tmp)
{
	// speedL = tmp;
	set_speed_accel(VMAX * (unsigned char)tmp / 256);
}

void set_status_update_interval(int miliseconds) {
	send_status_interval = miliseconds;
}


void set_stuck_on(void) {
	control_flags &= ~CONTROL_FLAG_NO_STUCK;
}

void set_stuck_off(void) {
	control_flags |= CONTROL_FLAG_NO_STUCK;
}
