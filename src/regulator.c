#include "regulator.h"
#include "uart.h"
#include "motor.h"
#include "encoder.h"
#include "config.h"
#include <stdint.h>
#include <p33FJ128MC802.h>
#include <libpic30.h>
#include <math.h>
#include "math.h"

long K1;
float K2;

float R_wheel;
float wheel_distance;
double wheel_correction_coeff = 1.0;

/*
	c_vmax = change of distance / ms
	c_accel = change of c_vmax / ms
*/
// static float c_vmax, c_accel;

/*
	c_omega = arc length / ms = wheel_distance * change of angle
	c_alpha = c_omega change / ms
*/
//static float c_omega, c_alpha;

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
static volatile long prev_rotation_error = 0;
static volatile long prev_distance_error = 0;
//

static volatile long keep_rotation = 0, keep_speed = 0, keep_count = 0;
// -------------------------------

static int16_t send_status_counter = 0;

static enum State current_status = STATUS_IDLE;
static enum State last_status = STATUS_IDLE;

// ---------------- HELPER FUNCTIONS ------------

long inc_angle_range_fix(long angle) {
	while(angle > K1/2)
		angle -= K1;
	while(angle < -K1/2)
		angle += K1;
	return angle;
}

long inc_angle_diff(long a, long b) {
	long d = a - b;
	if(d > K1/2)
		d -= K1;
	else if(d < -K1/2)
		d += K1;
	return d;
}

float get_distance_to(long x, long y) {
	return sqrt((x-X)*(x-X) + (y-Y)*(y-Y));
}

// ------------------------------------------


// --------------------------------------------------------------
volatile int check = 0;
// **********************************************************************
// ODOMETRY AND REGULATION (every 1ms)
// enters periodically every 1ms, and can last 30000 cpu cycles max (but recommended to last at most half of that, 15000 cycles)
// *********************************************************************
void __attribute__((interrupt(auto_psv))) _T1Interrupt(void)
{
	sys_time++;
	check++;
	IFS0bits.T1IF = 0;    // Clear Timer interrupt flag 

	static float vR;
	static float vL;
	
	static long sint, cost;
	
	static signed long regulator_distance, regulator_rotation;
	
	static long error;
	static volatile float x, y;
	static volatile float d;
	
	static volatile long theta = 0;

	// ODOMETRY, reading encoders and processing them
	//************************************************************************
	
	// read left encoder
	vL = encoder_left_get_count();
	positionL += vL;

	// read right encoder
	vR = encoder_right_get_count();
	positionR += vR;

	/* FIXME: moving too much can cause overflow when passed distance is too big
	 		  can be fixed by resetting values when crosses some value, but maintain references
			  and orientation
	*/
	L = (positionR*wheel_correction_coeff + positionL) / 2;

	// FIXME: rotating too much can cause overflow
	orientation = (positionR*wheel_correction_coeff - positionL);
	
	if (orientation > 0)
	{
		while(orientation > K1)
		{
			orientation -= K1;
		}
	}
	else
	{
		while( orientation < -K1 )
		{
			orientation = orientation + K1;
		}

	}
	
	if(orientation > K1/2) {
		orientation -= K1;
	}
	if(orientation < -K1/2) {
		orientation += K1;
	}

	theta = (orientation * 2*SINUS_MAX) / (K1 / 2);
	
	if(theta < 0)
		theta += 4*SINUS_MAX;

	d = vR + vL;
	
	sin_cos(theta, &sint, &cost);
	
	x = d * cost;
	y = d * sint;

	Xlong += (x/SINUS_AMPLITUDE);
	Ylong += (y/SINUS_AMPLITUDE);

	X = DOUBLED_INC_TO_MILIMETER(Xlong);
	Y = DOUBLED_INC_TO_MILIMETER(Ylong);
	
	
	// REGULATOR
	//*************************************************************************
	
	current_speed = (vL + vR) / 2; // v = s/t, current_speed [inc/ms]
	
	// keep speed for some time if interrupted
	if(keep_count > 0) {
		t_ref += keep_rotation;
		d_ref += keep_speed;
		keep_count--;
		if(keep_count == 0) {
			current_status = STATUS_IDLE;
		}
	}
	
	error = d_ref - L;
	// regulator_distance = error * Gp_D - Gd_D * current_speed; // PD (proportional, differential) regulator
	regulator_distance = error * c_pid_d_p - c_pid_d_d * current_speed; // PD (proportional, differential) regulator

	// ---------[ Distance Stuck Detection ]----------
	
	if( c_enable_stuck == 1 ) {
		
		if(absl(error-prev_distance_error) > MILIMETER_TO_INC(c_stuck_distance_jump)) {
			current_status = STATUS_STUCK;
		}
		
		if((signl(error) != signl(current_speed) && absl(current_speed) > 6) || (absl(regulator_distance) > MOTOR_MAX_SPEED/4 && absl(current_speed) < 3)) {
			if(++d_ref_fail_count > c_stuck_distance_max_fail_count) {
				current_status = STATUS_STUCK;
				d_ref_fail_count = 0;
			}
		} else {
			d_ref_fail_count = 0;
		}
		
	}
	
	prev_distance_error = error;
	// -------------------------------------------

	// ------------[ rotation regulator ]-------------
	angular_speed = vR - vL; // angular speed [inc/ms]

	error = ((long)orientation - t_ref) % K1;

	if(error > K1/2) {
		error -= K1;
	} else if(error < -K1/2) {
		error += K1;
	}

	// regulator_rotation = error * Gp_T - (-angular_speed * Gd_T); // PD (proportional, differential) regulator
	regulator_rotation = error * c_pid_r_p - (-angular_speed * c_pid_r_d); // PD (proportional, differential) regulator
	
	// ------------[ Rotation Stuck Detect ]---------------
	
	if( c_enable_stuck == 1 ) {
		
		if(absl(error-prev_rotation_error) > DEG_TO_INC_ANGLE(c_stuck_rotation_jump)) {
			current_status = STATUS_STUCK;
		}
		
		if(absl(error) > DEG_TO_INC_ANGLE(10) && signl(error) != signl(angular_speed) && absl(angular_speed) < DEG_TO_INC_ANGLE(1)) {
			if(++t_ref_fail_count > c_stuck_rotation_max_fail_count) {
				current_status = STATUS_STUCK;
				t_ref_fail_count = 0;
			}
		} else {
			t_ref_fail_count = 0;
		}
		
		// if robot stuck, then shut down engines
		
		if(current_status == STATUS_STUCK) {
			c_stuck = 1;
			c_distance_regulator = 0;
			c_rotation_regulator = 0;
			motor_turn_off();
		}
		
	}
	
	prev_rotation_error = error;
	// -------------------------------------------
	
	if(c_distance_regulator == 0) {
		regulator_distance = 0;
		d_ref = L;
	}
	if(c_rotation_regulator == 0) {
		regulator_rotation = 0;
		t_ref = orientation;
	}
	
	// ------ final PWM is result of superposition of 2 regulators --------
	if(check > 1) {
		start_packet('O');
			put_word(check);
		end_packet();
	}
	motor_left_set_power(regulator_distance - regulator_rotation);
	motor_right_set_power(regulator_distance + regulator_rotation);
	
	// send status periodically if requested
	if(c_send_status_interval > 0 && ++send_status_counter > c_send_status_interval) {
		send_status_counter = 0;
		send_status_and_position();
	}


	check--;
	IFS0bits.T1IF = 0;    // Clear Timer interrupt flag 
}

void reset_driver(void)
{
	positionR = positionL = 0;
	L = orientation = 0;
	
	motor_init();
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
	reset_stuck();
	wait_for_regulator();
}

static void setY(int tmp)
{
	Ylong = (long long)tmp * 2 * K2;
	d_ref = L;
	t_ref = orientation;
	reset_stuck();
	wait_for_regulator();
}


static void setO(int tmp)
{
	positionL = -DEG_TO_INC_ANGLE(tmp) / 2;
	positionR =  DEG_TO_INC_ANGLE(tmp) / 2;

	L = 0;
	orientation = (long int)(positionR - positionL) % K1;

	d_ref = L;
	t_ref = orientation;
	reset_stuck();

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
	if(!can_send_packet()) return;
	
	start_packet(CMD_SEND_STATUS);
		put_byte(current_status);
		put_word(X);
		put_word(Y);
		put_word(INC_TO_DEG_ANGLE(orientation));
		put_word(current_speed);
	end_packet();
}

void set_speed_accel(float v)
{
	c_vmax = v;
	c_omega = 2 * c_vmax;
	c_accel = c_vmax / ((350-500) * (v / VMAX) + 500);
	
	/*
	if(c_vmax < (VMAX * 161 / 256))
		// in 500 ms speeds up to c_vmax
		c_accel = c_vmax / (500 );
	else
		// in 375 ms speeds up to c_vmax
		c_accel = c_vmax / (375);
	*/
	c_alpha = 2 * c_accel;
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
	keep_count = 0;
}

void report_status() {
	if( c_status_change_report && current_status != last_status ) {
		send_status_counter = 0;
		send_status_and_position();
		last_status = current_status;
	}
}

struct MoveCmdParams {
	char active;
	int x;
	int y;
	char direction;
	int radius;
};

struct MoveCmdParams move_cmd_next;

/*
	read UART while moving
	return - OK (continue command) or BREAK/ERROR (break current command)
*/
static char get_command(void)
{	
	report_status();
	
	if(current_status == STATUS_STUCK) return ERROR;
	Packet* pkt;
	
	if((pkt = try_read_packet()) != 0)
	{
		switch(pkt->type)
		{           
			case CMD_SEND_STATUS:
				send_status_and_position();
				break;

			case CMD_HARD_STOP:
				// stop and become idle
				stop();

				// motor_init();
				current_status = STATUS_IDLE;
				__delay_ms(10);

				return BREAK;

			case CMD_SOFT_STOP:
				// stop and turn off PWM
				motor_turn_off();
				current_status = STATUS_IDLE;
				__delay_ms(10);

				return BREAK;
			
			case CMD_SMOOTH_STOP:
				smooth_stop();
				return BREAK;
				
			case CMD_SET_CONFIG:
				config_load(pkt->size, pkt->data);
				break;
				
			case CMD_GET_CONFIG: {
				int key = get_byte();
				uint32_t val = config_get_as_uint32(key);
				start_packet(CMD_GET_CONFIG);
					put_word(val >> 16);
					put_word(val);
				end_packet();
				break;
			}
				
			case CMD_KILL_REGULATOR:
				// turn off regulator (PWM can active, but regulator not setting PWM), must be turned on
				// explicitly with control_flags operator
				c_distance_regulator = 0;
				c_rotation_regulator = 0;
				break;
				
			case CMD_MOVE_TO: {
				int tmpX = get_word();
				int tmpY = get_word();
				char direction = get_byte();
				int radius = get_word();
				
				move_cmd_next.active = 1;
				move_cmd_next.x = tmpX;
				move_cmd_next.y = tmpY;
				move_cmd_next.direction = direction;
				move_cmd_next.radius = radius;
				
				break;
			}
			
			case CMD_SET_SPEED:
				set_speed(get_byte());
				break;
			
			// break current command, status remains for 5ms
			case CMD_BREAK:
				keep_count = c_keep_count;
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

// turn to point and move to it (Xd, Yd)
void turn_and_go(int Xd, int Yd, unsigned char end_speed, char direction)
{
	start_command();
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long L_dist, L0, L1, L2, L3;
	long D0, D1, D2;
	float v_vrh, v_end, v_ref, v0;
	int length, angle;
	long long int Xdlong, Ydlong;
	
	Xdlong = MILIMETER_TO_2INC(Xd);
	Ydlong = MILIMETER_TO_2INC(Yd);
	d_ref=L;
	v0 = 0;
	direction = (direction >= 0 ? 1 : -1);

	// turn to end point, find angle to turn
	angle = atan2(Ydlong-Ylong, Xdlong-Xlong) * (180 / PI) - orientation * 360 / K1;
	
	if(direction < 0)
		angle += 180;
	
	angle = deg_angle_range_fix(angle);

	if(turn(angle) == ERROR)
		return;

	length = get_distance_to(Xd, Yd);

	if((length < 100) && (c_vmax > K2/32)) {
		set_speed_accel(VMAX/3);
	}

	v_end = c_vmax * end_speed / 256;
	
	L_dist = MILIMETER_TO_INC(length);

	// calculate phase durations
	T1 = (c_vmax - v0) / c_accel;
	L0 = L;
	L1 = current_speed * T1 + c_accel * T1 * T1 / 2;

	T3 = (c_vmax - v_end) / c_accel;
	L3 = c_vmax * T3 - c_accel * T3 * T3 / 2;


	if( (L1 + L3) < L_dist)
	{
		// can reach c_vmax
		L2 = L_dist - L1 - L3;
		T2 = L2 / c_vmax;
	}
	else
	{
		// can't reach c_vmax
		T2 = 0;
		v_vrh = sqrt(c_accel * L_dist + (current_speed * current_speed + v_end * v_end) / 2);
		if( (v_vrh < current_speed) || (v_vrh < v_end) )
		{
			current_status = STATUS_ERROR;
			return; //mission impossible
		}

		T1 = (v_vrh - current_speed) / c_accel;
		T3 = (v_vrh - v_end) / c_accel;
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
		
		if(t <= t2) // refresh angle reference
		{
			if(direction > 0) {
				t_ref = RAD_TO_INC_ANGLE(atan2(Ydlong-Ylong, Xdlong-Xlong));
			} else {
				t_ref = RAD_TO_INC_ANGLE(atan2(Ylong-Ydlong, Xlong-Xdlong));
			}
		}

		if(t <= t1) // acceleration phase
		{
			v_ref = v0 + c_accel * (t-t0);
			D1 = D2 = d_ref = D0 + direction * (v0 * (t-t0) + c_accel * (t-t0)*(t-t0)/2);
		}
		else if(t <= t2) // constant speed phase
		{
			v_ref = c_vmax;
			D2 = d_ref = D1 + direction * c_vmax * (t-t1);
		}
		else if(t <= t3) // decceleration phase
		{
			d_ref = D2 + direction * (v_ref * (t-t2) - c_accel * (t-t2) * (t-t2) / 2);
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
	v_end = c_vmax * end_speed / 256;
	sign = (length >= 0) ? 1 : -1;
	L_dist = MILIMETER_TO_INC(length);

	T1 = (c_vmax - v_ref) / c_accel;
	L0 = L;
	L1 = v_ref * T1 + c_accel * T1 * (T1 / 2);

	T3 = (c_vmax - v_end) / c_accel;
	L3 = c_vmax * T3 - c_accel * T3 * (T3 / 2);

	if((L1 + L3) < (long)sign * L_dist)
	{
		// can reach
		L2 = sign * L_dist - L1 - L3;
		T2 = L2 / c_vmax;
	}
	else
	{
		// can't reach c_vmax
		T2 = 0;
		v_vrh = sqrt(c_accel * sign * L_dist + (v_ref * v_ref + v_end * v_end) / 2);
		if((v_vrh < v_ref) || (v_vrh < v_end))
		{
			current_status = STATUS_ERROR;
			return; //mission impossible
		}

		T1 = (v_vrh - v_ref) / c_accel;
		T3 = (v_vrh - v_end) / c_accel;
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
		
		if(t <= t1)
		{
			v_ref = v0 + c_accel * (t-t0);
			D1 = D2 = d_ref = D0 + sign * (v0 * (t-t0) + c_accel * (t-t0)*(t-t0)/2);
		}
		else if(t <= t2)
		{
			v_ref = c_vmax;
			D2 = d_ref = D1 + sign * c_vmax * (t-t1);
		}
		else if(t <= t3)
		{
			// v_ref = c_vmax - c_accel * (t-t2);
			d_ref = D2 + sign * (v_ref * (t-t2) - c_accel * (t-t2) * (t-t2) / 2);
		}
	}
	
	current_status = STATUS_IDLE;
}

void rotate_absolute_angle(int angle)
{
	int tmp = angle - INC_TO_DEG_ANGLE(orientation);
	tmp = deg_angle_range_fix(tmp);
	turn(tmp);
}

char turn(int angle)
{
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long Fi_total, Fi1;
	float angle_ref, w_ref = 0;
	char sign;

	sign = (angle >= 0 ? 1 : -1);

	// Fi_total = (long)angle * K1 / 360;
	Fi_total = DEG_TO_INC_ANGLE(angle);

	T1 = T3 = c_omega / c_alpha;
	Fi1 = c_alpha * T1 * T1 / 2;
	if(Fi1 > (sign * Fi_total / 2))
	{
		// triangle profile (speeds up to some speed and then slows down to 0)
		Fi1 = sign  * Fi_total / 2;
		T1 = T3 = sqrt(2 * Fi1 / c_alpha);
		T2 = 0;
	}
	else
	{
		// trapezoid profile of speed graph (similar like triangle profile except there is period of constant speed in between)
		T2 = (sign * Fi_total - 2 * Fi1) / c_omega;
	}

	angle_ref = t_ref;
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

			
			if(t <= t1)
			{
				w_ref += c_alpha;
				angle_ref += sign * (w_ref - c_alpha / 2);
				t_ref = angle_ref;
			}
			else if(t <= t2)
			{
				w_ref = c_omega;
				angle_ref += sign * c_omega;
				t_ref = angle_ref;
			}
			else if(t <= t3)
			{
				w_ref -= c_alpha;
				angle_ref += sign * (w_ref + c_alpha / 2);
				t_ref = angle_ref;
			}
		}
	}
	current_status = STATUS_IDLE;

	return OK;
}



void arc(long Xc, long Yc, int Fi, char direction_angle, char direction)
{
	float R, Fi_start, delta, arc_value;
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long Fi_total, Fi1;
	float v0, dist_ref, angle_ref, w_ref = 0, v_ref = 0;
	char sign;
	int angle;


	if (direction_angle) {
		sign = 1;
	} else {
		sign = -1;
	}
	
	R = get_distance_to(Xc, Yc);
	Fi_start = atan2(((int)Y-(int)Yc), ((int)X-(int)Xc));
	// angle = Fi_start * 180 / PI;
	angle = RAD_TO_DEG_ANGLE(Fi_start);
	direction = (direction >= 0 ? 1 : -1);

	angle = (angle + direction * sign * 90) % 360;
	angle = deg_angle_range_fix(angle);

	// angle -= orientation * 360 / K1;
	angle -= INC_TO_DEG_ANGLE(orientation);
	angle %= 360;

	angle = deg_angle_range_fix(angle);
	
	if(turn(angle))
		return;

	v0 = c_vmax;

	// Fi_total = (long)Fi * K1 / 360;
	Fi_total = DEG_TO_INC_ANGLE(Fi);

	T1 = T3 = c_omega / c_alpha;
	Fi1 = c_alpha * T1 * T1 / 2;
	if(Fi1 > (sign * Fi_total / 2))
	{
		// triangle profile
		Fi1 = sign  * Fi_total / 2;
		T1 = T3 = sqrt(2 * Fi1 / c_alpha);
		T2 = 0;
	}
	else
	{
		// trapezoid profile
		T2 = (sign * Fi_total - 2 * Fi1) / c_omega;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	angle_ref = t_ref;
	dist_ref = d_ref;

	current_status = STATUS_MOVING;
	while(t < t3) 
	{
		if(t == sys_time) continue;
		
		t = sys_time;
		if(get_command() == ERROR)
		{
			set_speed_accel(v0);
			return;
		}


		if(t <= t1)
		{
			w_ref += c_alpha;
			v_ref += c_accel;
			delta = sign * (w_ref - c_alpha / 2);
			arc_value = sign * delta * R / wheel_distance;
			angle_ref += delta;
			dist_ref += direction * arc_value;
			t_ref = angle_ref;
			d_ref = dist_ref;
		}
		else if(t <= t2)
		{
			w_ref = c_omega;
			v_ref = c_vmax;
			delta = sign * c_omega;
			arc_value = sign * delta * R / wheel_distance;
			angle_ref += delta;
			dist_ref += direction * arc_value;
			t_ref = angle_ref;
			d_ref = dist_ref;
		}
		else if(t <= t3)
		{
			w_ref -= c_alpha;
			v_ref -= c_accel;
			delta = sign * (w_ref + c_alpha / 2);
			arc_value = sign * delta * R / wheel_distance;
			angle_ref += delta;
			dist_ref += direction * arc_value;
			t_ref = angle_ref;
			d_ref = dist_ref;
		}
	}
	set_speed_accel(v0);
	current_status = STATUS_IDLE;
}

int pcnt = 0;
/*
	direction:
		0 - pick smallest rotation
		1 - forward
	   -1 - backward
*/
void move_to(long x, long y, char direction, int radius) {
	float speed = current_speed;
	float rotation_speed = angular_speed; /* [inc/ms] */
	
	long long int Xdlong, Ydlong;
	
	float dist = get_distance_to(x,y);
	
	float v_div_w = (float)MILIMETER_TO_INC( minf(dist, radius)/2.0f ) / (float)RAD_TO_INC_ANGLE(1);
	float w_div_v = 1.0f/v_div_w;
	
	float w;
	float v;
	
	Ydlong = MILIMETER_TO_2INC(y);
	Xdlong = MILIMETER_TO_2INC(x);
	long goal_angle;
	long angle_diff;
	goal_angle = RAD_TO_INC_ANGLE(atan2(y-Y, x-X));
	angle_diff = inc_angle_diff(goal_angle, orientation);
	
	if(direction > 0) direction = 1;
	else if(direction < 0) direction = -1;
	
	if(direction == 0) {
		// determine direction automatically
		if(absl(angle_diff) < K1/4) {
			direction = 1;
		} else {
			direction = -1;
		}
	}
	
	current_status = STATUS_MOVING;
	
	long D = d_ref;
	long R = t_ref;
	
	float t;
	
	v = c_vmax;
	w = w_div_v * v;
	if(w > c_omega) {
		w = c_omega;
		v = v_div_w * w;
		if(v > c_vmax) {
			current_status = STATUS_ERROR;
			return;
		}
	}
	
	
	long dt;
	
	int slowdown_phase = 0;
	int maxspeed_phase = 0;

	
	float accel;
	float alpha;
	start_command();
	long ls = sys_time;
	int lt = sys_time;
	while(1)
	{
		if(get_command() == ERROR) {
			return;
		}
		
		dt = sys_time - lt;
		
		accel = c_accel * dt;
		alpha = c_alpha * dt;
				
		goal_angle = RAD_TO_INC_ANGLE( atan2(y-Y, x-X) );
		long orient = orientation;
		
		if(direction == -1) {
			orient = inc_angle_range_fix( orient + DEG_TO_INC_ANGLE( 180 ) );
		}
		
		angle_diff = inc_angle_diff(goal_angle, orient);
		long abs_angle_diff = absl(angle_diff);
		
		dist = get_distance_to(x,y);
		
		char ss = signf(speed);
		char sr = signf(rotation_speed);
		float abs_speed = absf(speed);
		float abs_rotation_speed = absf(rotation_speed);
		
		
		// speed
		if(ss != direction) {
			start_packet('F');
				put_byte(5);
				put_word(pcnt++);
			end_packet();

			speed -= dval(ss, c_accel);
			if(speed == 0.0f) {
				speed += dval(direction, 0.01f);
			}
		} else {
			t = abs_speed/c_accel * c_slowdown;
			if(slowdown_phase == 1 || abs_speed*t/2.0f > MILIMETER_TO_INC(dist)) {
				slowdown_phase = 1;
				// slow down
				if(sys_time - ls > c_tmr) {
					ls = sys_time;
					start_packet('F');
						put_byte(0);
						put_word(pcnt++);
					end_packet();
				}
				speed = dval(ss, maxf(0.0f, abs_speed-accel ));
				if(speed == 0 || dist < 2.0f) {					
					d_ref = D;
					break;
				}
			} else if(abs_angle_diff < DEG_TO_INC_ANGLE(c_angle_speedup)) {
				maxspeed_phase = 1;
				// 0 -> vmax, 25 -> v
				// vmax-v / 25 + v
				float virt_max = c_vmax; //((v - c_vmax) * (float)abs_angle_diff / (float)DEG_TO_INC_ANGLE(c_angle_speedup)) + c_vmax;
				if(sys_time - ls > c_tmr ) {
					ls = sys_time;
					start_packet('F');
						put_byte(1);
						put_word(pcnt++);
						put_byte(speed);
					end_packet();
				}
				if(abs_speed < virt_max) {
					speed = dval(ss, minf(abs_speed+accel, virt_max));
				} else {
					speed = dval(ss, maxf(abs_speed-accel, virt_max));
				}
			} else {
				if(sys_time - ls > c_tmr) {
					ls = sys_time;
					start_packet('F');
						put_byte(2);
						put_word(pcnt++);
					end_packet();
				}
				if(abs_speed < v) {
					speed = dval(ss, minf(abs_speed+accel, v));
				} else if(abs_speed > v) {
					speed = dval(ss, maxf(v, abs_speed-accel));
				}
				
			}
		}
		
		// rotation
		if((char)signl(angle_diff) != sr) {
			rotation_speed -= dval(sr, alpha);
			if(rotation_speed == 0.0f) {
				rotation_speed += dval(signl(angle_diff), 0.01f);
			}
		} else {
			if(abs_angle_diff > DEG_TO_INC_ANGLE(1)) {
				t = abs_rotation_speed/c_alpha;
				if(abs_rotation_speed*t/2.0f > abs_angle_diff) {
					rotation_speed = dval(sr, maxf( abs_rotation_speed - alpha, 0 ));
				} else {
					rotation_speed = dval(sr, minf( abs_rotation_speed + alpha, w ));
				}
			} else {
				R = orientation + rotation_speed * dt;
				rotation_speed = 0;
			}
		}
		
		dt = sys_time - lt;
		lt = sys_time;

		D += speed * dt;
		R += rotation_speed * dt;
		
		d_ref = D;
		t_ref = R;
		
		wait_for_regulator();
	}
	current_status = STATUS_IDLE;
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

void smooth_stop(void) {
	float speed = current_speed;
	float ang_speed = angular_speed;
	while(speed != 0.0f || ang_speed != 0.0f) {
		if(absf(speed) > c_accel) {
			speed -= signf(speed) * c_accel;
		} else {
			speed = 0.0f;
		}
		
		if(absf(ang_speed) > c_alpha) {
			ang_speed -= dfval(ang_speed, c_alpha);
		} else {
			ang_speed = 0.0f;
		}
		d_ref += speed;
		t_ref += ang_speed;
		
		wait_for_regulator();
	}
	current_status = STATUS_IDLE;
}

void set_rotation_speed(unsigned char max_speed, unsigned char max_accel) {
	c_omega = VMAX * (unsigned char)max_speed / 256;
	
	if(c_omega < (VMAX * 161 / 256)) {
		// in 500 ms speeds up to c_vmax
		c_alpha = c_omega / (500 /*[ms]*/);
	} else {
		// in 375 ms speeds up to c_vmax
		c_alpha = c_omega / (375 /*[ms]*/);
	}
}

void set_speed(unsigned char tmp)
{
	set_speed_accel(VMAX * (unsigned char)tmp / 256);
}

void on_vmax_change() {
	c_vmax = VMAX * (unsigned char)c_vmax / 256;
}

void on_omega_change() {
	c_omega = VMAX * (unsigned char)c_omega / 256;
}

void on_alpha_change() {
	c_alpha = c_omega / (c_alpha /*[ms]*/);
}

void on_accel_change() {
	c_accel = c_vmax / (c_accel /*[ms]*/);
}

void reset_stuck() {
	prev_orientation = orientation;
	prev_L = L;
	t_ref_fail_count = 0;
	d_ref_fail_count = 0;
	prev_rotation_error = 0;
	prev_distance_error = 0;
	if(c_stuck) {
		c_distance_regulator = 1;
		c_rotation_regulator = 1;
		c_stuck = 0;
	}
}


void regulator_init(void) {
	move_cmd_next.active = 0;
	config_on_change(CONF_WHEEL_R1, calculate_K);
	config_on_change(CONF_WHEEL_R2, calculate_K);
	config_on_change(CONF_WHEEL_DISTANCE, calculate_K);
	config_on_change(CONF_OMEGA, on_omega_change);
	config_on_change(CONF_VMAX, on_vmax_change);
	config_on_change(CONF_ACCEL, on_accel_change);
	config_on_change(CONF_ALPHA, on_alpha_change);
}

void calculate_K() {
	wheel_distance = c_wheel_distance;
	R_wheel = c_wheel_r1;
	wheel_correction_coeff = (c_wheel_r2 / c_wheel_r1);
	K1 = (long)(4.0*2048.0 * wheel_distance / (R_wheel / 2.0));
	K2 = (float)(4.0*2048.0 / (R_wheel * PI));
}

