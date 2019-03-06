#include "config.h"
#include "hw/motor.h"
#include "hw/encoder.h"

#include "packet.h"
#include "regulator.h"
#include "math.h"

// #undef dbg
// #define dbg(...)

long K1;
float K2;

float R_wheel, wheel_distance;
double wheel_correction_coeff = 1.0;
static int setpoint1_active = 0, setpoint2_active = 0;

/*
	c_vmax = max speed / ms
	g_accel = speed_change / ms
*/
static float g_accel = 0, g_alpha = 0;

/*
	c_omega = arc_length / ms = wheel_distance * change_of_angle
	g_alpha = rot_speed_change / ms
*/
// changes in interrupt, so here are all volatiles
static volatile ldouble positionL=0, positionR=0, Xlong = 0, Ylong = 0;
static volatile long X = 0, Y = 0;
// static volatile long current_speed=0, angular_speed=0;
static volatile float current_speed=0, angular_speed=0;
static volatile unsigned long sys_time = 0;

// ----- variables for controlling robot -----
static volatile float orientation = 0;
static volatile ldouble L = 0; // total distance
static volatile long t_ref = 0, d_ref = 0;
static volatile long prev_rotation_error = 0, prev_distance_error = 0;
static volatile long keep_rotation = 0, keep_speed = 0, keep_count = 0;
// -------------------------------

// stuck detection in regulator
static long prev_orientation = 0;
static long prev_L = 0;
static int t_ref_fail_count = 0;
static int d_ref_fail_count = 0;

// static int16_t send_status_counter = 0;

static enum State current_status = STATUS_IDLE;
static enum State last_status = STATUS_IDLE;
// ---------------- HELPER FUNCTIONS ------------

long inc_angle_diff(long a, long b) {
	return angle_range_normalize_long(a - b, K1);
}

float get_distance_to(long x, long y) {
	return sqrt((x-X)*(x-X) + (y-Y)*(y-Y));
}

// ------------------------------------------


// --------------------------------------------------------------
// volatile int check = 0;
// **********************************************************************
// ODOMETRY AND REGULATION (every 1ms)
// enters periodically every 1ms, and can last 30000 cpu cycles max (but recommended to last at most half of that, 15000 cycles)
// *********************************************************************

// void INTERRUPT _INT1Interrupt(void) {
	/*
	#ifndef SIM
	IFS1bits.INT1IF = 0;
	// RUN_EACH_NTH_CYCLES(uint16_t, 200, {
		
		start_packet('X');
		end_packet();
	// })
	#endif
	*/
// }

// void INTERRUPT _INT2Interrupt(void) {
	// start_packet('x');
	// end_packet();
	/*
	#ifndef SIM
	IFS1bits.INT2IF = 0;
	// RUN_EACH_NTH_CYCLES(uint16_t, 200, {
		
		start_packet('x');
		end_packet();
	// })
	#endif
	*/
// }

struct regulator_t {
	int last_dist;
	int osc_count;
	int speed_error_accum;
};

#ifdef SIM
#include <time.h>
struct timespec diff(struct timespec start, struct timespec end);
struct timespec past;
int avg=0;
int count = 0;
int count_n = 1000;

int avg2=1000000;
int vals[10]={1000000,1000000,1000000,1000000,1000000,1000000,1000000,1000000,1000000,1000000};
int a_ptr=0;
#endif

static long speed_mode_error_accum1 = 0;
static long speed_mode_error_accum2 = 0;

void regulator_interrupt(void) {
	
	static float vL,vR;
	static long sint, cost, theta = 0;
	static signed long regulator_distance, regulator_rotation;
	static long error_distance, error_angular;
	static float x, y;
	
	sys_time++;
	
	
#ifdef SIM
	struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    struct timespec tmp=diff(past,now);
    
    avg += tmp.tv_nsec;
    if(++count >= count_n) {
		int passed = avg/count;
		// printf("time passed: %d\n", passed);
		
		int cur = avg2 * 10 - vals[a_ptr];
		vals[a_ptr++] = passed;
		a_ptr = a_ptr % 10;
		avg2 = (cur + passed) / 10;
		count = 0;
		avg=0;
	}
	
    past=now;
#endif

	// keep speed for some time if interrupted
	if(keep_count > 0) {
		t_ref += keep_rotation;
		d_ref += keep_speed;
		if(--keep_count == 0) {
			current_status = STATUS_IDLE;
		}
	}

	// ODOMETRY, reading encoders and processing them
	//************************************************************************
	
	// read left encoder
	vL = encoder_odometry_left_get_velocity();
	positionL += vL * wheel_correction_coeff;

	// read right encoder
	vR = encoder_odometry_right_get_velocity();
	positionR += vR;
	
	// speed
	current_speed = (vL + vR) / 2; // v = s/t, current_speed [inc/ms]
	angular_speed = vR - vL; // angular speed [inc/ms]

	// position
	L = (positionL + positionR) / 2;
	orientation = angle_range_normalize_float(positionR - positionL, K1);
	// printf("orientation: %f speed: %ld\n", orientation, angular_speed);
	
	// convert range from [-K1/2, K1/2] to [-2*SINUS_MAX, 2*SINUS_MAX]
	theta = orientation * 2*SINUS_MAX / (K1 / 2);
	sin_cos(theta, &sint, &cost);
	
	x = current_speed * cost;
	y = current_speed * sint;

	// update increment position
	Xlong += (x/SINUS_AMPLITUDE);
	Ylong += (y/SINUS_AMPLITUDE);
	
	// translate increment position to millimeters
	X = INC_TO_MM(Xlong);
	Y = INC_TO_MM(Ylong);
	
	// error
	error_distance = d_ref - L;
	error_angular = angle_range_normalize_long(orientation - t_ref, K1);

	// ---------[ Stuck Detection ]----------
	
	if( c_enable_stuck == 1 ) {
		// distance stuck
		if(absl(error_distance-prev_distance_error) > MM_TO_INC(c_stuck_distance_jump)) {
			current_status = STATUS_STUCK;
		}
		
		if((signl(error_distance) != signl(current_speed) && absl(current_speed) > 6) || 
			(absl(regulator_distance) > MOTOR_MAX_SPEED/4 && absl(current_speed) < 3)) {
			if(++d_ref_fail_count > c_stuck_distance_max_fail_count) {
				current_status = STATUS_STUCK;
				d_ref_fail_count = 0;
			}
		} else {
			d_ref_fail_count = 0;
		}
		
		// angular stuck
		if(absl(error_angular-prev_rotation_error) > DEG_TO_INC_ANGLE(c_stuck_rotation_jump)) {
			current_status = STATUS_STUCK;
		}
		
		if(absl(error_angular) > DEG_TO_INC_ANGLE(1) && (signl(error_angular) != signl(-angular_speed) || absl(angular_speed) < DEG_TO_INC_ANGLE(0.1))) {
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
	prev_distance_error = error_distance;
	prev_rotation_error = error_angular;
	
	// -------------------------------------------
	
	// REGULATOR
	//*************************************************************************
	
	// TEST
	// c_regulator_mode = 5;
	// motor_left_set_power(encoder_odometry_left_get_count());
	// motor_right_set_power(encoder_odometry_right_get_count());
	//
	
	switch(c_regulator_mode) {
		
		case REGULATOR_POSITION: {
		
			int speed_error = error_distance - (-current_speed);
			static float speed_accum_d = 0;
			speed_accum_d += speed_error * c_accum_speed;
			speed_accum_d = clip2(c_accum_clip, speed_accum_d);
		
			// PID 					proportional 						integral 						differential
			regulator_distance = (error_distance * c_pid_d_p)  /*+  (speed_accum_d * c_pid_d_i)*/  -  (c_pid_d_d * current_speed);
			// dbg(printf("error_dist: %ld d_ref: %ld L: %lf K1: %ld K2: %f vL: %f vR: %f\n", speed_error, d_ref, L, K1, K2, vL, vR));
			
			int rot_speed_error = error_angular - (-current_speed);
			static float speed_accum_r = 0;
			speed_accum_r += rot_speed_error * c_accum_speed;
			speed_accum_r = clip2(c_accum_clip, speed_accum_r);
	
			// PID 					proportional 					integral 						differential
			regulator_rotation = (error_angular * c_pid_r_p)  /*+  (speed_accum_r * c_pid_r_i)*/ - 	(-angular_speed * c_pid_r_d);
			// dbg(printf("rot_error: %ld, reg_rot: %ld t_ref: %ld orientation: %lf\n", rot_speed_error, regulator_rotation, t_ref, orientation));
	
			if(c_distance_regulator == 0) {
				regulator_distance = 0;
				d_ref = L;
			}
			if(c_rotation_regulator == 0) {
				regulator_rotation = 0;
				t_ref = orientation;
			}
			
			if(c_motor_connected == 1) {
				// ------ final PWM is result of superposition of 2 regulators --------
				motor_left_set_power((regulator_distance + regulator_rotation) * (long)c_motor_flip_left);
				motor_right_set_power((regulator_distance - regulator_rotation) * (long)c_motor_flip_right);
			}
			
			break;
		}
		case REGULATOR_LINEAR: {
			
			static long target_speed = 0;
			c_motor_connected = 0;
			long a,b;
			
			// encoder1
			a = minl(c_encoder1_max, 0);
			b = maxl(c_encoder1_max, 0);
			c_setpoint1 = clipl(a,b, c_setpoint1);
			
			static long error_accum1 = 0;
			error_distance = ((long)c_setpoint1 - positionL) * c_pid_lin1;
			target_speed = clipl2(c_speed1, -error_distance);
			error_accum1 += (target_speed - (-vL)) * c_pid_i1;

			if(setpoint1_active) {
				if(absl(error_distance) < c_tol1) {
					start_packet(MSG_ENCODER1_READY);
					end_packet();
					
					setpoint1_active = 0;
					positionL = clipl(a, b, positionL);
					c_setpoint1 = positionL;
					motor_left_set_power(0);
					error_accum1 = 0;
				} else if(error_distance != 0) {
					error_accum1 = clipl2(MOTOR_MAX_POWER, error_accum1);
					motor_left_set_power(error_accum1);
				}
			} else {
				error_accum1 = 0;
			}
			
			// encoder2
			a = minl(c_encoder2_max, 0);
			b = maxl(c_encoder2_max, 0);
			c_setpoint2 = clipl(a, b, c_setpoint2);
			
			static long error_accum2 = 0;
			
			error_distance = ((long)c_setpoint2 - positionR) * c_pid_lin2;
			target_speed = clipl2(c_speed2, -error_distance);
			error_accum2 += (target_speed - (-vR)) * c_pid_i2;
			
			if(setpoint2_active) {
				if(absl(error_distance) < c_tol2) {
					start_packet(MSG_ENCODER2_READY);
					end_packet();

					setpoint2_active = 0;
					positionR = clipl(a, b, positionR);
					c_setpoint2 = positionR;
					motor_right_set_power(0);
					error_accum2 = 0;
				} else if(error_distance != 0) {
					error_accum2 = clipl2(MOTOR_MAX_POWER, error_accum2);
					motor_right_set_power(error_accum2);
				}
			} else {
				error_accum2 = 0;
			}
			
			break;
		}
			
		case REGULATOR_SPEED: {
			
			speed_mode_error_accum1 = clipl2(c_accum_clip, speed_mode_error_accum1 + (c_setpoint1 - vL) * c_pid_i1);
			speed_mode_error_accum2 = clipl2(c_accum_clip, speed_mode_error_accum2 + (c_setpoint2 - vR) * c_pid_i1);
			
			motor_left_set_power(speed_mode_error_accum1 * (long)c_motor_flip_left);
			motor_right_set_power(speed_mode_error_accum2 * (long)c_motor_flip_right);
			
			break;
		}
	}
	
	// periodically send status
	RUN_EACH_NTH_CYCLES(int16_t, c_send_status_interval, send_status_and_position());

	if(c_debug_encoders) {
		RUN_EACH_NTH_CYCLES(int, 50, {
			start_packet(MSG_DEBUG_ENCODER1);
				put_long(positionL);
				put_word(vL);
				put_word(motor_left_get_power());
			end_packet();
			
			start_packet(MSG_DEBUG_ENCODER2);
				put_long(positionR);
				put_word(vR);
				put_word(motor_right_get_power());
			end_packet();
		})
	}
	
	report_status();
}

void set_regulator_mode(int mode) {
	if (mode < 0 || mode > REGULATOR_SPEED) {
		mode = REGULATOR_POSITION;
	}
	c_regulator_mode = mode;
	switch(mode) {
		case REGULATOR_POSITION:
			d_ref = L;
			t_ref = orientation;
			c_motor_connected = 1;
			break;
		case REGULATOR_LINEAR:
			break;
		case REGULATOR_SPEED:
			c_motor_connected = 0;
			c_setpoint1=0;
			c_setpoint2=0;
			speed_mode_error_accum1=0;
			speed_mode_error_accum2=0;
			break;
	}
}


void reset_driver(void) {
	positionR = positionL = 0;
	L = orientation = 0;
	
	motor_init();
	set_speed(0x32);
	set_position(0, 0, 0);
	current_status = STATUS_IDLE;
}

void wait_for_regulator() {
	unsigned long t = sys_time;
	while(sys_time == t) {
		#ifdef SIM
			usleep(100);
		#endif
	}
}

static void setX(int tmp) {
	Xlong = (long long)tmp * K2;
	d_ref = L;
	t_ref = orientation;
	reset_stuck();
	wait_for_regulator();
}

static void setY(int tmp) {
	Ylong = (long long)tmp * K2;
	d_ref = L;
	t_ref = orientation;
	reset_stuck();
	wait_for_regulator();
}

static void setO(int tmp) {
	positionL = -DEG_TO_INC_ANGLE(tmp) / 2;
	positionR =  DEG_TO_INC_ANGLE(tmp) / 2;

	L = 0;
	orientation = (long int)(positionR - positionL) % K1;

	d_ref = L;
	t_ref = orientation;
	reset_stuck();

	wait_for_regulator();
}

void set_position(int X, int Y, int orientation) {
	setX(X);
	setY(Y);
	setO(orientation);
	current_status = STATUS_IDLE;
}

void send_status_and_position(void) {
	// if(!can_send_packet()) return;
	start_packet(MSG_SEND_STATUS_AND_POSITION);
		put_byte(current_status);
		put_word(X);
		put_word(Y);
		put_word(INC_TO_DEG_ANGLE(orientation));
	end_packet();
}

void cmd_pwm_opto() {
	#ifndef SIM
	c_motor_connected = 0;
	while(PORTBbits.RB13 != 1) {
		motor_left_set_power(-1000);
	}
	motor_left_set_power(0);
	c_motor_connected = 0;
	set_regulator_mode(REGULATOR_LINEAR);
	positionL = 0;
	c_setpoint1 = 0;
	
	start_packet('X');
	end_packet();
	#endif
}

void set_speed_accel(float v) {
	c_vmax = v;
	// printf("set_speed: %f\n", c_vmax);
	c_omega = 2 * c_vmax;
	
	if(c_accel == 0) {
		g_accel = c_vmax / ((350-500) * (v / VMAX) + 500);
	} else {
		g_accel = c_vmax / c_accel;
	}
	if(c_alpha == 0) {
		g_alpha = 2 * g_accel;
	} else {
		g_alpha = c_omega / c_alpha;
	}
}


enum State get_status(void) {
	return current_status;
}

void force_status(enum State newStatus) {
	current_status = newStatus;
}

void start_command() {
	keep_count = 0;
}

void report_status() {
	if( c_status_change_report && current_status != last_status ) {
		start_packet(MSG_STATUS_CHANGED);
			put_byte(current_status);
			put_word(X);
			put_word(Y);
			put_word(INC_TO_DEG_ANGLE(orientation));
		end_packet();
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

int m1,m2;

/*
	receive command while moving
	return - OK (continue command) or BREAK/ERROR (break current command)
*/
static char get_command(void)
{	
	report_status();
	
	if(current_status == STATUS_STUCK) return ERROR;
	Packet* pkt = try_read_packet();
	
	if(pkt != 0) {
		switch(pkt->type) {           
			case CMD_SEND_STATUS_AND_POSITION:
				send_status_and_position();
				break;

			case CMD_HARD_STOP:
				// stop and become idle
				stop();
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
				uint32_t val;
				int exponent;
				int sign;
				config_get_as_fixed_point(key, (int32_t*)&val, &exponent, &sign);
				start_packet(CMD_GET_CONFIG);
					put_word(val >> 16);
					put_word(val);
					put_byte(sign);
					put_byte(exponent);
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
			
			case CMD_MOTOR:
				m1 = get_word();
				m2 = get_word();
				break;
				
			case CMD_KEEP_SPEED:
				m1 = (int16_t)get_word() * VMAX / 255;
				m2 = (int16_t)get_word() * VMAX / 255;
				break;
			
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
		}
	}

	return OK;
}


void motor_const(int a, int b) {
	char old_val = c_motor_connected;
	char old_stuck = c_enable_stuck;
	int old_rate_of_change = c_motor_rate_of_change;
	c_motor_connected = 0;
	c_motor_rate_of_change = c_motor_const_roc;
	c_enable_stuck = 0;
	start_command();
	current_status = STATUS_MOVING;
	motor_init();
	m1=a;m2=b;
	while(1) {
		if(get_command() == ERROR) {
			motor_turn_off();
			c_motor_connected = old_val;
			c_motor_rate_of_change = old_rate_of_change;
			c_enable_stuck = old_stuck;
			current_status = STATUS_IDLE;
			break;
		}
		motor_left_set_power(m1);
		motor_right_set_power(m2);
		wait_for_regulator();
	}
}

void speed_const(int a, int b) {
	a = a * VMAX / 255;
	b = b * VMAX / 255;
	
	char old_val = c_motor_connected;
	char old_stuck = c_enable_stuck;
	int old_rate_of_change = c_motor_rate_of_change;
	
	c_motor_rate_of_change = c_motor_const_roc;
	c_enable_stuck = 0;
	
	set_regulator_mode(REGULATOR_SPEED);
	start_command();
	current_status = STATUS_MOVING;
	report_status();
	motor_init();
	m1=a;m2=b;
	while(1) {
		if(get_command() == ERROR) {
			motor_turn_off();
			c_motor_connected = old_val;
			c_motor_rate_of_change = old_rate_of_change;
			c_enable_stuck = old_stuck;
			current_status = STATUS_IDLE;
			break;
		}
		c_setpoint1=m1;
		c_setpoint2=m2;
		wait_for_regulator();
	}
	set_regulator_mode(REGULATOR_POSITION);
}


// turn to point and move to it (Xd, Yd)
void turn_and_go(int Xd, int Yd, char direction) {
	motor_init();
	start_command();
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long L_dist, L0, L1, L2, L3;
	long D0, D1, D2;
	float v_vrh, v_end, v_ref, v0;
	int length;
	long long int Xdlong, Ydlong;
	
	Xdlong = MM_TO_INC(Xd);
	Ydlong = MM_TO_INC(Yd);
	d_ref=L;
	v0 = 0;
	direction = (direction >= 0 ? 1 : -1);

	// turn to end point, find angle to turn
	length = get_distance_to(Xd, Yd);
	if(length > 1) {
		rotate_absolute_angle( RAD_TO_DEG_ANGLE(atan2(Ydlong-Ylong, Xdlong-Xlong)) + (direction < 0 ? 180 : 0) );
	} else {
		current_status = STATUS_ERROR;
		return;
	}

	v_end = c_vmax * /*end_speed*/0 / 256;
	L_dist = MM_TO_INC(length);

	// calculate phase durations
	T1 = (c_vmax - v0) / g_accel;
	L0 = L;
	L1 = current_speed * T1 + g_accel * T1 * T1 / 2;

	T3 = (c_vmax - v_end) / g_accel;
	L3 = c_vmax * T3 - g_accel * T3 * T3 / 2;

	if( (L1 + L3) < L_dist) {
		// can reach c_vmax
		L2 = L_dist - L1 - L3;
		T2 = L2 / c_vmax;
	} else {
		// can't reach c_vmax
		T2 = 0;
		v_vrh = sqrt(g_accel * L_dist + (current_speed * current_speed + v_end * v_end) / 2);
		if( (v_vrh < current_speed) || (v_vrh < v_end) ) {
			current_status = STATUS_ERROR;
			return; //mission impossible
		}

		T1 = (v_vrh - current_speed) / g_accel;
		T3 = (v_vrh - v_end) / g_accel;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	D0 = d_ref;

	current_status = STATUS_MOVING;
	report_status();
	while(t < t3) {
		wait_for_regulator();
		t = sys_time;
		if(get_command() == ERROR) {
			return;
		}
		if(t <= t2) {// refresh angle reference
			if(direction > 0) {
				t_ref = RAD_TO_INC_ANGLE(atan2(Ydlong-Ylong, Xdlong-Xlong));
			} else {
				t_ref = RAD_TO_INC_ANGLE(atan2(Ylong-Ydlong, Xlong-Xdlong));
			}
		}

		if(t <= t1) {// speeding up phase
			v_ref = v0 + g_accel * (t-t0);
			D1 = D2 = d_ref = D0 + direction * (v0 * (t-t0) + g_accel * (t-t0)*(t-t0)/2);
		} else if(t <= t2) {// constant speed phase
			v_ref = c_vmax;
			D2 = d_ref = D1 + direction * c_vmax * (t-t1);
		} else if(t <= t3) {// slowing down phase
			d_ref = D2 + direction * (v_ref * (t-t2) - g_accel * (t-t2) * (t-t2) / 2);
		}
	}
	current_status = STATUS_IDLE;
}


void forward_lazy(int length, uint8_t speed) {
	long L_dist = MM_TO_INC(length);
	long end = d_ref+L_dist;
	current_status = STATUS_MOVING;
	report_status();
	long add_inc = VMAX * speed / 255;
	set_regulator_mode(REGULATOR_SPEED);
	c_setpoint1 = add_inc;
	c_setpoint2 = add_inc;
	while(end-L > 0) {
		wait_for_regulator();
		if(get_command() == ERROR) {
			break;
		}
		d_ref = L;
	}
	d_ref = L;
	set_regulator_mode(REGULATOR_POSITION);
	current_status = STATUS_IDLE;
}

// move robot forward in direction its facing
void forward(int length) {
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long L_dist, L0, L1, L2, L3;
	long D0, D1, D2;
	float v_vrh, v_end, v0, v_ref;
	char sign;
	
	motor_init();
	
	start_packet('K');
		put_byte('D');
	end_packet();

	d_ref=L;
	v_ref=current_speed;
	v0 = v_ref;
	v_end = c_vmax * /*end_speed*/0 / 256;
	sign = (length >= 0) ? 1 : -1;
	
	L_dist = MM_TO_INC(length);

	T1 = (c_vmax - v_ref) / g_accel;
	L0 = L;
	L1 = v_ref * T1 + g_accel * T1 * (T1 / 2);

	T3 = (c_vmax - v_end) / g_accel;
	L3 = c_vmax * T3 - g_accel * T3 * (T3 / 2);
	
	

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
		v_vrh = sqrt(g_accel * sign * L_dist + (v_ref * v_ref + v_end * v_end) / 2);
		if((v_vrh < v_ref) || (v_vrh < v_end))
		{
			current_status = STATUS_ERROR;
			// printf("ERROR\n");
			report_status();
			return; //mission impossible
		}

		T1 = (v_vrh - v_ref) / g_accel;
		T3 = (v_vrh - v_end) / g_accel;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	D0 = d_ref;
	// printf("forw times: %d,%d,%d total: %d %f\n", T1,T2,T3, T1+T2+T3, v_ref);
	current_status = STATUS_MOVING;
	report_status();
	while(t < t3)  {
		// printf("t,t3: %ld %ld\n", t, t3);
		// printf("k2: %f\n",K2);
		// printf("L_dist: %ld L1: %ld L2: %ld L3: %ld\n", L_dist, L1,L2,L3);
		// printf("T1: %ld, T2: %ld, T3: %ld\n", T1, T2, T3);
		// printf("speed: %f accel: %f omega: %f alpha: %f ", c_vmax, g_accel, c_omega, g_alpha);
		wait_for_regulator();
		t = sys_time;
		
		if(get_command() == ERROR) {
			return;
		}
		
		if(t <= t1)
		{
			v_ref = v0 + g_accel * (t-t0);
			D1 = D2 = d_ref = D0 + sign * (v0 * (t-t0) + g_accel * (t-t0)*(t-t0)/2);
		}
		else if(t <= t2)
		{
			v_ref = c_vmax;
			D2 = d_ref = D1 + sign * c_vmax * (t-t1);
		}
		else if(t <= t3)
		{
			// v_ref = c_vmax - g_accel * (t-t2);
			d_ref = D2 + sign * (v_ref * (t-t2) - g_accel * (t-t2) * (t-t2) / 2);
		}
		// printf("POS %ld %ld\n", X, Y);
	}
	// printf("DONE %ld %ld\n", X, Y);
	current_status = STATUS_IDLE;
}

void rotate_absolute_angle(int angle) {
	int tmp = angle - INC_TO_DEG_ANGLE(orientation);
	tmp = deg_angle_range_normalize(tmp);
	turn(tmp);
}

char turn(int angle) {
	long t, t0, t1, t2, t3;
	long T1, T2, T3;
	long Fi_total, Fi1;
	double angle_ref, w_ref = 0;
	char sign;
		
	motor_init();

	sign = angle >= 0 ? 1 : -1;

	Fi_total = DEG_TO_INC_ANGLE(angle);
	float w_max;
	w_max = c_omega;
	T1 = T3 = c_omega / g_alpha;
	Fi1 = g_alpha * T1 * T1 / 2;
	if(Fi1 > (sign * Fi_total / 2)) {
		// triangle profile (speeds up to some speed and then slows down to 0)
		Fi1 = sign  * Fi_total / 2;
		T1 = T3 = sqrt(2 * Fi1 / g_alpha);
		T2 = 0;
		w_max = w_max * (g_alpha * T1) / c_omega;
	} else {
		// trapezoid profile of speed graph (similar like triangle profile except there is period of constant speed in between)
		T2 = (sign * Fi_total - 2 * Fi1) / c_omega;
	}

	angle_ref = t_ref;
	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	
	dbg(printf("T BEF: %lf\n", angle_ref);)
	dbg(printf("T PLAN: %ld => %ld\n", Fi_total, angle_ref+Fi_total);)

	// printf("turn times: %d,%d,%d total: %d rots: %ld %ld %ld\n", T1,T2,T3, T1+T2+T3, Fi_total, Fi1, K1);
	
	current_status = STATUS_ROTATING;
	report_status();
	while(t <= t3) {
		wait_for_regulator();
		t = sys_time;
		if(get_command() == ERROR) return ERROR;

		if(t <= t1) {
			w_ref = w_max * (t-t0)/T1;
			angle_ref += sign * (w_ref - g_alpha / 2);
		} else if(t <= t2) {
			w_ref = w_max;
			angle_ref += sign * c_omega;
		} else if(t <= t3) {
			w_ref = w_max - w_max * (t-t2)/T1;
			angle_ref += sign * (w_ref + g_alpha / 2);
		}
		t_ref = angle_ref;
	}
	dbg(printf("T AFT: %lf\n", angle_ref);)
	current_status = STATUS_IDLE;
	return OK;
}

void arc(long Xc, long Yc, int Fi, char direction) {
	double R, dist_ref, angle_ref, w_ref = 0, v_ref = 0;
	uint32_t t, t0, t1, t2, t3;
	int32_t T1, T2, T3;
	int8_t s;

	motor_init();
	s = sign(Fi);
	direction =  direction > 0 ? 1 : -1;
	R = get_distance_to(Xc, Yc);
	if(R > 0) {
		rotate_absolute_angle( RAD_TO_DEG_ANGLE( atan2((int)Y-(int)Yc, (int)X-(int)Xc) ) + direction * s * 90);
	}
	
	float w_max, v_max, speed, accel;
	int32_t L_dist;
	if (R > wheel_distance) {
		speed = c_vmax;
		accel = g_accel;
		w_max = speed * wheel_distance / R;
		v_max = speed;
		L_dist = absl(DEG_TO_RAD_ANGLE(Fi)*MM_TO_INC(R));
	} else {
		speed = c_omega;
		accel = g_alpha;
		w_max = speed;
		v_max = speed * R / wheel_distance;
		L_dist = absl( DEG_TO_INC_ANGLE(Fi) );
	}
	
	T1 = T3 = speed / accel;
	int32_t L1 = accel * T1 * T1 / 2;
	
	if(L1 > L_dist / 2) {
		// triangle profile
		L1 = L_dist / 2;
		T1 = T3 = sqrt(2 * L1 / accel);
		T2 = 0;
		w_max = w_max * (accel * T1) / speed;
		v_max = v_max * (accel * T1) / speed;
	} else {
		// trapezoid profile
		T2 = (L_dist - 2*L1) / speed;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	angle_ref = t_ref;
	dist_ref = d_ref;
	// printf("T BEF: %f\n", angle_ref);
	// printf("T PLAN: %ld => %ld\n", Fi_total, (long)(angle_ref+Fi_total));
	current_status = STATUS_MOVING;
	report_status();
	while(t <= t3) {
		wait_for_regulator();
		t = sys_time;
		
		if(get_command() == ERROR) {
			break;
		}

		if(t <= t1) {
			w_ref = w_max * (t-t0)/T1;
			v_ref = v_max * (t-t0)/T1;
		} else if(t <= t2) {
			w_ref = w_max;
			v_ref = v_max;
		} else if(t <= t3) {
			w_ref = w_max - w_max * (t-t2)/T3;
			v_ref = v_max - v_max * (t-t2)/T3;
		}
		
		angle_ref += s * w_ref;
		dist_ref += direction * absd(v_ref);
		t_ref = angle_ref;
		d_ref = dist_ref;
	}
	// printf("T AFT: %f %ld\n", angle_ref, t_ref);
	current_status = STATUS_IDLE;
}

void arc_relative(int R, int Fi) {
	float dist_ref, angle_ref, w_ref = 0, v_ref = 0;
	uint32_t t, t0, t1, t2, t3;
	int32_t T1, T2, T3;
	int8_t s;

	motor_init();
	s = sign(R) * sign(Fi);
	R = abs(R);
	float w_max, v_max, speed, accel;
	int32_t L_dist;
	if (R > wheel_distance) {
		speed = c_vmax;
		accel = g_accel;
		w_max = speed * wheel_distance / R;
		v_max = speed;
		L_dist = absl(DEG_TO_RAD_ANGLE(Fi)*MM_TO_INC(R));
	} else {
		speed = c_omega;
		accel = g_alpha;
		w_max = speed;
		v_max = speed * R / wheel_distance;
		L_dist = absl( DEG_TO_INC_ANGLE(Fi) );
	}
	
	T1 = T3 = speed / accel;
	int32_t L1 = accel * T1 * T1 / 2;
	
	if(L1 > L_dist / 2) {
		// triangle profile
		L1 = L_dist / 2;
		T1 = T3 = sqrt(2 * L1 / accel);
		T2 = 0;
		w_max = w_max * (accel * T1) / speed;
		v_max = v_max * (accel * T1) / speed;
	} else {
		// trapezoid profile
		T2 = (L_dist - 2*L1) / speed;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	angle_ref = t_ref;
	dist_ref = d_ref;
	// printf("T BEF: %f\n", angle_ref);
	// printf("T PLAN: %ld => %ld\n", Fi_total, (long)(angle_ref+Fi_total));
	current_status = STATUS_MOVING;
	report_status();
	while(t <= t3) {
		wait_for_regulator();
		t = sys_time;
		
		if(get_command() == ERROR) {
			break;
		}

		if(t <= t1) {
			w_ref = w_max * (t-t0)/T1;
			v_ref = v_max * (t-t0)/T1;
		} else if(t <= t2) {
			w_ref = w_max;
			v_ref = v_max;
		} else if(t <= t3) {
			w_ref = w_max - w_max * (t-t2)/T3;
			v_ref = v_max - v_max * (t-t2)/T3;
		}
		
		angle_ref += s * w_ref;
		dist_ref += sign(Fi) * absd(v_ref);
		t_ref = angle_ref;
		d_ref = dist_ref;
	}
	// printf("T AFT: %f %ld\n", angle_ref, t_ref);
	current_status = STATUS_IDLE;
}

void diff_drive(int x, int y, int Fi) {
	t_ref = orientation;
	d_ref = L;
	dbg(printf("x: %d, y: %d, fi: %d\n",x,y,Fi);)
	float w_max=0,v_max=0;
	float cur_theta, goal_theta, theta, angle_to_heading;
	float a,b;
	int d;
	double v,w;
	double v_old=0,w_old=0;
	long dist;
	
	current_status = STATUS_MOVING;
	report_status();
	while(1) {
		wait_for_regulator();
		
		if(get_command() == ERROR) {
			break;
		}
		
		angle_to_heading = atan2(y-(int)Y, x-(int)X);
		cur_theta = INC_TO_RAD_ANGLE(orientation);
		a = angle_to_heading - cur_theta;
		
		goal_theta = DEG_TO_RAD_ANGLE( Fi );
		theta = rad_angle_range_normalize( cur_theta - goal_theta );
		b = -(theta + a); // goal_theta - angle_to_heading
		
		int direction = 1;
		
		d = get_distance_to(x,y);
		a = rad_angle_range_normalize(a);
		b = rad_angle_range_normalize(b);
		
		// printf("theta: %f dist: %d\n", theta, d);
		int linear_tolerance = 50;
		dist = MM_TO_INC(d);
		if(d < linear_tolerance) {
			// printf("lin tolerance\n");
			v = 0;
			w = RAD_TO_INC_ANGLE( c_kb *0.001* theta );
			if(absf(theta) < DEG_TO_RAD_ANGLE(5)) {
				break;
			}
		} else {
			v = c_kp * dist * direction;
			w = RAD_TO_INC_ANGLE(c_ka * a + c_kb * b);
		}
		
		v_max = c_vmax;
		w_max = c_omega;
		
		dbg(printf("w_max: (%f %lf), v_max: (%f %lf)   cur_theta: %f   angle_to_heading: %f\n", w_max, w, v_max, v, RAD_TO_DEG_ANGLE(cur_theta), RAD_TO_DEG_ANGLE(angle_to_heading));)
		
		float ratio;
		if (absd(v) > v_max) {
			ratio = v_max / absd(v);
			w = w * ratio;
			v = v * ratio;
		}
		
		if (absd(w) > w_max) {
			ratio = w_max / absd(w);
			w = w * ratio;
			v = v * ratio;
		}
		
		if (d > linear_tolerance && absd(v) > 0.05 && absd(v) < VMAX * 0.03) {
			ratio = (VMAX * 0.03) / absd(v);
			w = w * ratio;
			v = v * ratio;
		}
		
		w = clipd_margin(w_old, g_alpha, w);
		v = clipd_margin(v_old, g_accel, v);
		
		dbg(printf("- w_max: (%f %lf), v_max: (%f %lf)   cur_theta: %f   angle_to_heading: %f\n", w_max, w, v_max, v, RAD_TO_DEG_ANGLE(cur_theta), RAD_TO_DEG_ANGLE(angle_to_heading));)
		t_ref += w;
		d_ref += v;
		v_old = v;
		w_old = w;
    }
    current_status = STATUS_IDLE;
}

/*
	direction:
		0 - pick smallest rotation
		1 - forward
	   -1 - backward
*/
void move_to(long x, long y, int radius, char direction) {
	move_cmd_next.active = 0;
	float speed = current_speed;
	float rotation_speed = angular_speed; /* [inc/ms] */
	float min_speed = c_speed_drop * c_vmax;
	
	motor_init();
	
	radius = absl(radius);
	float dist = get_distance_to(x,y);
	float start_dist;
	
	float v_div_w = (float)MM_TO_INC( minf(dist, radius)/2.0f ) / (float)RAD_TO_INC_ANGLE(1);
	float w_div_v = 1.0f/v_div_w;
	
	float w;
	float v;

	long goal_angle;
	long angle_diff;
	goal_angle = RAD_TO_INC_ANGLE(atan2(y-Y, x-X));
	angle_diff = inc_angle_diff(goal_angle, orientation);
	
	if(direction > 0) direction = 1;
	else if(direction < 0) direction = -1;
	else {
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
	
	sys_time = 0;
	unsigned long dt;
	
	int slowdown_phase = 0;
	int maxspeed_phase = 0;

	int xd, yd;
	
	float accel;
	float alpha;
	start_command();
	int lt = sys_time;
	xd = X - x;
	yd = Y - y;
	while(1) {
		if(get_command() == ERROR) {
			return;
		}
		
		dt = sys_time - lt;
		
		accel = g_accel * dt;
		alpha = g_alpha * dt;
				
		goal_angle = RAD_TO_INC_ANGLE( atan2(y-Y, x-X) );
		long orient = orientation;
		
		if(direction == -1) {
			orient = angle_range_normalize_long( orient + DEG_TO_INC_ANGLE( 180 ), K1 );
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
			speed -= dval(ss, g_accel);
			if(speed == 0.0f) {
				speed += dval(direction, 0.01f);
			}
		} else {
			t = abs_speed/g_accel * c_slowdown;
			if(slowdown_phase == 1 || abs_speed*t/2.0f > MM_TO_INC(dist)) {
				if(slowdown_phase == 0) {
					slowdown_phase = 1;
					start_dist = dist;
					if(move_cmd_next.active) {
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
						min_speed = v;
					}
				}

				speed = dval(ss, maxf(min_speed, abs_speed-accel ));
				
				if(dist < 2.0f || ((X - x) * xd + (Y-y) * yd) > 0) {
					if(move_cmd_next.active == 1) {
						
						// done cmd
						start_packet('N');
						end_packet();
						
						move_cmd_next.active = 0;
						direction = move_cmd_next.direction;
						x = move_cmd_next.x;
						y = move_cmd_next.y;
						
						xd = X - x;
						yd = Y - y;
						radius = move_cmd_next.radius;
						min_speed = c_speed_drop * c_vmax;
						slowdown_phase = 0;
						continue;
					} else {
						d_ref = D;
						break;
					}
				}
				
				if(speed <= min_speed || dist < 2.0f) {
					accel = minf(accel * dist / start_dist, accel);
					
					speed = dval(ss, maxf(0.0f, abs_speed - accel));
					if(move_cmd_next.active != 1 && speed == 0.0f) {
						d_ref = D;
						
						break;
					}
				}
			} else if(abs_angle_diff < DEG_TO_INC_ANGLE(c_angle_speedup)) {
				maxspeed_phase = 1;
				// 0 -> vmax, 25 -> v
				// vmax-v / 25 + v
				float virt_max = c_vmax; //((v - c_vmax) * (float)abs_angle_diff / (float)DEG_TO_INC_ANGLE(c_angle_speedup)) + c_vmax;
				if(abs_speed < virt_max) {
					speed = dval(ss, minf(abs_speed+accel, virt_max));
				} else {
					speed = dval(ss, maxf(abs_speed-accel, virt_max));
				}
			} else {
				
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
			if(abs_angle_diff > DEG_TO_INC_ANGLE(2)) {
				t = abs_rotation_speed/g_alpha * c_slowdown_angle;
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
		if(absf(speed) > g_accel) {
			speed -= signf(speed) * g_accel;
		} else {
			speed = 0.0f;
		}
		
		if(absf(ang_speed) > g_alpha) {
			ang_speed -= dfval(ang_speed, g_alpha);
		} else {
			ang_speed = 0.0f;
		}
		d_ref += speed;
		t_ref += ang_speed;
		
		wait_for_regulator();
	}
	current_status = STATUS_IDLE;
}

void soft_stop(void) {
	motor_turn_off();
}

void set_rotation_speed(unsigned char max_speed, unsigned char max_accel) {
	c_omega = VMAX * (unsigned char)max_speed / 256;
	
	if(c_omega < (VMAX * 161 / 256)) {
		// in 500 ms speeds up to c_vmax
		g_alpha = c_omega / (500 /*[ms]*/);
	} else {
		// in 375 ms speeds up to c_vmax
		g_alpha = c_omega / (375 /*[ms]*/);
	}
}

void set_speed(unsigned char tmp)
{
	set_speed_accel(VMAX * (unsigned char)tmp / 256);
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



// ========[ CONFIG ]=============


static void on_odometry_coefficients_changed() {
	wheel_distance = c_wheel_distance;
	R_wheel = c_wheel_r1;
	wheel_correction_coeff = (c_wheel_r2 / c_wheel_r1);
	K1 = (long)(4.0*2048.0 * wheel_distance / (R_wheel / 2.0)); // for angles
	K2 = (float)(4.0*2048.0 / (R_wheel * PI)); // for distance
}

static void on_vmax_change() {
	c_vmax = VMAX * (unsigned char)c_vmax / 256;
}

static void on_omega_change() {
	c_omega = VMAX * (unsigned char)c_omega / 256;
}

static void on_encoder1_change() {
	positionL = c_encoder1;
}
static void on_encoder2_change() {
	positionR = c_encoder2;
}

static void on_accel_changed() {
	if(c_accel == 0) {
		// default accel calc
		g_accel = c_vmax / ((350-500) * (c_vmax / VMAX) + 600);
	} else {
		g_accel = c_vmax / c_accel;
	}
}

static void on_alpha_changed() {
	if(c_alpha == 0) {
		// default alpha calc
		g_alpha = 2 * g_accel;
	} else {
		g_alpha = c_omega / c_alpha;
	}
}

static void on_setpoint1() {
	if(c_regulator_mode == REGULATOR_LINEAR) {
		setpoint1_active = 1;
	}
}

static void on_setpoint2() {
	if(c_regulator_mode == REGULATOR_LINEAR) {
		setpoint2_active = 1;
	}
}

void regulator_init(void) {
	move_cmd_next.active = 0;
	
	c_accel = 0;
	c_alpha = 0;
	
	config_on_change(CONF_WHEEL_R1, on_odometry_coefficients_changed);
	config_on_change(CONF_WHEEL_R2, on_odometry_coefficients_changed);
	config_on_change(CONF_WHEEL_DISTANCE, on_odometry_coefficients_changed);
	config_on_change(CONF_OMEGA, on_omega_change);
	config_on_change(CONF_VMAX, on_vmax_change);
	
	config_on_change(CONF_ENCODER1, on_encoder1_change);
	config_on_change(CONF_ENCODER2, on_encoder2_change);
	
	config_on_change(CONF_ALPHA, on_alpha_changed);
	config_on_change(CONF_ACCEL, on_accel_changed);
	
	config_on_change(CONF_SETPOINT1, on_setpoint1);
	config_on_change(CONF_SETPOINT2, on_setpoint2);
}



