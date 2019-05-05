#include "config.h"
#include "hw/motor.h"
#include "hw/encoder.h"
#include "hw/interrupt.h"

#include "packet.h"
#include "regulator.h"
#include "math.h"

#undef dbg
#define dbg(...)
int blocked = 0;
void block(int b) {
	blocked = b;
}

int packet_count = 0;
void reset_packet_count() {
	packet_count = 0;
}

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
static long long encL=0,encR=0;
static volatile long X = 0, Y = 0;
// static volatile long current_speed=0, angular_speed=0;
static volatile float current_speed=0, angular_speed=0;
static volatile unsigned long sys_time = 0;

// ----- variables for controlling robot -----
static volatile float orientation = 0;
static volatile ldouble L = 0; // total distance
static volatile long t_ref = 0, d_ref = 0;
static volatile long prev_rotation_error = 0, prev_distance_error = 0;
static volatile int16_t keep_count = 0;

static float keep_rotation_acc = 0, keep_speed_acc = 0;
static float keep_rotation = 0, keep_speed = 0;
// -------------------------------

// stuck detection in regulator
static long prev_orientation = 0;
static long prev_L = 0;
static int t_ref_fail_count = 0;
static int d_ref_fail_count = 0;

// static int16_t send_status_counter = 0;

static enum State current_status = STATUS_IDLE;
static enum State last_status = STATUS_IDLE;

struct filter_t filter_speed1;
struct filter_t filter_speed2;
int16_t filter_speed_array1[5]={0};
int16_t filter_speed_array2[5]={0};
int16_t filter_speed_coef[5] = {1,1,1,1,1};

float filt_speed = 0;
float filt_ang_speed = 0;

// ============== HELPER FUNCTIONS =============

long inc_angle_diff(long a, long b) {
	return angle_range_normalize_long(a - b, K1);
}

float get_distance_to(long x, long y) {
	return sqrt((x-X)*(x-X) + (y-Y)*(y-Y));
}

// ==========================================


// --------------------------------------------------------------
// volatile int check = 0;
// **********************************************************************
// ODOMETRY AND REGULATION (every 1ms)
// enters periodically every 1ms, and can last 30000 cpu cycles max (but recommended to last at most half of that, 15000 cycles)
// *********************************************************************

/*
void INTERRUPT _INT1Interrupt(void) {
	#ifndef SIM
	IFS1bits.INT1IF = 0;
	RUN_EACH_NTH_CYCLES(uint16_t, 200, {
		start_packet('X');
		end_packet();
	})
	#endif
}

void INTERRUPT _INT2Interrupt(void) {
	start_packet('x');
	end_packet();
	#ifndef SIM
	IFS1bits.INT2IF = 0;
	RUN_EACH_NTH_CYCLES(uint16_t, 200, {
		start_packet('x');
		end_packet();
	})
	#endif
}
*/




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
#include <stdlib.h>
static long speed_mode_error_accum1 = 0;
static long speed_mode_error_accum2 = 0;
static volatile ldouble prev_positionR = 0;
void regulator_interrupt(void) {
	
	static float vL,vR;
	static long sint, cost, theta = 0;
	static signed long regulator_distance, regulator_rotation;
	static long error_distance, error_angular;
	static float x, y;
	
	sys_time++;
	
	is_interrupt = 1;
	
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
		
		keep_rotation_acc += keep_rotation;
		keep_speed_acc += keep_speed;
		
		int16_t k_rot = keep_rotation_acc;
		int16_t k_speed = keep_speed_acc;
		
		t_ref += k_rot;
		d_ref += k_speed;
		
		keep_rotation_acc -= k_rot;
		keep_speed_acc -= k_speed;
		
		// t_ref += keep_rotation;
		// d_ref += keep_speed;
		
		// trapezoid reduction
		if(--keep_count == 0) {
			report_status(STATUS_IDLE);
		}
	}

	// ODOMETRY, reading encoders and processing them
	//************************************************************************
	
	// read left encoder
	vL = encoder_odometry_left_get_velocity();
	encL += vL;
	positionL = encL;
	
	// read right encoder
	vR = encoder_odometry_right_get_velocity();
	encR += vR;
	positionR = (encR * c_wheel_r2) / c_wheel_r1;
	//positionR = encR;
	vR = (vR * c_wheel_r2) / c_wheel_r1;
	prev_positionR = positionR;
	//printf("%d %d\n", (int)vL, (int)vR);
	
	static float s_vL = 0;
	static float s_vR = 0;
	s_vL += vL;
	s_vR += vR;
	
	// speed
	current_speed = (vL + vR) / 2; // v = s/t, current_speed [inc/ms]
	float dbl = (vL + vR); // v = s/t, current_speed [inc/ms]
	angular_speed = vR - vL; // angular speed [inc/ms]

	// filtered speeds
	filt_speed = filter_in(&filter_speed1, current_speed);
	filt_ang_speed = filter_in(&filter_speed2, angular_speed);
	
	// position
	L = (positionL + positionR) / 2;
	orientation = angle_range_normalize_float(positionR - positionL, K1);
	// printf("orientation: %f speed: %ld\n", orientation, angular_speed);
	
	// convert range from [-K1/2, K1/2] to [-2*SINUS_MAX, 2*SINUS_MAX]
	theta = orientation * 2*SINUS_MAX / (K1 / 2);
	sin_cos(theta, &sint, &cost);
	
	x = dbl * cost / 2;
	y = dbl * sint / 2;

	// update increment position
	Xlong += (x/SINUS_AMPLITUDE);
	Ylong += (y/SINUS_AMPLITUDE);
	
	// translate increment position to millimeters
	X = INC_TO_MM(Xlong);
	Y = INC_TO_MM(Ylong);
	
	// printf("%d %d : %d %d : %f | %f %f\n", (int)L, (int)INC_TO_MM(L), (int)positionL, (int)positionR, s_vL, s_vR);
	
	// errors
	error_distance = d_ref - L;
	error_angular = angle_range_normalize_long(orientation - t_ref, K1);

	// ---------[ Stuck Detection ]----------
	
	if( c_enable_stuck == 1 ) {
		// distance stuck
		if(absl(error_distance-prev_distance_error) > MM_TO_INC(c_stuck_distance_jump)) {
			report_status(STATUS_STUCK);
		}
		
		if((signl(error_distance) != signl(current_speed) && absl(current_speed) > 6) || 
			(absl(regulator_distance) > MOTOR_MAX_SPEED/4 && absl(current_speed) < 3)) {
			if(++d_ref_fail_count > c_stuck_distance_max_fail_count) {
				report_status(STATUS_STUCK);
				d_ref_fail_count = 0;
			}
		} else {
			d_ref_fail_count = 0;
		}
		
		// angular stuck
		if(absl(error_angular-prev_rotation_error) > DEG_TO_INC_ANGLE(c_stuck_rotation_jump)) {
			report_status(STATUS_STUCK);
		}
		
		if(absl(error_angular) > DEG_TO_INC_ANGLE(1) && (signl(error_angular) != signl(-angular_speed) || 
			absl(angular_speed) < DEG_TO_INC_ANGLE(0.1))) {
			if(++t_ref_fail_count > c_stuck_rotation_max_fail_count) {
				report_status(STATUS_STUCK);
				t_ref_fail_count = 0;
			}
		} else {
			t_ref_fail_count = 0;
		}
		
		// if robot stuck, then shut down engines
		if(current_status == STATUS_STUCK) {
			c_stuck = 1;
			c_rotation_regulator = c_distance_regulator = 0;
			motor_turn_off();
		}
	}
	prev_distance_error = error_distance;
	prev_rotation_error = error_angular;
	
	// -------------------------------------------
	
	// REGULATOR
	//*************************************************************************
	
	switch(c_regulator_mode) {
		
		case REGULATOR_POSITION: {
		
			long speed_error = error_distance - (-current_speed);
			static float speed_accum_d = 0;
			speed_accum_d += speed_error * c_accum_speed;
			speed_accum_d = clip2(c_accum_clip, speed_accum_d);
		
			// PID 					proportional 						integral 						differential
			regulator_distance = (error_distance * c_pid_d_p)  +  (speed_accum_d * c_pid_d_i)  -  (c_pid_d_d * current_speed);
			// dbg(printf("error_dist: %ld d_ref: %ld L: %lf K1: %ld K2: %f vL: %f vR: %f\n", speed_error, d_ref, L, K1, K2, vL, vR));
			
			long rot_speed_error = error_angular - (-current_speed);
			static float speed_accum_r = 0;
			speed_accum_r += rot_speed_error * c_accum_speed;
			speed_accum_r = clip2(c_accum_clip, speed_accum_r);
	
			// PID 					proportional 					integral 						differential
			regulator_rotation = (error_angular * c_pid_r_p)  +  (speed_accum_r * c_pid_r_i) - 	(-angular_speed * c_pid_r_d);
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
				put_byte(vL);
				put_word(motor_left_get_power());
			end_packet();
			
			start_packet(MSG_DEBUG_ENCODER2);
				put_long(positionR);
				put_byte(vR);
				put_word(motor_right_get_power());
			end_packet();
		
			start_packet(MSG_DEBUG_ENCODER);
				put_long(encL);
				
			end_packet();
			
			
			start_packet(MSG_DEBUG4);
				put_long(encR);
				//put_long(positionR - positionR_2);
			end_packet();
		})
	}
	
	if(c_debug_errors) {
		RUN_EACH_NTH_CYCLES(int, 50, {
			start_packet(MSG_DEBUG_DISTANCE);
				put_long(d_ref);
				put_word(error_distance);
			end_packet();
		
			start_packet(MSG_DEBUG_ROTATION);
				put_long(t_ref);
				put_word(error_angular);
			end_packet();
		})
	}
	is_interrupt = 0;
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
	encR = encL = 0;
	L = orientation = 0;
	
	motor_init();
	set_speed(0x32);
	set_position(0, 0, 0);
	report_status(STATUS_IDLE);
}

unsigned long wait_for_regulator() {
	unsigned long t = sys_time;
	while(sys_time == t) {
		#ifdef SIM
			usleep(10);
		#endif
	}
	return sys_time;
}

void keep_moving() {
	keep_count = c_keep_count;
	keep_rotation_acc = keep_speed_acc = 0;
	
	// keep_speed = current_speed;
	// keep_rotation = angular_speed;
	
	printf("keep moving: %f:%f  %f:%f\n", current_speed, filt_speed, angular_speed, filt_ang_speed);
	keep_speed = filt_speed;
	keep_rotation = filt_ang_speed;
}

void set_position(int x, int y, int orient_angle) {
	Xlong = (long long)x * K2;
	Ylong = (long long)y * K2;
	X = INC_TO_MM(Xlong);
	Y = INC_TO_MM(Ylong);
	
	encL = positionL = -DEG_TO_INC_ANGLE(orient_angle) / 2;
	
	positionR =  DEG_TO_INC_ANGLE(orient_angle) / 2;
	encR = positionR * c_wheel_r1 / c_wheel_r2;
	
	L = 0;
	orientation = (long int)(positionR - positionL) % K1;
	
	d_ref = L;
	t_ref = orientation;
	
	reset_stuck();
	wait_for_regulator();
	report_status(STATUS_IDLE);
}

void send_status_and_position(void) {
	// if(!can_send_packet()) return;
	start_packet(MSG_SEND_STATUS_AND_POSITION);
		put_byte(current_status);
		put_word(X);
		put_word(Y);
		put_word((int16_t)INC_TO_DEG_ANGLE(orientation) | (packet_count << 9));
	end_packet();
}

void set_speed_accel(float v) {
	c_vmax = v;
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
	report_status(newStatus);
}

void start_command() {
	motor_init();
	// cancel keep_moving
	keep_count = 0;
	if (blocked) return;
	//packet_count++;
}

void end_command() {
	if (blocked) return;
	if (c_end_speed > 0) {
		keep_moving();
	}
	report_status(STATUS_IDLE);
}

void report_status(int new_status) {
	if(blocked || current_status == new_status) return;
	current_status = new_status;
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
				report_status(STATUS_IDLE);
				__delay_ms(10);

				return BREAK;

			case CMD_SOFT_STOP:
				// stop and turn off PWM
				motor_turn_off();
				report_status(STATUS_IDLE);
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
				// turn off regulator (PWM can active, but regulator not setting PWM), 
				// must be turned on explicitly with control_flags operator
				c_rotation_regulator = c_distance_regulator = 0;
				break;
				
			case CMD_MOVE_TO: {
				move_cmd_next.active = 1;
				move_cmd_next.x = get_word();
				move_cmd_next.y = get_word();
				move_cmd_next.direction = get_byte();
				move_cmd_next.radius = get_word();
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
				printf("pkt: i (break)\n");
				keep_moving();
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
	report_status(STATUS_MOVING);
	m1=a;m2=b;
	while(1) {
		if(get_command() == ERROR) {
			motor_turn_off();
			c_motor_connected = old_val;
			c_motor_rate_of_change = old_rate_of_change;
			c_enable_stuck = old_stuck;
			break;
		}
		motor_left_set_power(m1);
		motor_right_set_power(m2);
		wait_for_regulator();
	}
	report_status(STATUS_IDLE);
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
	report_status(STATUS_MOVING);
	m1=a; m2=b;
	while(1) {
		if(get_command() == ERROR) {
			motor_turn_off();
			c_motor_connected = old_val;
			c_motor_rate_of_change = old_rate_of_change;
			c_enable_stuck = old_stuck;
			break;
		}
		c_setpoint1=m1;
		c_setpoint2=m2;
		wait_for_regulator();
	}
	set_regulator_mode(REGULATOR_POSITION);
	end_command();
}


// goto
// turn to point and move to it (Xd, Yd)
void turn_and_go(int Xd, int Yd, char direction) {
	start_command();
	report_status(STATUS_MOVING);
	long t, t0, t1, t2, t3;
	long T1, T2, T3, D0, D1, D2;
	long L_dist, L0, L1, L2, L3;
	float v_peak, v_end, v_ref, v0;
	int length;
	long long int Xdlong, Ydlong;
	
	Xdlong = MM_TO_INC(Xd);
	Ydlong = MM_TO_INC(Yd);
	d_ref=L;
	v0 = 0;
	direction = (direction >= 0 ? 1 : -1);
	block(1);
	// turn to end point, find angle to turn
	length = get_distance_to(Xd, Yd);
	if(length > 2) {
		rotate_absolute_angle_inc( 
			RAD_TO_INC_ANGLE(atan2(Ydlong-Ylong, Xdlong-Xlong)) + 
			DEG_TO_INC_ANGLE(direction < 0 ? 180 : 0)
		);
	} else {
		block(0);
		report_status(STATUS_IDLE);
		return;
	}
	block(0);
	v_end = VMAX * c_end_speed / 256;
	L_dist = MM_TO_INC(length);
	
	float vmax;
	
	if(c_debug) {
		start_packet(MSG_DEBUG);
			put_long(L_dist);
		end_packet();
	}
	
	float accel = g_accel;
	if (accel == 0) return;
	// calculate phase durations
	T1 = maxf(0, c_vmax - v0) / accel + 0.5;
	L0 = L;
	L1 = current_speed * T1 + accel * T1 * T1 / 2;

	T3 = maxf(0, c_vmax - v_end) / accel + 0.5;
	L3 = c_vmax * T3 - accel * T3 * T3 / 2;

	if( (L1 + L3) < L_dist ) {
		// can reach c_vmax
		L2 = L_dist - L1 - L3;
		// T2 = ceil(L2 / c_vmax);
		T2 = L2 / c_vmax + 0.5;
		vmax = c_vmax;
	} else {
		// can't reach c_vmax
		T2 = 0;
		v_peak = sqrt(accel * L_dist + (current_speed * current_speed + v_end * v_end) / 2);
		if( (v_peak < current_speed) || (v_peak < v_end) ) {
			report_status(STATUS_ERROR);
			return; //mission impossible
		}
		vmax = v_peak;
		T1 = (v_peak - current_speed) / accel + 0.5;
		T3 = (v_peak - v_end) / accel + 0.5;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	
	long T = T1+T2+T3;
	if(T <= 10) {
		end_command();
		return;
	}
	
	D0 = d_ref;
	long orig = d_ref;
	long ref = 0;
	long ref_max = accel * T1*T1/2 + vmax * T2 + accel * T3 * T3 / 2;
	long ref_err = (L_dist - ref_max) ;
	// ref_err = 0;
	
	while(t <= t3) {
		t = sys_time;
		if(get_command() == ERROR) {
			break;
		}
		if(t <= t2) {// refresh angle reference
			if(direction > 0) {
				t_ref = RAD_TO_INC_ANGLE(atan2(Ydlong-Ylong, Xdlong-Xlong));
			} else {
				t_ref = RAD_TO_INC_ANGLE(atan2(-(Ydlong-Ylong), -(Xdlong-Xlong)));
			}
		}
		
		if(t <= t1) {// speeding up phase
			ref = accel * (t-t0)*(t-t0)/2;
		} else if(t <= t2) {// constant speed phase
			ref = (accel * T1*T1/2) + vmax * (t-t1);
		} else if(t <= t3) {// slowing down phase
			ref = (accel * T1*T1/2) + vmax * T2 + (vmax * (t-t2) - accel * (t-t2) * (t-t2) / 2);
		}
		d_ref = orig + direction * (ref + ref_err * (t-t0) / T);
		wait_for_regulator();
	}

	end_command();
}


void forward_lazy(int length, uint8_t speed) {
	long L_dist = MM_TO_INC(length);
	long end = d_ref+L_dist;
	start_command();
	report_status(STATUS_MOVING);
	long add_inc = VMAX * speed / 255;
	set_regulator_mode(REGULATOR_SPEED);
	c_setpoint1 = add_inc;
	c_setpoint2 = add_inc;
	while(end-L > 0) {
		if(get_command() == ERROR) {
			break;
		}
		d_ref = L;
		wait_for_regulator();
	}
	d_ref = L;
	set_regulator_mode(REGULATOR_POSITION);
	end_command();
}

// move robot forward in direction its facing
void forward(int length) {
	long t, t0, t1, t2, t3;
	long L_dist, L0, L1, L2, L3;
	long D0, D1, D2, T1, T2, T3;
	float v_peak, v_end, v0, v_ref;
	char s;
	
	start_command();
	report_status(STATUS_MOVING);
	
	d_ref = L;
	v_ref = current_speed;
	v0 = v_ref;
	v_end = VMAX * c_end_speed / 256;
	s = sign(length);
	float accel = g_accel;
	float vmax = c_vmax;
	L_dist = MM_TO_INC(length);

	T1 = maxf(0, vmax - v_ref) / accel;
	L0 = L;
	L1 = v_ref * T1 + accel * T1 * T1 / 2;

	T3 = maxf(0, vmax - v_end) / accel;
	L3 = vmax * T3 - accel * T3 * T3 / 2;
	
	if((L1 + L3) < (long)s * L_dist) {
		// can reach
		L2 = s * L_dist - L1 - L3;
		T2 = L2 / vmax;
	} else {
		// can't reach vmax
		T2 = 0;
		v_peak = sqrt(accel * s * L_dist + (v_ref * v_ref + v_end * v_end) / 2);
		if( (v_peak < v_ref) || (v_peak < v_end) ) {
			report_status(STATUS_ERROR);
			// printf("ERROR\n");
			return; //mission impossible
		}

		T1 = (v_peak - v_ref) / accel;
		T3 = (v_peak - v_end) / accel;
	}
	
	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	D0 = d_ref;
	// printf("forw times: %d,%d,%d total: %d %f\n", T1,T2,T3, T1+T2+T3, v_ref);
	while(t < t3) {
		
		t = sys_time;
		
		if(get_command() == ERROR) {
			return;
		}
		
		if(t <= t1) {
			v_ref = v0 + accel * (t-t0);
			D1 = D2 = d_ref = D0 + (long)s * (v0 * (t-t0) + accel * (t-t0)*(t-t0)/2);
		} else if(t <= t2) {
			v_ref = vmax;
			D2 = d_ref = D1 + (long)s * vmax * (t-t1);
		} else if(t <= t3) {
			// v_ref = vmax - accel * (t-t2);
			d_ref = D2 + (long)s * (v_ref * (t-t2) - accel * (t-t2) * (t-t2) / 2);
		}
		wait_for_regulator();
	}
	end_command();
}

void rotate_absolute_angle(int angle) {
	rotate_absolute_angle_inc(DEG_TO_INC_ANGLE(angle));
}

void rotate_absolute_angle_inc(int32_t angle) {
	turn_inc(inc_angle_diff(angle, orientation));
}

char turn(int angle) {
	return turn_inc(DEG_TO_INC_ANGLE(angle));
}

char turn_inc(int32_t angle) {
	long t, t0, t1, t2, t3;
	long T1, T2, T3, Fi_total, Fi1;
	double angle_ref, w_ref = 0;
	char sign;

	start_command();
	report_status(STATUS_ROTATING);
	
	sign = angle >= 0 ? 1 : -1;
	Fi_total = angle;
	
	float w_max;
	float alpha = g_alpha;
	w_max = c_omega;
	T1 = T3 = c_omega / alpha;
	Fi1 = alpha * T1 * T1 / 2;
	if(Fi1 > (sign * Fi_total / 2)) {
		// triangle profile (speeds up to some speed and then slows down to 0)
		Fi1 = sign  * Fi_total / 2;
		T1 = T3 = sqrt(2 * Fi1 / alpha) + 0.5;
		// a * T^2/2 = Fi
		T2 = 0;
		w_max = alpha * T1;
	} else {
		// trapezoid profile of speed graph (similar like triangle profile except there is period of constant speed in between)
		T2 = (sign * Fi_total - 2 * Fi1) / c_omega + 0.5;
		w_max = c_omega;
	}
	long T = T1+T2+T3;
	if(T == 0) {
		end_command();
		return OK;
	}
	
	long s = alpha * T1*T1/2 * 2 + T2 * w_max;
	
	if(c_debug) {
		start_packet(MSG_DEBUG);
			put_word(Fi_total);
			put_word(s);
		end_packet();
		start_packet(MSG_DEBUG2);
			put_word(T1);
			put_word(T2);
		end_packet();
		
	}
	
	long orig = t_ref;
	// angle_ref = t_ref;
	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	
	long max_ref = alpha * T1*T1/2 + w_max * T2 + alpha * T3*T3/2;
	long ref_err = absl(Fi_total) - max_ref;
	// ref_err = 0;
	dbg(printf("T BEF: %lf\n", angle_ref);)
	dbg(printf("T PLAN: %ld => %lf\n", Fi_total, angle_ref+Fi_total);)

	// printf("turn times: %d,%d,%d total: %d rots: %ld %ld %ld\n", T1,T2,T3, T1+T2+T3, Fi_total, Fi1, K1);
	// double prev;
	int c1,c2,c3;
	w_ref = 0;
	while(t <= t3) {
		t = sys_time;
		if(get_command() == ERROR) {
			break;
		}

		if(t <= t1) {
			angle_ref = alpha * (t-t0)*(t-t0)/2;
		} else if(t <= t2) {
			angle_ref = alpha * T1*T1/2 + w_max * (t-t1);
		} else if(t <= t3) {
			angle_ref = alpha * T1*T1/2 + w_max * T2+ (w_max*(t-t2) - alpha * (t-t2)*(t-t2)/2);
		}
		t_ref = orig + sign * (angle_ref + ref_err * (t-t0)/T);
		wait_for_regulator();
	}
	dbg(printf("T AFT: %lf\n", angle_ref);)
	end_command();
}

void arc(long Xc, long Yc, int Fi, char direction) {
	double R, dist_ref, angle_ref, w_ref = 0, v_ref = 0;
	uint32_t t, t0, t1, t2, t3;
	int32_t T1, T2, T3;
	int8_t s;

	start_command();
	report_status(STATUS_MOVING);
	
	s = sign(Fi);
	direction =  direction > 0 ? 1 : -1;
	R = get_distance_to(Xc, Yc);
	if(R > 0) {
		block(1);
		rotate_absolute_angle( RAD_TO_DEG_ANGLE( atan2((int)Y-(int)Yc, (int)X-(int)Xc) ) + direction * s * 90);
		block(0);
	}
	
	float w_end, v_end, w_max, v_max, speed, accel;
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
	
	v_end = w_end = VMAX * c_end_speed / 255;
	
	T1 = speed / accel;
	T3 = (speed - v_end) / accel;
	int32_t L1 = accel * T1 * T1 / 2;
	
	if(L1 > L_dist / 2) {
		// triangle profile
		L1 = L_dist / 2;
		T1 = sqrt(2 * L1 / accel);
		w_max = w_max * (accel * T1) / speed;
		v_max = v_max * (accel * T1) / speed;
		T3 = (v_max - v_end) / accel;
		T2 = 0;
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
	while(t <= t3) {
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
			w_ref = w_max - (w_max-w_end) * (t-t2)/T3;
			v_ref = v_max - (v_max-v_end) * (t-t2)/T3;
		}
		
		angle_ref += s * w_ref;
		dist_ref += direction * absd(v_ref);
		t_ref = angle_ref;
		d_ref = dist_ref;
		
		wait_for_regulator();
	}
	// printf("T AFT: %f %ld\n", angle_ref, t_ref);
	end_command();
}

void arc_relative(int R, int Fi) {
	float dist_ref, angle_ref, w_ref = 0, v_ref = 0;
	uint32_t t, t0, t1, t2, t3;
	int32_t T1, T2, T3;
	int8_t s;

	start_command();
	report_status(STATUS_MOVING);
	
	s = sign(R) * sign(Fi);
	R = abs(R);
	float w_max, v_max, speed, accel;
	float w_end, v_end;
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
	
	v_end = w_end = VMAX * c_end_speed / 255;
	
	if (accel == 0) {
		end_command();
		return;
	}
	
	T1 = speed / accel;
	T3 = (speed - v_end) / accel;
	int32_t L1 = accel * T1 * T1 / 2;
	
	if(L1 > L_dist / 2) {
		// triangle profile
		L1 = L_dist / 2;
		T1 = T3 = sqrt(2 * L1 / accel);
		w_max = w_max * (accel * T1) / speed;
		v_max = v_max * (accel * T1) / speed;
		T3 = (v_max - v_end) / accel;
		T2 = 0;
	} else {
		// trapezoid profile
		T2 = (L_dist - 2*L1) / speed;
	}
	
	if (T1 == 0 || T3 == 0) {
		end_command();
		return;
	}

	t = t0 = sys_time;
	t1 = t0 + T1;
	t2 = t1 + T2;
	t3 = t2 + T3;
	angle_ref = t_ref;
	dist_ref = d_ref;
	
	while(t <= t3) {
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
		} else if(T3 > 0 && t <= t3) {
			w_ref = w_max - (w_max-w_end) * (t-t2)/T3;
			v_ref = v_max - (v_max-v_end) * (t-t2)/T3;
		}
		
		angle_ref += s * w_ref;
		dist_ref += sign(Fi) * absd(v_ref);
		t_ref = angle_ref;
		d_ref = dist_ref;
		
		wait_for_regulator();
	}
	end_command();
}

void diff_drive(int x, int y, int Fi) {
	t_ref = orientation;
	d_ref = L;
	dbg(printf("x: %d, y: %d, fi: %d\n",x,y,Fi);)
	float w_max=0,v_max=0;
	float a, b, cur_theta, goal_theta, theta, angle_to_heading;
	double v,w, v_old=0, w_old=0;
	int d;
	long dist;
	start_command();
	report_status(STATUS_MOVING);
	
	while(1) {
		
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
		
		dbg(printf("w_max: (%f %lf), v_max: (%f %lf)   cur_theta: %f   angle_to_heading: %f\n", 
			w_max, w, v_max, v, RAD_TO_DEG_ANGLE(cur_theta), RAD_TO_DEG_ANGLE(angle_to_heading));)
		
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
		
		dbg(printf("- w_max: (%f %lf), v_max: (%f %lf)   cur_theta: %f   angle_to_heading: %f\n", 
			w_max, w, v_max, v, RAD_TO_DEG_ANGLE(cur_theta), RAD_TO_DEG_ANGLE(angle_to_heading));)
		t_ref += w;
		d_ref += v;
		v_old = v;
		w_old = w;
		
		wait_for_regulator();
    }
    end_command();
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
	
	
	start_command();
	report_status(STATUS_MOVING);
	
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
	
	long D = d_ref;
	long R = t_ref;
	
	float t;
	
	v = c_vmax;
	w = w_div_v * v;
	if(w > c_omega) {
		w = c_omega;
		v = v_div_w * w;
		if(v > c_vmax) {
			report_status(STATUS_ERROR);
			return;
		}
	}
	
	sys_time = 0;
	unsigned long dt;
	
	int slowdown_phase = 0;
	int maxspeed_phase = 0;

	int xd, yd, lt = sys_time;
	
	float accel, alpha;
	xd = X - x;
	yd = Y - y;
	while(1) {
		if(get_command() == ERROR) {
			break;
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
		
		char ss = signf(speed), sr = signf(rotation_speed);
		float abs_speed = absf(speed), abs_rotation_speed = absf(rotation_speed);
		
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
					
					// next cmd
					if(move_cmd_next.active) {
						v = c_vmax;
						w = w_div_v * v;
						if(w > c_omega) {
							w = c_omega;
							v = v_div_w * w;
							if(v > c_vmax) {
								report_status(STATUS_ERROR);
								return;
							}
						}
						min_speed = v;
					}
				}

				speed = dval(ss, maxf(min_speed, abs_speed-accel ));
				
				if(dist < 2.0f || ((X - x) * xd + (Y-y) * yd) > 0) {
					
					// next cmd
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
					
					// no more commands, stop
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
	end_command();
}

// hard stop
void stop(void) {
	start_command();
	d_ref = L;
	t_ref = orientation;

	__delay_ms(120);

	d_ref = L;
	t_ref = orientation;

	__delay_ms(20);

	report_status(STATUS_IDLE);
}

// regulated soft stop
void smooth_stop(void) {
	start_command();
	report_status(STATUS_MOVING);
	float speed = current_speed;
	float ang_speed = angular_speed;
	while(speed != 0.0f || ang_speed != 0.0f) {
		
		if(absf(speed) > g_accel) {
			speed -= signf(speed) * g_accel;
		} else {
			speed = 0.0f;
		}
		
		if(absf(ang_speed) > g_alpha) {
			ang_speed -= signf(ang_speed) * g_alpha;
		} else {
			ang_speed = 0.0f;
		}
		
		d_ref += speed;
		t_ref += ang_speed;
		
		wait_for_regulator();
	}
	end_command();
}

// unregulated soft stop
void soft_stop(void) {
	start_command();
	motor_turn_off();
	report_status(STATUS_IDLE);
}

void set_rotation_speed(unsigned char max_speed, unsigned char max_accel) {
	c_omega = VMAX * max_speed / 256;
	
	if(c_omega < (VMAX * 161 / 256)) {
		// in 500 ms speeds up to c_vmax
		g_alpha = c_omega / (500 /*[ms]*/);
	} else {
		// in 375 ms speeds up to c_vmax
		g_alpha = c_omega / (375 /*[ms]*/);
	}
}

void set_speed(unsigned char tmp) {
	set_speed_accel(VMAX * tmp / 256);
}

void reset_stuck() {
	prev_orientation = orientation;
	prev_L = L;
	d_ref_fail_count = t_ref_fail_count = 0;
	prev_distance_error = prev_rotation_error = 0;
	if(c_stuck) {
		c_rotation_regulator = c_distance_regulator = 1;
		c_stuck = 0;
	}
}

// ========================[ CONFIG ]=================================


static void on_odometry_coefficients_changed() {
	wheel_distance = c_wheel_distance;
	//R_wheel = (c_wheel_r1 + c_wheel_r2) / 2;
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
	
	filter_init(&filter_speed1, 5, filter_speed_array1, filter_speed_coef);
	filter_init(&filter_speed2, 5, filter_speed_array2, filter_speed_coef);
	
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



