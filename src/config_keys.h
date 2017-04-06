// THIS FILE IS GENERATED WITH PYTHON SCRIPT "config_keys_gen.py" and must only be included within "config.h"
#ifdef CONFIG_H

#define CONFIG_MAX_BYTES 5
#define CONFIG_MAX_INTS 4
#define CONFIG_MAX_FLOATS 11


enum ConfEnum { 
	CONF_DISTANCE_REGULATOR,
	CONF_ROTATION_REGULATOR,
	CONF_STUCK,
	CONF_DEBUG,
	CONF_STATUS_CHANGE_REPORT,

	CONF_STUCK_DISTANCE_JUMP,
	CONF_STUCK_ROTATION_JUMP,
	CONF_STUCK_DISTANCE_MAX_FAIL_COUNT,
	CONF_STUCK_ROTATION_MAX_FAIL_COUNT,

	CONF_WHEEL_DISTANCE,
	CONF_WHEEL_R1,
	CONF_WHEEL_R2,
	CONF_PID_D_P,
	CONF_PID_D_D,
	CONF_PID_R_P,
	CONF_PID_R_D,
	CONF_SPEED,
	CONF_RSPEED,
	CONF_ACCEL,
	CONF_ALPHA,
};


#define c_distance_regulator config_bytes[CONF_DISTANCE_REGULATOR-CONFIG_BYTE_OFFSET]
#define c_rotation_regulator config_bytes[CONF_ROTATION_REGULATOR-CONFIG_BYTE_OFFSET]
#define c_stuck config_bytes[CONF_STUCK-CONFIG_BYTE_OFFSET]
#define c_debug config_bytes[CONF_DEBUG-CONFIG_BYTE_OFFSET]
#define c_status_change_report config_bytes[CONF_STATUS_CHANGE_REPORT-CONFIG_BYTE_OFFSET]

#define c_stuck_distance_jump config_ints[CONF_STUCK_DISTANCE_JUMP-CONFIG_INT_OFFSET]
#define c_stuck_rotation_jump config_ints[CONF_STUCK_ROTATION_JUMP-CONFIG_INT_OFFSET]
#define c_stuck_distance_max_fail_count config_ints[CONF_STUCK_DISTANCE_MAX_FAIL_COUNT-CONFIG_INT_OFFSET]
#define c_stuck_rotation_max_fail_count config_ints[CONF_STUCK_ROTATION_MAX_FAIL_COUNT-CONFIG_INT_OFFSET]

#define c_wheel_distance config_floats[CONF_WHEEL_DISTANCE-CONFIG_FLOAT_OFFSET]
#define c_wheel_r1 config_floats[CONF_WHEEL_R1-CONFIG_FLOAT_OFFSET]
#define c_wheel_r2 config_floats[CONF_WHEEL_R2-CONFIG_FLOAT_OFFSET]
#define c_pid_d_p config_floats[CONF_PID_D_P-CONFIG_FLOAT_OFFSET]
#define c_pid_d_d config_floats[CONF_PID_D_D-CONFIG_FLOAT_OFFSET]
#define c_pid_r_p config_floats[CONF_PID_R_P-CONFIG_FLOAT_OFFSET]
#define c_pid_r_d config_floats[CONF_PID_R_D-CONFIG_FLOAT_OFFSET]
#define c_speed config_floats[CONF_SPEED-CONFIG_FLOAT_OFFSET]
#define c_rspeed config_floats[CONF_RSPEED-CONFIG_FLOAT_OFFSET]
#define c_accel config_floats[CONF_ACCEL-CONFIG_FLOAT_OFFSET]
#define c_alpha config_floats[CONF_ALPHA-CONFIG_FLOAT_OFFSET]


void config_set_b(int key, char value);
void config_set_i(int key, int value);
void config_set_f(int key, float value);

static inline void config_load_defaults(void) {
	config_set_b(CONF_DISTANCE_REGULATOR, 0);
	config_set_b(CONF_ROTATION_REGULATOR, 0);
	config_set_b(CONF_STUCK, 0);
	config_set_b(CONF_DEBUG, 0);
	config_set_b(CONF_STATUS_CHANGE_REPORT, 0);
	config_set_i(CONF_STUCK_DISTANCE_JUMP, 0);
	config_set_i(CONF_STUCK_ROTATION_JUMP, 0);
	config_set_i(CONF_STUCK_DISTANCE_MAX_FAIL_COUNT, 0);
	config_set_i(CONF_STUCK_ROTATION_MAX_FAIL_COUNT, 0);
	config_set_f(CONF_WHEEL_DISTANCE, 330.7f);
	config_set_f(CONF_WHEEL_R1, 92.936704f);
	config_set_f(CONF_WHEEL_R2, 92.936704f);
	config_set_f(CONF_PID_D_P, 0.0f);
	config_set_f(CONF_PID_D_D, 0.0f);
	config_set_f(CONF_PID_R_P, 0.0f);
	config_set_f(CONF_PID_R_D, 0.0f);
	config_set_f(CONF_SPEED, 0.0f);
	config_set_f(CONF_RSPEED, 0.0f);
	config_set_f(CONF_ACCEL, 0.0f);
	config_set_f(CONF_ALPHA, 0.0f);
}


#endif

