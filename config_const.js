static get CONFIG_DISTANCE_REGULATOR() { return 0; }
static get CONFIG_ROTATION_REGULATOR() { return 1; }
static get CONFIG_ENABLE_STUCK() { return 2; }
static get CONFIG_STUCK() { return 3; }
static get CONFIG_DEBUG() { return 4; }
static get CONFIG_STATUS_CHANGE_REPORT() { return 5; }
static get CONFIG_KEEP_COUNT() { return 6; }

static get CONFIG_STUCK_DISTANCE_JUMP() { return 7; }
static get CONFIG_STUCK_ROTATION_JUMP() { return 8; }
static get CONFIG_STUCK_DISTANCE_MAX_FAIL_COUNT() { return 9; }
static get CONFIG_STUCK_ROTATION_MAX_FAIL_COUNT() { return 10; }
static get CONFIG_MOTOR_SPEED_LIMIT() { return 11; }
static get CONFIG_MOTOR_RATE_OF_CHANGE() { return 12; }
static get CONFIG_SEND_STATUS_INTERVAL() { return 13; }

static get CONFIG_WHEEL_DISTANCE() { return 14; }
static get CONFIG_WHEEL_R1() { return 15; }
static get CONFIG_WHEEL_R2() { return 16; }
static get CONFIG_PID_D_P() { return 17; }
static get CONFIG_PID_D_D() { return 18; }
static get CONFIG_PID_R_P() { return 19; }
static get CONFIG_PID_R_D() { return 20; }
static get CONFIG_OMEGA() { return 21; }
static get CONFIG_VMAX() { return 22; }
static get CONFIG_SPEED() { return 23; }
static get CONFIG_RSPEED() { return 24; }
static get CONFIG_ACCEL() { return 25; }
static get CONFIG_ALPHA() { return 26; }


static var config_names_enum = [
	'CONFIG_DISTANCE_REGULATOR',
	'CONFIG_ROTATION_REGULATOR',
	'CONFIG_ENABLE_STUCK',
	'CONFIG_STUCK',
	'CONFIG_DEBUG',
	'CONFIG_STATUS_CHANGE_REPORT',
	'CONFIG_KEEP_COUNT',
	'CONFIG_STUCK_DISTANCE_JUMP',
	'CONFIG_STUCK_ROTATION_JUMP',
	'CONFIG_STUCK_DISTANCE_MAX_FAIL_COUNT',
	'CONFIG_STUCK_ROTATION_MAX_FAIL_COUNT',
	'CONFIG_MOTOR_SPEED_LIMIT',
	'CONFIG_MOTOR_RATE_OF_CHANGE',
	'CONFIG_SEND_STATUS_INTERVAL',
	'CONFIG_WHEEL_DISTANCE',
	'CONFIG_WHEEL_R1',
	'CONFIG_WHEEL_R2',
	'CONFIG_PID_D_P',
	'CONFIG_PID_D_D',
	'CONFIG_PID_R_P',
	'CONFIG_PID_R_D',
	'CONFIG_OMEGA',
	'CONFIG_VMAX',
	'CONFIG_SPEED',
	'CONFIG_RSPEED',
	'CONFIG_ACCEL',
	'CONFIG_ALPHA',
];

static get CMD_SET_CONFIG() { return 'c'; }
static get CMD_GET_CONFIG() { return 'C'; }
static get CMD_MOVE_TO() { return 'N'; }
static get CMD_SEND_STATUS() { return 'P'; }
static get CMD_SET_SPEED() { return 'V'; }
static get CMD_SET_ROTATION_SPEED() { return 'r'; }
static get CMD_MOVE_FORWARD() { return 'r'; }
static get CMD_RELATIVE_ROTATE() { return 'T'; }
static get CMD_ABSOLUTE_ROTATE() { return 'A'; }
static get CMD_TURN_AND_GO() { return 'G'; }
static get CMD_CURVE() { return 'Q'; }
static get CMD_HARD_STOP() { return 'S'; }
static get CMD_SOFT_STOP() { return 's'; }
static get CMD_SMOOTH_STOP() { return 't'; }
static get CMD_RESET_DRIVER() { return 'R'; }
static get CMD_KILL_REGULATOR() { return 'H'; }
static get CMD_FORWARD() { return 'D'; }
static get CMD_SET_POSITION_AND_ORIENTATION() { return 'I'; }
static get CMD_BREAK() { return 'i'; }

