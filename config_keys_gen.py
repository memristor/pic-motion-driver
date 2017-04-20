#!/bin/env python3

keys_float = {
	'wheel_distance': 330.7,
	'wheel_R1': 92.52,
	'wheel_R2': 92.55,
	
	'PID_d_p': 5.5,
	'PID_d_d': 200,
	'PID_r_p': 3,
	'PID_r_d': 90,
	
	'vmax': 0x32,
	'omega': 0x32,
	
	'accel': 0x32,
	'alpha': 0x32,
	'slowdown': 1.0,
	'angle_speedup': 20
}

keys_int = {
	'stuck_distance_jump': 400, # in milimeters
	'stuck_rotation_jump': 180, # in degrees
	
	'stuck_distance_max_fail_count': 200,
	'stuck_rotation_max_fail_count': 200,
	
	# 3200 is max motor speed
	'motor_speed_limit': 3200,
	'motor_rate_of_change': 3200,
	'send_status_interval': 0,
}


keys_byte = {
	'distance_regulator': 1,
	'rotation_regulator': 1,
	'enable_stuck': 1,
	'stuck': 0, # is it stuck right now
	'debug': 0,
	'status_change_report': 1,
	'keep_count': 100,
}

commands = {
	'set_config': 'c',
	'get_config': 'C',
	'move_to': 'N',
	'send_status': 'P',
	'set_speed': 'V',
	'set_rotation_speed': 'r',
	'move_forward': 'r',
	'relative_rotate': 'T',
	'absolute_rotate': 'A',
	'turn_and_go': 'G',
	'curve': 'Q',
	'hard_stop': 'S',
	'soft_stop': 's',
	'smooth_stop': 't',
	'reset_driver': 'R',
	'kill_regulator': 'H',
	'forward': 'D',
	'set_position_and_orientation': 'I',
	'break': 'i',
}

# -------------------------------------

def gen_mcu_code():
	s = '// THIS FILE IS GENERATED WITH PYTHON SCRIPT "config_keys_gen.py" and must only be included within "config.h"\n'
	s += '#ifdef CONFIG_H\n\n'
	s += '#define CONFIG_MAX_BYTES ' + str(len(keys_byte)) + '\n'
	s += '#define CONFIG_MAX_INTS ' + str(len(keys_int)) + '\n'
	s += '#define CONFIG_MAX_FLOATS ' + str(len(keys_float)) + '\n'
	
	s += '\n\n'
	
	cnt = 0
	s += 'enum ConfEnum { \n'
	for i in keys_byte:
		s += '\tCONF_' + i.upper() + ' = ' + str(cnt) + ',\n'
		cnt += 1
	s += '\n'
	for i in keys_int:
		s += '\tCONF_' + i.upper() + ' = ' + str(cnt) + ',\n'
		cnt += 1
	s += '\n'
	for i in keys_float:
		s += '\tCONF_' + i.upper() + ' = ' + str(cnt) + ',\n'
		cnt += 1
		
	s += '};\n'
	
	s += '\n\n'
	
	prefix = 'CONF_'
	for i in keys_byte:
		s += '#define c_' + i.lower() + ' ' + 'config_bytes[' + prefix + i.upper() + '-CONFIG_BYTE_OFFSET]' + '\n'
	s += '\n'
	for i in keys_int:
		s += '#define c_' + i.lower() + ' ' + 'config_ints[' + prefix + i.upper() + '-CONFIG_INT_OFFSET]' + '\n'
	s += '\n'
	for i in keys_float:
		s += '#define c_' + i.lower() + ' ' + 'config_floats[' + prefix + i.upper() + '-CONFIG_FLOAT_OFFSET]' + '\n'
	
	s += '\n\n'
	
	s += 'void config_set_b(int key, int8_t value);\n' + \
		 'void config_set_i(int key, int value);\n' + \
		 'void config_set_f(int key, float value);\n\n'
	
	s += 'static inline void config_load_defaults(void) {\n'
	for kv in keys_byte.items():
		s += '\tconfig_set_b(' + prefix + kv[0].upper() + ', ' + str(int(kv[1]) & 0xff) + ');\n'
		
	for kv in keys_int.items():
		s += '\tconfig_set_i(' + prefix + kv[0].upper() + ', ' + str(int(kv[1]) & 0xffff) + ');\n'
		
	for kv in keys_float.items():
		s += '\tconfig_set_f(' + prefix + kv[0].upper() + ', ' + str(float(kv[1])) + 'f);\n'
	s += '}\n'
	
	prefix = 'CMD_'
	for kv in commands.items():
		s += '#define ' + prefix + kv[0].upper() + ' \'' + kv[1][0] + '\'\n'

	s += '\n\n#endif\n'
	print(s)

def gen_js_code():
	prefix = 'CONFIG_'
	# s = '// THIS FILE IS GENERATED WITH PYTHON SCRIPT "config_keys_gen.py"\n'
	s = ''
	cnt = 0
	for i in keys_byte:
		s += 'static get ' + prefix + i.upper() + '() { return ' + str(cnt) + '; }' + '\n'
		cnt += 1
	s += '\n'
	for i in keys_int:
		s += 'static get ' + prefix + i.upper() + '() { return ' + str(cnt) + '; }' + '\n'
		cnt += 1
	s += '\n'
	for i in keys_float:
		s += 'static get ' + prefix + i.upper() + '() { return ' + str(cnt) + '; }' + '\n'
		cnt += 1
	
	s += '\n\n'
	
	prefix = 'CONFIG_'
	cnt = 0
	s += 'static var config_names_enum = [\n'
	for i in keys_byte:
		s += '\t\'' + prefix + i.upper() + '\','
		if cnt % 10 == 0:
			s += '\n'
	for i in keys_int:
		s += '\t\'' + prefix + i.upper() + '\','
		if cnt % 10 == 0:
			s += '\n'
	for i in keys_float:
		s += '\t\'' + prefix + i.upper() + '\','
		if cnt % 10 == 0:
			s += '\n'
	s += '];\n\n'
	
	
	prefix = 'CMD_'
	for kv in commands.items():
		s += 'static get ' + prefix + kv[0].upper() + '() { return \'' + kv[1][0] + '\'; }' + '\n'
	
	print(s)

import sys
if sys.argv[-1] == 'mcu':
	gen_mcu_code()
elif sys.argv[-1] == 'js':
	gen_js_code()
