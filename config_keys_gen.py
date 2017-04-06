#!/bin/env python3

keys_float = {

	'wheel_distance': 330.7,
	'wheel_R1': 92.936704,
	'wheel_R2': 92.936704,
	
	'PID_d_p': 0,
	'PID_d_d': 0,
	'PID_r_p': 0,
	'PID_r_d': 0,
	
	'speed': 0,
	'rspeed': 0,
	'accel': 0,
	'alpha': 0
}

keys_int = {
	'stuck_distance_jump': 0,
	'stuck_rotation_jump': 0,
	
	'stuck_distance_max_fail_count': 0,
	'stuck_rotation_max_fail_count': 0,	
}


keys_byte = {
	'distance_regulator': 0,
	'rotation_regulator': 0,
	'stuck': 0,
	'debug': 0,
	'status_change_report': 0	
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
	
	for i in keys_byte:
		s += '#define c_' + i.lower() + ' ' + 'config_bytes[CONF_' + i.upper() + '-CONFIG_BYTE_OFFSET]' + '\n'
	s += '\n'
	for i in keys_int:
		s += '#define c_' + i.lower() + ' ' + 'config_ints[CONF_' + i.upper() + '-CONFIG_INT_OFFSET]' + '\n'
	s += '\n'
	for i in keys_float:
		s += '#define c_' + i.lower() + ' ' + 'config_floats[CONF_' + i.upper() + '-CONFIG_FLOAT_OFFSET]' + '\n'
	
	s += '\n\n'
	
	s += 'void config_set_b(int key, char value);\n' + \
		 'void config_set_i(int key, int value);\n' + \
		 'void config_set_f(int key, float value);\n\n'
	
	s += 'static inline void config_load_defaults(void) {\n'
	for kv in keys_byte.items():
		s += '\tconfig_set_b(CONF_' + kv[0].upper() + ', ' + str(int(kv[1]) & 0xff) + ');\n'
		
	for kv in keys_int.items():
		s += '\tconfig_set_i(CONF_' + kv[0].upper() + ', ' + str(int(kv[1]) & 0xffff) + ');\n'
		
	for kv in keys_float.items():
		s += '\tconfig_set_f(CONF_' + kv[0].upper() + ', ' + str(float(kv[1])) + 'f);\n'
	s += '}\n'

	s += '\n\n#endif\n'
	print(s)

def gen_js_code():
	s = '// THIS FILE IS GENERATED WITH PYTHON SCRIPT "config_keys_gen.py"\n'
	cnt = 0
	for i in keys_byte:
		s += 'static get CONFIG_' + i.upper() + '() { return ' + str(cnt) + '; }' + '\n'
		cnt += 1
	s += '\n'
	for i in keys_int:
		s += 'static get CONFIG_' + i.upper() + '() { return ' + str(cnt) + '; }' + '\n'
		cnt += 1
	s += '\n'
	for i in keys_float:
		s += 'static get CONFIG_' + i.upper() + '() { return ' + str(cnt) + '; }' + '\n'
		cnt += 1
	print(s)

import sys
if sys.argv[-1] == 'mcu':
	gen_mcu_code()
elif sys.argv[-1] == 'js':
	gen_js_code()
