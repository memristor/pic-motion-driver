#!/bin/env python3


# -------------------------------------
import sys
sys.dont_write_bytecode = True

import importlib, textwrap
if len(sys.argv) >= 3:
	config_for_robot = sys.argv[2]
else:
	config_for_robot = 'big'

import robots.default as defs

try:
	imp_conf = importlib.import_module('robots.'+config_for_robot)
except:
	sys.stderr.write('no robot called ' + config_for_robot + '\n')
	exit(-1)

from general import *

defs.conf_floats.update(imp_conf.conf_floats)
defs.conf_integers.update(imp_conf.conf_integers)
defs.conf_bytes.update(imp_conf.conf_bytes)


conf = {
	'float': defs.conf_floats,
	'int': defs.conf_integers,
	'byte': defs.conf_bytes
}

try:
	import subprocess
	verstr = subprocess.check_output(['git', 'rev-parse', 'HEAD'])
	verstr = verstr[:4]
	conf['int']['version'] = int(verstr, 16)
except:
	pass

def simple_hash(v):
	h=5381
	for i in v:
		h = h*33 + ord(i)
	return int(h) & 0xffff

def gen_mcu_code():
	s = '// THIS FILE IS GENERATED WITH PYTHON SCRIPT "gen_config.py" and must only be included within "config.h"\n'
	s += '#ifdef CONFIG_H\n\n'
	s += '#define CONFIG_MAX_BYTES ' + str(len(conf['byte'])) + '\n'
	s += '#define CONFIG_MAX_INTS ' + str(len(conf['int'])) + '\n'
	s += '#define CONFIG_MAX_FLOATS ' + str(len(conf['float'])) + '\n'
	
	s += '\n\n'
	
	cnt = 0
	s += 'enum ConfEnum { \n'
	for i in conf['byte']:
		s += '\tCONF_' + i.upper() + ' = ' + str(cnt) + ',\n'
		cnt += 1
	s += '\n'
	for i in conf['int']:
		s += '\tCONF_' + i.upper() + ' = ' + str(cnt) + ',\n'
		cnt += 1
	s += '\n'
	for i in conf['float']:
		s += '\tCONF_' + i.upper() + ' = ' + str(cnt) + ',\n'
		cnt += 1
		
	s += '};\n'
	
	s += '\n\n'
	
	prefix = 'CONF_'
	for i in conf['byte']:
		s += '#define c_' + i.lower() + ' ' + 'config_bytes[' + prefix + i.upper() + '-CONFIG_BYTE_OFFSET]' + '\n'
	s += '\n'
	for i in conf['int']:
		s += '#define c_' + i.lower() + ' ' + 'config_ints[' + prefix + i.upper() + '-CONFIG_INT_OFFSET]' + '\n'
	s += '\n'
	for i in conf['float']:
		s += '#define c_' + i.lower() + ' ' + 'config_floats[' + prefix + i.upper() + '-CONFIG_FLOAT_OFFSET]' + '\n'
	
	s += '\n\n'
	
	s += 'static inline void config_load_defaults(void) {\n'
	from collections import Counter, OrderedDict
	hash_keys = OrderedDict()
	hash_map=[]
	hash_names=[]
	
	def hash_it(v):
		v = v.lower()
		h = simple_hash(v)
		hash_keys[h] = v
		hash_names.append(v)
		hash_map.append(h)
	
	for kv in conf['byte'].items():
		s += '\tconfig_set_b(' + prefix + kv[0].upper() + ', ' + str(int(kv[1]) & 0xff) + ');\n'
		hash_it(kv[0])
		
	for kv in conf['int'].items():
		s += '\tconfig_set_i(' + prefix + kv[0].upper() + ', ' + str(int(kv[1]) & 0xffff) + ');\n'
		hash_it(kv[0])
		
	for kv in conf['float'].items():
		s += '\tconfig_set_f(' + prefix + kv[0].upper() + ', ' + str(float(kv[1])) + 'f);\n'
		hash_it(kv[0])
	
	s += '}\n'
	s += 'static inline void config_load_hash(void) {\n'
	hashmap = str(hash_map).replace('[','{').replace(']','}') + ';\n';
	s += '\tint16_t hash_map[] = ' + '\n\t\t'.join(textwrap.wrap(hashmap, 80)) + '\n'
	s += '\tconfig_load_hash_map(hash_map);\n'
	s += '}\n';
		
	if len(hash_map) != len(set(hash_map)):
		cnt = Counter(hash_map)
		dupl=[hash_names[i]+':'+str(j) for i,j in enumerate(hash_map) if cnt[j] >= 2]
		s += '#error hash map has duplicates('+str(len(dupl))+'): '
		s += ', '.join(dupl) + '\n'
				
	prefix = 'CMD_'
	for kv in commands.items():
		s += '#define ' + prefix + kv[0].upper() + ' \'' + kv[1][0] + '\'\n'
		
	prefix = 'MSG_'
	for kv in messages.items():
		s += '#define ' + prefix + kv[0].upper() + ' \'' + kv[1][0] + '\'\n'

	s += '\n\n#endif\n'
	print(s)

def gen_js_code():
	prefix = 'CONFIG_'
	# s = '// THIS FILE IS GENERATED WITH PYTHON SCRIPT "config_keys_gen.py"\n'
	s = ''
	cnt = 0
	for i in conf['byte']:
		s += 'static get ' + prefix + i.upper() + '() { return ' + str(cnt) + '; }' + '\n'
		cnt += 1
	s += '\n'
	for i in conf['int']:
		s += 'static get ' + prefix + i.upper() + '() { return ' + str(cnt) + '; }' + '\n'
		cnt += 1
	s += '\n'
	for i in conf['float']:
		s += 'static get ' + prefix + i.upper() + '() { return ' + str(cnt) + '; }' + '\n'
		cnt += 1
	
	s += '\n\n'
	
	prefix = 'CONFIG_'
	cnt = 0
	s += 'static var config_names_enum = [\n'
	for i in conf['byte']:
		s += '\t\'' + prefix + i.upper() + '\','
		if cnt % 10 == 0:
			s += '\n'
	for i in conf['int']:
		s += '\t\'' + prefix + i.upper() + '\','
		if cnt % 10 == 0:
			s += '\n'
	for i in conf['float']:
		s += '\t\'' + prefix + i.upper() + '\','
		if cnt % 10 == 0:
			s += '\n'
	s += '];\n\n'
	
	
	prefix = 'CMD_'
	for kv in commands.items():
		s += 'static get ' + prefix + kv[0].upper() + '() { return \'' + kv[1][0] + '\'; }' + '\n'
	
	prefix = 'MSG_'
	for kv in messages.items():
		s += 'static get ' + prefix + kv[0].upper() + '() { return \'' + kv[1][0] + '\'; }' + '\n'
	
	print(s)


def gen_py_code():
	cnt = 0
	s = 'config_bytes = [\n'
	for i in conf['byte']:
		s += '\t\'' + i.lower() + '\',\n'
		cnt += 1
	s += '\n'
	s += ']\n'
	s += 'config_ints = [\n'
	for i in conf['int']:
		s += '\t\'' + i.lower() + '\',\n'
		cnt += 1
	s += '\n'
	s += ']\n'
	
	s += 'config_floats = [\n'
	for i in conf['float']:
		s += '\t\'' + i.lower() + '\',\n'
		cnt += 1
	s += ']\n'
	
	s += 'commands = {\n'
	for i in commands:
		s += '\t\'' + i + '\': \'' + commands[i] + '\',\n'
	s += '}\n'
	
	s += 'messages = {\n'
	for i in messages:
		s += '\t\'' + i + '\': \'' + messages[i] + '\',\n'
	s += '}\n'
	print(s)
	
import sys
if sys.argv[1] == 'mcu':
	gen_mcu_code()
elif sys.argv[1] == 'js':
	gen_js_code()
elif sys.argv[1] == 'py':
	gen_py_code()
