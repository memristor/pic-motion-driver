#include "config.h"
#include "hw/bootloader.h"
#include "math.h"
#include "regulator.h"
int8_t config_bytes[CONFIG_MAX_BYTES];
int16_t config_ints[CONFIG_MAX_INTS];
float config_floats[CONFIG_MAX_FLOATS];



#define MAX_EXPONENT_BITS 9

static ConfigCallback config_callbacks[CONFIG_MAX];
int16_t config_hash_map[CONFIG_MAX];

void config_init(void) {
	int i;
	for(i=0; i < CONFIG_MAX; i++) {
		config_callbacks[i] = 0;
	}
	config_load_hash();
}

static uint32_t load_uint32_bigendian(uint8_t* s) {
	return (((uint32_t)s[0]) << 24) | (((uint32_t)s[1]) << 16) | (((uint32_t)s[2]) << 8) | ((uint32_t)s[3]);
}

void config_load_from_stream(int length, uint8_t* stream) {
	// key uint32 sign exponent = 7 (max)
	while(length >= 7) {
		int key = *stream++;
		int32_t val = load_uint32_bigendian(stream);
		stream += 4;
		int sign = *stream++;
		int exponent = *stream++;
		val = sign ? -val : val;
		config_set_as_fixed_point(key, val, exponent);
		stream++;
		length -= 8;
	}
}


int config_process_packet(Packet* pkt) {
	switch(pkt->type) {
		case CMD_SET_CONFIG:
			config_load_from_stream(pkt->size, pkt->data);
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
		
		case CMD_SET_CONFIG_HASH: {
			int hash, exp, key;
			int32_t val;
			hash = get_word();
			val = get_long();
			exp = get_byte();
			config_set_as_fixed_point(config_get_key(hash), val, exp);
			break;
		}
			
		case CMD_GET_CONFIG_HASH: {
			uint32_t val;
			int exponent, sign, hash, key;
			hash = get_word();
			key = config_get_key(hash);
			config_get_as_fixed_point(key, (int32_t*)&val, &exponent, &sign);
			
			start_packet(CMD_GET_CONFIG);
				put_word(val >> 16);
				put_word(val);
				put_byte(sign);
				put_byte(exponent);
			end_packet();
			break;
		}
		
		case CMD_SAVE_CONFIG: {
			config_save_to_program_memory();
			break;
		}
		
		case CMD_LOAD_CONFIG: {
			config_load_defaults();
			break;
		}
		
		case CMD_L_RESP:
			L_resp_set();
			break;
			
		case CMD_RESET: {
			sw_reset();
			break;
		}
		
		
		default:
			return 0;
	}
	return 1;
}

int config_get_as_stream(int key, uint8_t* stream) {
	int32_t val;
	uint32_t uval;
	int exponent, sign;
	config_get_as_fixed_point(key, &val, &exponent, &sign);
	uval = absl(val);
	*stream++ = key;
	*stream++ = (uval >> 24) & 0xff;
	*stream++ = (uval >> 16) & 0xff;
	*stream++ = (uval >> 8) & 0xff;
	*stream++ = (uval >> 0) & 0xff;
	*stream++ = sign == -1;
	*stream++ = exponent;
}

void config_save_to_program_memory() {
	int i;
	uint8_t block[8];
	int8_t* ptr = eeprom_get_ptr();
	if(ptr) {
		for(i=0; i < CONFIG_MAX; i++) {
			config_get_as_stream(i, (uint8_t*)ptr);
			ptr += 8;
		}
		eeprom_save();
	}
}


void config_load_from_memory() {
	int8_t* ptr = eeprom_get_ptr();
	eeprom_load();
	config_load_from_stream(CONFIG_MAX*8, ptr);
}

void config_load(void) {
	int i;
	if (eeprom_initialized()) {
		config_load_from_memory();
	} else {
		config_load_defaults();
		config_save_to_program_memory();
	}
}

uint8_t config_get_key(int16_t hash) {
	int i;
	for(i=0; i < CONFIG_MAX; i++){
		if (config_hash_map[i] == hash) {
			return i;
		}
	}
	return -1;
}

void config_load_hash_map(int16_t* hash) {
	int i;
	for(i=0; i < CONFIG_MAX; i++){
		config_hash_map[i] = hash[i];
	}
}

void config_set_as_fixed_point(int key, int32_t value, int exponent) {
	float exp = exponent;
	float val = value / powf(10.0f, exp);
	if(key >= CONFIG_MAX) return;
	if(key < CONFIG_INT_OFFSET) {
		config_bytes[key-CONFIG_BYTE_OFFSET] = val;
	} else if(key < CONFIG_FLOAT_OFFSET) {
		config_ints[key-CONFIG_INT_OFFSET] = val;	
	} else { // float offset
		config_floats[key-CONFIG_FLOAT_OFFSET] = val;
	}
	
	ConfigCallback cb = config_callbacks[key];
	if(cb != 0) {
		cb();
	}
}

int config_get_as_fixed_point(int key, int32_t* value, int *exponent, int *sgn) {
	if(key >= CONFIG_MAX) return 0;
	
	float val;
	if(key < CONFIG_INT_OFFSET) {
		val = config_get_b(key);
	} else if(key < CONFIG_FLOAT_OFFSET) {
		val = config_get_i(key);
	} else { // float offset
		val = config_get_f(key);
	}
	
	int fract_numbers = MAX_EXPONENT_BITS - ( (int)log10(fabsf(maxf(1,val))) + 1 );	
	float exp = fract_numbers;
	*value = val * powf(10.0f, exp);
	*sgn = sign(val) == -1;
	*exponent = fract_numbers;
	return 1;
}



void config_on_change(int key, ConfigCallback callback) {
	config_callbacks[key] = callback;
}

void config_set_i(int key, int value) {
	int idx = key - CONFIG_INT_OFFSET;
	if(idx < 0 || idx >= CONFIG_MAX_INTS) return;
	config_ints[key-CONFIG_INT_OFFSET] = value;
	ConfigCallback cb = config_callbacks[key];
	if(cb != 0) {
		cb();
	}
}

void config_set_b(int key, int8_t value) {
	int idx = key - CONFIG_BYTE_OFFSET;
	if(idx < 0 || idx >= CONFIG_MAX_BYTES) return;
	config_bytes[idx] = value;
	ConfigCallback cb = config_callbacks[key];
	if(cb != 0) {
		cb();
	}
}

void config_set_f(int key, float value) {
	int idx = key - CONFIG_FLOAT_OFFSET;
	if(idx < 0 || idx >= CONFIG_MAX_FLOATS) return;
	config_floats[idx] = value;
	ConfigCallback cb = config_callbacks[key];
	if(cb != 0) {
		cb();
	}
}



