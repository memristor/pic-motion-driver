#include "config.h"
#include <math.h>
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
}

static uint32_t load_uint32_bigendian(uint8_t* s) {
	return (((uint32_t)s[0]) << 24) | (((uint32_t)s[1]) << 16) | (((uint32_t)s[2]) << 8) | ((uint32_t)s[3]);
}

void config_load(int length, uint8_t* stream) {
	// key uint32 sign exponent = 7 (max)
	while(length >= 7) {
		int key = *stream++;
		int32_t val = load_uint32_bigendian(stream);
		stream += 4;
		int sign = *stream++;
		int exponent = *stream++;
		config_set_as_fixed_point(key, sign ? -val : val, exponent);
		length -= 7;
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
	for(i=0; i<CONFIG_MAX; i++){
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

int config_get_as_fixed_point(int key, int32_t* value, int *exponent, int *sign) {
	if(key >= CONFIG_MAX) return 0;
	
	float val;
	if(key < CONFIG_INT_OFFSET) {
		val = config_get_b(key);
	} else if(key < CONFIG_FLOAT_OFFSET) {
		val = config_get_i(key);
	} else { // float offset
		val = config_get_f(key);
	}
	
	int fract_numbers = MAX_EXPONENT_BITS - ( (int)log10(fabsf(val)) + 1 );	
	float exp = fract_numbers;
	*value = val * powf(10.0f, exp);
	*exponent = fract_numbers;
	return 1;
}

void config_save_to_program_memory() {
	
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



