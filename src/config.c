#include "config.h"
#include <math.h>
int8_t config_bytes[CONFIG_MAX_BYTES];
int16_t config_ints[CONFIG_MAX_INTS];
float config_floats[CONFIG_MAX_FLOATS];

#define EXPONENT_BITS 4

static ConfigCallback config_callbacks[CONFIG_MAX];

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
	while(length >= 5) {
		int key = *stream++;
		uint32_t val = load_uint32_bigendian(stream);
		config_set_as_uint32(key, val);
		stream += 4;
		length -= 5;
	}
}

uint32_t config_get_as_uint32(int key) {
	if(key >= CONFIG_MAX) return 0;
	
	if(key < CONFIG_INT_OFFSET) {
		return config_get_b(key);
	} else if(key < CONFIG_FLOAT_OFFSET) {
		return config_get_i(key);
	} else { // float offset
		uint32_t exp = EXPONENT_BITS;
		uint32_t val = config_get_f(key) * powf(10.0f, exp);
		return (val << exp) | (exp & ((1u<<EXPONENT_BITS)-1));
	}
}

void config_on_change(int key, ConfigCallback callback) {
	config_callbacks[key] = callback;
}

inline int config_get_b(int key) {
	return config_bytes[key-CONFIG_BYTE_OFFSET];
}

inline int config_get_i(int key) {
	return config_ints[key-CONFIG_INT_OFFSET];
}

inline float config_get_f(int key) {
	return config_floats[key-CONFIG_FLOAT_OFFSET];
}

void config_set_i(int key, int value) {
	config_ints[key-CONFIG_INT_OFFSET] = value;
	ConfigCallback cb = config_callbacks[key];
	if(cb != 0) {
		cb();
	}
}

void config_set_b(int key, char value) {
	config_bytes[key-CONFIG_BYTE_OFFSET] = value;
	ConfigCallback cb = config_callbacks[key];
	if(cb != 0) {
		cb();
	}
}

void config_set_f(int key, float value) {
	config_floats[key-CONFIG_FLOAT_OFFSET] = value;
	ConfigCallback cb = config_callbacks[key];
	if(cb != 0) {
		cb();
	}
}


void config_set_as_uint32(int key, uint32_t value) {
	if(key >= CONFIG_MAX) return;
	if(key < CONFIG_INT_OFFSET) {
		config_bytes[key-CONFIG_BYTE_OFFSET] = value;
	} else if(key < CONFIG_FLOAT_OFFSET) {
		config_ints[key-CONFIG_INT_OFFSET] = value;	
	} else { // float offset
		float exp = (value & ((1u<<EXPONENT_BITS)-1));
		config_floats[key-CONFIG_FLOAT_OFFSET] = (value >> EXPONENT_BITS);
		config_floats[key-CONFIG_FLOAT_OFFSET] /= powf(10.0f, exp);
	}
	
	ConfigCallback cb = config_callbacks[key];
	if(cb != 0) {
		cb();
	}
}
