#include "config.h"

int8_t config_bytes[CONFIG_MAX_BYTES];
int16_t config_ints[CONFIG_MAX_INTS];
float config_floats[CONFIG_MAX_FLOATS];


static ConfigCallback config_callbacks[CONFIG_MAX];

void config_init(void) {
	int i;
	for(i=0; i < CONFIG_MAX; i++) {
		config_callbacks[i] = 0;
	}
	config_load_defaults();
}

int config_parse(int length, uint8_t* stream) {
	if(length <= 0) return 0;
	uint8_t key = *stream++;
	if(key >= CONFIG_MAX) return 0;
	
	if(key < CONFIG_BYTE_OFFSET) {
		config_set_b(key, *stream);
		return 2;
	} else if(key < CONFIG_INT_OFFSET) {
		config_set_i(key, *((int16_t*)stream));
		return 3;
	} else { // float offset
		config_set_f_with_uint32(key, *((uint32_t*)stream));
		return 5;
	}
}

void config_load(int length, uint8_t* stream) {
	int parsed = 0;
	while(length > 0) {
		parsed = config_parse(length, stream);
		stream += parsed;
		length -= parsed;
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

#define EXPONENT_BITS 3
void config_set_f_with_uint32(int key, uint32_t value) {
	float exp = (value & ((1<<EXPONENT_BITS)-1));
	config_floats[key] = (value >> EXPONENT_BITS);
	config_floats[key] /= 10.0f * exp; 
}
