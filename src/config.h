#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// -----------[ config keys ]-------------------
#include "config_keys.h"
// ---------------------------------------------


#define CONFIG_MAX (CONFIG_MAX_BYTES+CONFIG_MAX_FLOATS+CONFIG_MAX_INTS)

// must be B < I < F
#define CONFIG_BYTE_OFFSET 0
#define CONFIG_INT_OFFSET (CONFIG_MAX_BYTES)
#define CONFIG_FLOAT_OFFSET (CONFIG_INT_OFFSET+CONFIG_MAX_INTS)

extern int8_t config_bytes[CONFIG_MAX_BYTES];
extern int16_t config_ints[CONFIG_MAX_INTS];
extern float config_floats[CONFIG_MAX_FLOATS];

void config_load(int length, uint8_t* stream); // reads all keys from stream
uint32_t config_get_as_uint32(int key);

extern inline int config_get_b(int key);
extern inline int config_get_i(int key);
extern inline float config_get_f(int key);

void config_init(void);

void config_set_b(int key, char value);
void config_set_i(int key, int value);
void config_set_f(int key, float value);
void config_set_as_uint32(int key, uint32_t value);

typedef void (*ConfigCallback)(void);
void config_on_change(int key, ConfigCallback callback);



#endif
