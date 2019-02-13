#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

void config_load_hash_map(int16_t* hash);
void config_set_b(int key, int8_t value);
void config_set_i(int key, int value);
void config_set_f(int key, float value);

// -----------[ config keys ]-------------------
#include "config_keys.h"
// ---------------------------------------------


#define CONFIG_MAX (CONFIG_MAX_BYTES+CONFIG_MAX_FLOATS+CONFIG_MAX_INTS)

// must be B < I < F
#define CONFIG_BYTE_OFFSET 0
#define CONFIG_INT_OFFSET (CONFIG_MAX_BYTES)
#define CONFIG_FLOAT_OFFSET (CONFIG_INT_OFFSET+CONFIG_MAX_INTS)

// diagnostics
extern int8_t config_changed[CONFIG_MAX];
extern int8_t config_bytes[CONFIG_MAX_BYTES];
extern int16_t config_ints[CONFIG_MAX_INTS];
extern float config_floats[CONFIG_MAX_FLOATS];

void config_save_to_program_memory();

void config_load(int length, uint8_t* stream); // reads all keys from stream

void config_set_as_fixed_point(int key, int32_t value, int exponent);
int config_get_as_fixed_point(int key, int32_t* value, int *exponent, int *sign);

static inline int config_get_b(int key) {
	return config_bytes[key-CONFIG_BYTE_OFFSET];
}

static inline int config_get_i(int key) {
	return config_ints[key-CONFIG_INT_OFFSET];
}

static inline float config_get_f(int key) {
	return config_floats[key-CONFIG_FLOAT_OFFSET];
}

void config_init(void);
uint8_t config_get_key(int16_t hash);


typedef void (*ConfigCallback)(void);
void config_on_change(int key, ConfigCallback callback);

#ifndef SIM

	#ifdef BOARD_V2
		#define FCY 29491200ULL
		#include <xc.h>
		#define interrupt_lock
		#define interrupt_unlock
		#define INTERRUPT
		#define DMA_SPACE
		#define __delay_ms(t)
		#define REGULATOR_INTERRUPT void INTERRUPT _T1Interrupt(void)
	#else
		#define FCY 29491200ULL
		#include <p33FJ128MC802.h>
		#include <libpic30.h>
		#define interrupt_lock SRbits.IPL = 7;
		#define interrupt_unlock SRbits.IPL = 0;
		// #define INTERRUPT __attribute__((interrupt(auto_psv)))
		#define INTERRUPT __attribute__((interrupt(auto_psv)))
		#define DMA_SPACE __attribute__((space(dma)))
		#define REGULATOR_INTERRUPT void INTERRUPT _T1Interrupt(void)
	#endif


#else
	#define interrupt_lock
	#define interrupt_unlock
	#define INTERRUPT
	#define __delay_ms(t) usleep(t*1000)
#endif

#endif
