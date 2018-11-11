#ifndef UTIL_MATH_H
#define UTIL_MATH_H
#include <stdint.h>
#include <math.h>

#define SINUS_MAX 8192u
#define SINUS_AMPLITUDE 0x7fff

int deg_angle_range_fix(int angle);
float rad_angle_range_fix(float angle);
long inc_angle_range_fix(long angle);

#define in_range(val, a,b) ((val) >= (a) && (val) <= (b))

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))

static inline long minl(long a, long b) {
	return a < b ? a : b;
}
static inline long maxl(long a, long b) {
	return a > b ? a : b;
}

static inline float dval(char dir, float val) {
	if(dir >= 0) {
		return val;
	} else {
		return -val;
	}
}

static inline float dfval(float dir, float val) {
	if(dir >= 0) {
		return val;
	} else {
		return -val;
	}
}

static inline long absl(long a)
{
	return a >= 0 ? a : -a;
}
static inline float absf(float a)
{
	return a >= 0 ? a : -a;
}

static inline long clipl(long a, long b, long value) {
	if(value <= a)
		value = a;
	else if(value > b)
		value = b;
	return value;
}

#define clipl2(amp, value) clipl(-amp,amp, value)

static inline long long clipll(long long a, long long b, long long value) {
	if(value <= a)
		value = a;
	else if(value > b)
		value = b;
	return value;
}

static inline int clip(int a, int b, int value) {
	if(value <= a)
		value = a;
	else if(value > b)
		value = b;
	return value;
}

static inline int in_range_long(long val, long a, long b) {
	return in_range(val, a,b);
}

static inline int sign(int x) {
	return x >= 0L ? 1 : -1;
}

static inline long signl(long x) {
	return x >= 0L ? 1 : -1;
}

static inline char signf(float x) {
	return x >= 0.0f ? 1 : -1;
}

static inline float minf(float a, float b) {
	return a < b ? a : b;
}

static inline float maxf(float a, float b) {
	return a > b ? a : b;
}

uint32_t uint32_log10(uint32_t v);

void sin_cos(long theta, long *sint, long *cost);

int deg_angle_diff(int a, int b);

float min3f(float a, float b, float c);
float max3f(float a, float b, float c);

#endif
