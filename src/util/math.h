#ifndef UTIL_MATH_H
#define UTIL_MATH_H
#include <stdint.h>
#include <math.h>

#define SINUS_MAX 8192u
#define SINUS_AMPLITUDE 0x7fff

int deg_angle_range_normalize(int angle);
float rad_angle_range_normalize(float angle);
long angle_range_normalize_long(long angle, long max_angle_amplitude);
float angle_range_normalize_float(float angle, float angle_period);

#define in_range(val, a,b) ((val) >= (a) && (val) <= (b))
static inline int in_range_long(long val, long a, long b) {
	return in_range(val, a,b);
}

static inline float dval(char dir, float val) {
	return dir >= 0 ? val : -val;
}

static inline float dfval(float dir, float val) {
	return dir >= 0 ? val : -val;
}

#define proc_def(def, func) \
	def(int, func) \
	def(long, func ## l) \
	def(long long, func ## ll) \
	def(float, func ## f) \
	def(double, func ## d)

#define def_abs(type, f) \
	static inline type f(type a) { \
		return a >= 0 ? a : -a; \
	}

proc_def(def_abs, abs)

#define def_clip(type, f) \
	static inline type f(type a, type b, type v) { \
		if(v < a) v = a; \
		else if(v > b) v = b; \
		return v; \
	} \
	static inline type f ## 2(type amp, type v) { return f(-amp,amp,v); } \
	static inline type f ## _margin(type center, type margin, type v) { return f(center-margin,center+margin,v); }
	
proc_def(def_clip, clip)


#define def_sign(type, f) \
	static inline type f(type x) { \
		return x >= 0 ? 1 : -1; \
	}

proc_def(def_sign, sign)

#define def_min(type, f) \
	static inline type f(type a, type b) { \
		return a < b ? a : b; \
	}

#define def_max(type, f) \
	static inline type f(type a, type b) { \
		return a > b ? a : b; \
	}

proc_def(def_min, min)
proc_def(def_max, max)

uint32_t uint32_log10(uint32_t v);

void sin_cos(long theta, long *sint, long *cost);

int deg_angle_diff(int a, int b);

float min3f(float a, float b, float c);
float max3f(float a, float b, float c);

#endif
