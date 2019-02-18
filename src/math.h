#ifndef UTIL_MATH_H
#define UTIL_MATH_H

#include <stdint.h>
#define SINUS_MAX 8192u
#define SINUS_AMPLITUDE 0x7fff
#include <math.h>

#undef abs
#undef min
#undef max

int deg_angle_range_normalize(int angle);
float rad_angle_range_normalize(float angle);
long angle_range_normalize_long(long angle, long max_angle_amplitude);
float angle_range_normalize_float(float angle, float angle_period);
uint32_t uint32_log10(uint32_t v);
void sin_cos(long theta, long *sint, long *cost);
int deg_angle_diff(int a, int b);
float min3f(float a, float b, float c);
float max3f(float a, float b, float c);

#define in_range(val, a, b) ((val) >= (a) && (val) <= (b))
int in_range_long(long val, long a, long b);
float dval(char dir, float val);
float dfval(float dir, float val);

#define proc_def(def, func) 	\
	def(int, func); 			\
	def(long, func ## l); 		\
	def(long long, func ## ll); \
	def(float, func ## f); 		\
	def(double, func ## d);		\
	
#define def_abs_decl(type, f) type f(type a);
#define def_clip_decl(type, f) \
	type f(type a, type b, type v); \
	type f ## 2(type amp, type v); \
	type f ## _margin(type center, type margin, type v);
	
#define def_sign_decl(type, f) type f(type x);
#define def_min_decl(type, f) type f(type a, type b);
#define def_max_decl(type, f) type f(type a, type b);

proc_def(def_abs_decl, abs);
proc_def(def_clip_decl, clip);
proc_def(def_sign_decl, sign);
proc_def(def_min_decl, min);
proc_def(def_max_decl, max);


#endif
