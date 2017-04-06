#ifndef MATH_H
#define MATH_H
#include <stdint.h>

#define SINUS_MAX 8192u
#define SINUS_AMPLITUDE 0x7fff

extern inline long absl(long a);
extern inline float absf(float a);

inline long long clipll(long long a, long long b, long long value);

int deg_angle_range_fix(int angle);
float rad_angle_range_fix(float angle);
long inc_angle_range_fix(long angle);

inline int clip(int a, int b, int value);

extern inline int sign(int x);
extern inline long signl(long x);
extern inline char signf(float x);
extern inline float dfval(float dir, float val);
extern inline float dval(char dir, float val);

extern inline float minf(float a, float b);
extern inline float maxf(float a, float b);
extern inline long minl(long a, long b);
extern inline long maxl(long a, long b);

uint32_t uint32_log10(uint32_t v);

void sin_cos(long theta, long *sint, long *cost);

int deg_angle_diff(int a, int b);

float min3f(float a, float b, float c);
float max3f(float a, float b, float c);

#endif
