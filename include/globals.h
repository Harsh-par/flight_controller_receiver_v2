#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

extern int64_t time_us_last_packet;

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef int64_t i64;
typedef int32_t i32;
typedef int16_t i16;
typedef int8_t  i8;

typedef double f64;
typedef float  f32;

float map(float x, float in_min, float in_max, float out_min, float out_max);

#define MAPF(in, in_min, in_max, out_min, out_max) \
((float)(in) - (float)(in_min)) * ((float)(out_max) - (float)(out_min)) / ((float)(in_max) - (float)(in_min)) + (float)(out_min)

#define MAPI(in, in_min, in_max, out_min, out_max) \
((int)(in) - (int)(in_min)) * ((int)(out_max) - (int)(out_min)) / ((int)(in_max) - (int)(in_min)) + (int)(out_min)

#define CONSTRAIN(in, in_min, in_max) \
(((in) < (in_min)) ? (in_min) : (((in) > (in_max)) ? (in_max) : (in)))

#define DEG_TO_RAD(in) \
((in) * 0.01745329252f)

#define RAD_TO_DEG(in) \
((in) * 57.295779513f)

#define ABS(in) \
((in) < 0 ? -(in) : (in))

#endif