#pragma once

#include <math/types.h>

typedef struct {
  f32 x;
} vec1_t;

typedef struct {
  f32 x;
  f32 y;
} vec2_t;

typedef struct {
  f32 x;
  f32 y;
  f32 z;
} vec3_t;

typedef struct {
  f32 x;
  f32 y;
  f32 z;
  f32 w;
} vec4_t;

/* Constructors */

vec1_t vec1(f32 x);
vec2_t vec2(f32 x, f32 y);
vec3_t vec3(f32 x, f32 y, f32 z);
vec4_t vec4(f32 x, f32 y, f32 z, f32 w);

/* Arithmetic */

vec1_t vec1_add(vec1_t a, vec1_t b);
vec2_t vec2_add(vec2_t a, vec2_t b);
vec3_t vec3_add(vec3_t a, vec3_t b);
vec4_t vec4_add(vec4_t a, vec4_t b);

vec1_t vec1_sub(vec1_t a, vec1_t b);
vec2_t vec2_sub(vec2_t a, vec2_t b);
vec3_t vec3_sub(vec3_t a, vec3_t b);
vec4_t vec4_sub(vec4_t a, vec4_t b);

vec1_t vec1_mul(vec1_t v, f32 scalar);
vec2_t vec2_mul(vec2_t v, f32 scalar);
vec3_t vec3_mul(vec3_t v, f32 scalar);
vec4_t vec4_mul(vec4_t v, f32 scalar);

vec1_t vec1_div(vec1_t v, f32 scalar);
vec2_t vec2_div(vec2_t v, f32 scalar);
vec3_t vec3_div(vec3_t v, f32 scalar);
vec4_t vec4_div(vec4_t v, f32 scalar);

/* Geometry */

f32 vec1_dot(vec1_t a, vec1_t b);
f32 vec2_dot(vec2_t a, vec2_t b);
f32 vec3_dot(vec3_t a, vec3_t b);
f32 vec4_dot(vec4_t a, vec4_t b);

f32 vec1_magnitude(vec1_t v);
f32 vec2_magnitude(vec2_t v);
f32 vec3_magnitude(vec3_t v);
f32 vec4_magnitude(vec4_t v);

vec1_t vec1_normalize(vec1_t v);
vec2_t vec2_normalize(vec2_t v);
vec3_t vec3_normalize(vec3_t v);
vec4_t vec4_normalize(vec4_t v);

vec3_t vec3_cross(vec3_t a, vec3_t b);
