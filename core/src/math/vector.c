#include <math/vector.h>

#include <math.h>

/*
 * Constructors
 */

vec1_t vec1(f32 x) { return (vec1_t){.x = x}; }

vec2_t vec2(f32 x, f32 y) { return (vec2_t){.x = x, .y = y}; }

vec3_t vec3(f32 x, f32 y, f32 z) { return (vec3_t){.x = x, .y = y, .z = z}; }

vec4_t vec4(f32 x, f32 y, f32 z, f32 w) {
  return (vec4_t){.x = x, .y = y, .z = z, .w = w};
}

/*
 * Addition
 */

vec1_t vec1_add(vec1_t a, vec1_t b) { return vec1(a.x + b.x); }

vec2_t vec2_add(vec2_t a, vec2_t b) { return vec2(a.x + b.x, a.y + b.y); }

vec3_t vec3_add(vec3_t a, vec3_t b) {
  return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

vec4_t vec4_add(vec4_t a, vec4_t b) {
  return vec4(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w);
}

/*
 * Subtraction
 */

vec1_t vec1_sub(vec1_t a, vec1_t b) { return vec1(a.x - b.x); }

vec2_t vec2_sub(vec2_t a, vec2_t b) { return vec2(a.x - b.x, a.y - b.y); }

vec3_t vec3_sub(vec3_t a, vec3_t b) {
  return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

vec4_t vec4_sub(vec4_t a, vec4_t b) {
  return vec4(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w);
}

/*
 * Scalar multiplication
 */

vec1_t vec1_mul(vec1_t v, f32 scalar) { return vec1(v.x * scalar); }

vec2_t vec2_mul(vec2_t v, f32 scalar) {
  return vec2(v.x * scalar, v.y * scalar);
}

vec3_t vec3_mul(vec3_t v, f32 scalar) {
  return vec3(v.x * scalar, v.y * scalar, v.z * scalar);
}

vec4_t vec4_mul(vec4_t v, f32 scalar) {
  return vec4(v.x * scalar, v.y * scalar, v.z * scalar, v.w * scalar);
}

/*
 * Scalar division
 */

vec1_t vec1_div(vec1_t v, f32 scalar) { return vec1(v.x / scalar); }

vec2_t vec2_div(vec2_t v, f32 scalar) {
  return vec2(v.x / scalar, v.y / scalar);
}

vec3_t vec3_div(vec3_t v, f32 scalar) {
  return vec3(v.x / scalar, v.y / scalar, v.z / scalar);
}

vec4_t vec4_div(vec4_t v, f32 scalar) {
  return vec4(v.x / scalar, v.y / scalar, v.z / scalar, v.w / scalar);
}

/*
 * Dot product
 */

f32 vec1_dot(vec1_t a, vec1_t b) { return a.x * b.x; }

f32 vec2_dot(vec2_t a, vec2_t b) { return (a.x * b.x) + (a.y * b.y); }

f32 vec3_dot(vec3_t a, vec3_t b) {
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

f32 vec4_dot(vec4_t a, vec4_t b) {
  return (a.x * b.x) + (a.y * b.y) + (a.z * b.z) + (a.w * b.w);
}

/*
 * Magnitude
 */

f32 vec1_magnitude(vec1_t v) { return sqrtf(v.x * v.x); }

f32 vec2_magnitude(vec2_t v) { return sqrtf((v.x * v.x) + (v.y * v.y)); }

f32 vec3_magnitude(vec3_t v) {
  return sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z));
}

f32 vec4_magnitude(vec4_t v) {
  return sqrtf((v.x * v.x) + (v.y * v.y) + (v.z * v.z) + (v.w * v.w));
}

/*
 * Normalization
 */

vec1_t vec1_normalize(vec1_t v) {
  f32 magnitude = vec1_magnitude(v);

  if (magnitude == 0.0f)
    return vec1(0.0f);

  return vec1_div(v, magnitude);
}

vec2_t vec2_normalize(vec2_t v) {
  f32 magnitude = vec2_magnitude(v);

  if (magnitude == 0.0f)
    return vec2(0.0f, 0.0f);

  return vec2_div(v, magnitude);
}

vec3_t vec3_normalize(vec3_t v) {
  f32 magnitude = vec3_magnitude(v);

  if (magnitude == 0.0f)
    return vec3(0.0f, 0.0f, 0.0f);

  return vec3_div(v, magnitude);
}

vec4_t vec4_normalize(vec4_t v) {
  f32 magnitude = vec4_magnitude(v);

  if (magnitude == 0.0f)
    return vec4(0.0f, 0.0f, 0.0f, 0.0f);

  return vec4_div(v, magnitude);
}

/*
 * Cross product
 */

vec3_t vec3_cross(vec3_t a, vec3_t b) {
  return vec3((a.y * b.z) - (a.z * b.y), (a.z * b.x) - (a.x * b.z),
              (a.x * b.y) - (a.y * b.x));
}
