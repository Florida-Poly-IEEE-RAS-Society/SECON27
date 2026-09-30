#pragma once

#include <math/types.h>
#include <math/vector.h>

/*
 * Matrix types
 */

typedef struct {
  f32 m[2][2];
} mat2_t;

typedef struct {
  f32 m[3][3];
} mat3_t;

typedef struct {
  f32 m[4][4];
} mat4_t;

/*
 * Constructors
 */

mat2_t mat2(f32 m00, f32 m01, f32 m10, f32 m11);

mat3_t mat3(f32 m00, f32 m01, f32 m02, f32 m10, f32 m11, f32 m12, f32 m20,
            f32 m21, f32 m22);

mat4_t mat4(f32 m00, f32 m01, f32 m02, f32 m03, f32 m10, f32 m11, f32 m12,
            f32 m13, f32 m20, f32 m21, f32 m22, f32 m23, f32 m30, f32 m31,
            f32 m32, f32 m33);

/*
 * Identity matrices
 */

mat2_t mat2_identity(void);
mat3_t mat3_identity(void);
mat4_t mat4_identity(void);

/*
 * Matrix addition
 */

mat2_t mat2_add(mat2_t a, mat2_t b);
mat3_t mat3_add(mat3_t a, mat3_t b);
mat4_t mat4_add(mat4_t a, mat4_t b);

/*
 * Matrix subtraction
 */

mat2_t mat2_sub(mat2_t a, mat2_t b);
mat3_t mat3_sub(mat3_t a, mat3_t b);
mat4_t mat4_sub(mat4_t a, mat4_t b);

/*
 * Scalar multiplication
 */

mat2_t mat2_mul(mat2_t m, f32 scalar);
mat3_t mat3_mul(mat3_t m, f32 scalar);
mat4_t mat4_mul(mat4_t m, f32 scalar);

/*
 * Matrix multiplication
 */

mat2_t mat2_mul_mat2(mat2_t a, mat2_t b);
mat3_t mat3_mul_mat3(mat3_t a, mat3_t b);
mat4_t mat4_mul_mat4(mat4_t a, mat4_t b);

/*
 * Matrix vector multiplication
 */

vec2_t mat2_mul_vec2(mat2_t m, vec2_t v);
vec3_t mat3_mul_vec3(mat3_t m, vec3_t v);
vec4_t mat4_mul_vec4(mat4_t m, vec4_t v);

/*
 * Transpose
 */

mat2_t mat2_transpose(mat2_t m);
mat3_t mat3_transpose(mat3_t m);
mat4_t mat4_transpose(mat4_t m);

/*
 * Determinant
 */

f32 mat2_det(mat2_t m);
f32 mat3_det(mat3_t m);
f32 mat4_det(mat4_t m);

/*
 * Inverse
 */

mat2_t mat2_inverse(mat2_t m);
mat3_t mat3_inverse(mat3_t m);
mat4_t mat4_inverse(mat4_t m);
