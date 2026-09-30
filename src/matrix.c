#include <math/matrix.h>

#include <math.h>

/*
 * Constructors
 */

mat2_t mat2(f32 m00, f32 m01, f32 m10, f32 m11) {
  return (mat2_t){.m = {{m00, m01}, {m10, m11}}};
}

mat3_t mat3(f32 m00, f32 m01, f32 m02, f32 m10, f32 m11, f32 m12, f32 m20,
            f32 m21, f32 m22) {
  return (mat3_t){.m = {{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}}};
}

mat4_t mat4(f32 m00, f32 m01, f32 m02, f32 m03, f32 m10, f32 m11, f32 m12,
            f32 m13, f32 m20, f32 m21, f32 m22, f32 m23, f32 m30, f32 m31,
            f32 m32, f32 m33) {
  return (mat4_t){.m = {{m00, m01, m02, m03},
                        {m10, m11, m12, m13},
                        {m20, m21, m22, m23},
                        {m30, m31, m32, m33}}};
}

/*
 * Identity matrices
 */

mat2_t mat2_identity(void) { return mat2(1.0f, 0.0f, 0.0f, 1.0f); }

mat3_t mat3_identity(void) {
  return mat3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}

mat4_t mat4_identity(void) {
  return mat4(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
              0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
}

/*
 * Matrix addition
 */

mat2_t mat2_add(mat2_t a, mat2_t b) {
  mat2_t result;

  for (u32 i = 0; i < 2; i++)
    for (u32 j = 0; j < 2; j++)
      result.m[i][j] = a.m[i][j] + b.m[i][j];

  return result;
}

mat3_t mat3_add(mat3_t a, mat3_t b) {
  mat3_t result;

  for (u32 i = 0; i < 3; i++)
    for (u32 j = 0; j < 3; j++)
      result.m[i][j] = a.m[i][j] + b.m[i][j];

  return result;
}

mat4_t mat4_add(mat4_t a, mat4_t b) {
  mat4_t result;

  for (u32 i = 0; i < 4; i++)
    for (u32 j = 0; j < 4; j++)
      result.m[i][j] = a.m[i][j] + b.m[i][j];

  return result;
}

/*
 * Matrix subtraction
 */

mat2_t mat2_sub(mat2_t a, mat2_t b) {
  mat2_t result;

  for (u32 i = 0; i < 2; i++)
    for (u32 j = 0; j < 2; j++)
      result.m[i][j] = a.m[i][j] - b.m[i][j];

  return result;
}

mat3_t mat3_sub(mat3_t a, mat3_t b) {
  mat3_t result;

  for (u32 i = 0; i < 3; i++)
    for (u32 j = 0; j < 3; j++)
      result.m[i][j] = a.m[i][j] - b.m[i][j];

  return result;
}

mat4_t mat4_sub(mat4_t a, mat4_t b) {
  mat4_t result;

  for (u32 i = 0; i < 4; i++)
    for (u32 j = 0; j < 4; j++)
      result.m[i][j] = a.m[i][j] - b.m[i][j];

  return result;
}

/*
 * Scalar multiplication
 */

mat2_t mat2_mul(mat2_t m, f32 scalar) {
  for (u32 i = 0; i < 2; i++)
    for (u32 j = 0; j < 2; j++)
      m.m[i][j] *= scalar;

  return m;
}

mat3_t mat3_mul(mat3_t m, f32 scalar) {
  for (u32 i = 0; i < 3; i++)
    for (u32 j = 0; j < 3; j++)
      m.m[i][j] *= scalar;

  return m;
}

mat4_t mat4_mul(mat4_t m, f32 scalar) {
  for (u32 i = 0; i < 4; i++)
    for (u32 j = 0; j < 4; j++)
      m.m[i][j] *= scalar;

  return m;
}

/*
 * Matrix multiplication
 */

mat2_t mat2_mul_mat2(mat2_t a, mat2_t b) {
  mat2_t result = {0};

  for (u32 i = 0; i < 2; i++)
    for (u32 j = 0; j < 2; j++)
      for (u32 k = 0; k < 2; k++)
        result.m[i][j] += a.m[i][k] * b.m[k][j];

  return result;
}

mat3_t mat3_mul_mat3(mat3_t a, mat3_t b) {
  mat3_t result = {0};

  for (u32 i = 0; i < 3; i++)
    for (u32 j = 0; j < 3; j++)
      for (u32 k = 0; k < 3; k++)
        result.m[i][j] += a.m[i][k] * b.m[k][j];

  return result;
}

mat4_t mat4_mul_mat4(mat4_t a, mat4_t b) {
  mat4_t result = {0};

  for (u32 i = 0; i < 4; i++)
    for (u32 j = 0; j < 4; j++)
      for (u32 k = 0; k < 4; k++)
        result.m[i][j] += a.m[i][k] * b.m[k][j];

  return result;
}

/*
 * Matrix vector multiplication
 */

vec2_t mat2_mul_vec2(mat2_t m, vec2_t v) {
  return vec2((m.m[0][0] * v.x) + (m.m[0][1] * v.y),
              (m.m[1][0] * v.x) + (m.m[1][1] * v.y));
}

vec3_t mat3_mul_vec3(mat3_t m, vec3_t v) {
  return vec3((m.m[0][0] * v.x) + (m.m[0][1] * v.y) + (m.m[0][2] * v.z),

              (m.m[1][0] * v.x) + (m.m[1][1] * v.y) + (m.m[1][2] * v.z),

              (m.m[2][0] * v.x) + (m.m[2][1] * v.y) + (m.m[2][2] * v.z));
}

vec4_t mat4_mul_vec4(mat4_t m, vec4_t v) {
  return vec4((m.m[0][0] * v.x) + (m.m[0][1] * v.y) + (m.m[0][2] * v.z) +
                  (m.m[0][3] * v.w),

              (m.m[1][0] * v.x) + (m.m[1][1] * v.y) + (m.m[1][2] * v.z) +
                  (m.m[1][3] * v.w),

              (m.m[2][0] * v.x) + (m.m[2][1] * v.y) + (m.m[2][2] * v.z) +
                  (m.m[2][3] * v.w),

              (m.m[3][0] * v.x) + (m.m[3][1] * v.y) + (m.m[3][2] * v.z) +
                  (m.m[3][3] * v.w));
}

/*
 * Transpose
 */

mat2_t mat2_transpose(mat2_t m) {
  mat2_t result;

  for (u32 i = 0; i < 2; i++)
    for (u32 j = 0; j < 2; j++)
      result.m[i][j] = m.m[j][i];

  return result;
}

mat3_t mat3_transpose(mat3_t m) {
  mat3_t result;

  for (u32 i = 0; i < 3; i++)
    for (u32 j = 0; j < 3; j++)
      result.m[i][j] = m.m[j][i];

  return result;
}

mat4_t mat4_transpose(mat4_t m) {
  mat4_t result;

  for (u32 i = 0; i < 4; i++)
    for (u32 j = 0; j < 4; j++)
      result.m[i][j] = m.m[j][i];

  return result;
}

/*
 * Determinant
 */

f32 mat2_det(mat2_t m) {
  return (m.m[0][0] * m.m[1][1]) - (m.m[0][1] * m.m[1][0]);
}

f32 mat3_det(mat3_t m) {
  return m.m[0][0] * ((m.m[1][1] * m.m[2][2]) - (m.m[1][2] * m.m[2][1])) -
         m.m[0][1] * ((m.m[1][0] * m.m[2][2]) - (m.m[1][2] * m.m[2][0])) +
         m.m[0][2] * ((m.m[1][0] * m.m[2][1]) - (m.m[1][1] * m.m[2][0]));
}

/*
 * Inverse
 */

mat2_t mat2_inverse(mat2_t m) {
  f32 det = mat2_det(m);

  if (det == 0.0f)
    return (mat2_t){0};

  f32 inv_det = 1.0f / det;

  return mat2(m.m[1][1] * inv_det, -m.m[0][1] * inv_det,

              -m.m[1][0] * inv_det, m.m[0][0] * inv_det);
}

mat3_t mat3_inverse(mat3_t m) {
  f32 det = mat3_det(m);

  if (det == 0.0f)
    return (mat3_t){0};

  f32 inv_det = 1.0f / det;

  return mat3((m.m[1][1] * m.m[2][2] - m.m[1][2] * m.m[2][1]) * inv_det,

              (m.m[0][2] * m.m[2][1] - m.m[0][1] * m.m[2][2]) * inv_det,

              (m.m[0][1] * m.m[1][2] - m.m[0][2] * m.m[1][1]) * inv_det,

              (m.m[1][2] * m.m[2][0] - m.m[1][0] * m.m[2][2]) * inv_det,

              (m.m[0][0] * m.m[2][2] - m.m[0][2] * m.m[2][0]) * inv_det,

              (m.m[0][2] * m.m[1][0] - m.m[0][0] * m.m[1][2]) * inv_det,

              (m.m[1][0] * m.m[2][1] - m.m[1][1] * m.m[2][0]) * inv_det,

              (m.m[0][1] * m.m[2][0] - m.m[0][0] * m.m[2][1]) * inv_det,

              (m.m[0][0] * m.m[1][1] - m.m[0][1] * m.m[1][0]) * inv_det);
}

/*
 * 4x4 determinant / inverse
 *
 * TODO:
 * Implement after establishing numerical/error semantics.
 */

f32 mat4_det(mat4_t m) {
  f32 det = 0.0f;

  for (u32 col = 0; col < 4; col++) {
    mat3_t minor = {0};

    u32 minor_row = 0;

    for (u32 row = 1; row < 4; row++) {
      u32 minor_col = 0;

      for (u32 current_col = 0; current_col < 4; current_col++) {
        if (current_col == col)
          continue;

        minor.m[minor_row][minor_col] = m.m[row][current_col];

        minor_col++;
      }

      minor_row++;
    }

    f32 sign = (col % 2 == 0) ? 1.0f : -1.0f;

    det += sign * m.m[0][col] * mat3_det(minor);
  }

  return det;
}

mat4_t mat4_inverse(mat4_t m) {
  f32 det = mat4_det(m);

  if (det == 0.0f)
    return (mat4_t){0};

  mat4_t result = {0};

  for (u32 row = 0; row < 4; row++) {
    for (u32 col = 0; col < 4; col++) {
      mat3_t minor = {0};

      u32 minor_row = 0;

      for (u32 i = 0; i < 4; i++) {
        if (i == row)
          continue;

        u32 minor_col = 0;

        for (u32 j = 0; j < 4; j++) {
          if (j == col)
            continue;

          minor.m[minor_row][minor_col] = m.m[i][j];

          minor_col++;
        }

        minor_row++;
      }

      f32 sign = ((row + col) % 2 == 0) ? 1.0f : -1.0f;

      /*
       * Cofactor transpose (adjugate)
       */
      result.m[col][row] = sign * mat3_det(minor) / det;
    }
  }

  return result;
}
