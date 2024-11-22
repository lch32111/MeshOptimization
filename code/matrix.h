#ifndef __MATRIX_H__
#define __MATRIX_H__

struct Vector3;

// 4x4 matrix with column-wise memory layout and column-wise interface
struct Matrix4
{
    float v[4][4];
};

Matrix4 matrix4_translate(Vector3 v);
Matrix4 matrix4_scale(Vector3 s);
Matrix4 matrix4_mul(const Matrix4* a, const Matrix4* b);
Vector3 matrix4_mul_vec3(const Matrix4* m, Vector3 v);
Matrix4 matrix4_mul_scalar(const Matrix4* a, float s);
Matrix4 matrix4_inverse(Matrix4* m);


#endif