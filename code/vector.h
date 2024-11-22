#ifndef __VECTOR_H__
#define __VECTOR_H__

struct Vector3
{
    float v[3];
};

struct Vector4
{
    float v[4];
};

extern Vector3 VECTOR3_COLOR_RED;
extern Vector3 VECTOR3_COLOR_GREEN;
extern Vector3 VECTOR3_COLOR_BLUE;
extern Vector3 VECTOR3_COLOR_WHITE;
extern Vector3 VECTOR3_COLOR_ORANGE;
extern Vector3 VECTOR3_COLOR_YELLOW;
extern Vector3 VECTOR3_COLOR_INDIGO;
extern Vector3 VECTOR3_COLOR_PURPLE;
extern Vector3 VECTOR3_COLOR_MAGENTA;
extern Vector3 VECTOR3_COLOR_PALE_PINK;
extern Vector3 VECTOR3_COLOR_LIGHT_PINK;
extern Vector3 VECTOR3_COLOR_LAVENDER;
extern Vector3 VECTOR3_COLOR_LIGHT_PURPLE;
extern Vector3 VECTOR3_COLOR_LIGHT_BLUE;
extern Vector3 VECTOR3_COLOR_SKY_BLUE;
extern Vector3 VECTOR3_COLOR_MINT_GREEN;
extern Vector3 VECTOR3_COLOR_PALE_GREEN;
extern Vector3 VECTOR3_COLOR_LIGHT_YELLOW;
extern Vector3 VECTOR3_COLOR_LIGHT_PEACH;

Vector3 vector3_set1(float v);
Vector3 vector3_set3(float v1, float v2, float v3);
Vector3 vector3_setp(const float* v);
Vector3 vector3_add(Vector3 a, Vector3 b);
Vector3 vector3_sub(Vector3 a, Vector3 b);
Vector3 vector3_mul_scalar(Vector3 a, float s);
Vector3 vector3_normalize(Vector3 v);
Vector3 vector3_cross(Vector3 a, Vector3 b);
float vector3_dot(Vector3 a, Vector3 b);
float vector3_distance(Vector3 a, Vector3 b);
float vector3_distance_sq(Vector3 a, Vector3 b);
float vector3_length(Vector3 v);
float vector3_length_sq(Vector3 v);
float vector3_max(Vector3 v);
float vector3_min(Vector3 v);
float vector3_angle(Vector3 v1, Vector3 v2);

Vector4 vector4_set1(float v);
Vector4 vector4_set4(float v1, float v2, float v3, float v4);
Vector4 vector4_setp(const float* v);
Vector4 vector4_add(Vector4 a, Vector4 b);
Vector4 vector4_sub(Vector4 a, Vector4 b);
Vector4 vector4_mul(Vector4 a, Vector4 b);
Vector4 vector4_mul_scalar(Vector4 a, float s);


#endif 