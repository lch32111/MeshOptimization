#include "vector.h"
#include <math.h>

Vector3 VECTOR3_COLOR_RED = { 1.f, 0.f, 0.f };
Vector3 VECTOR3_COLOR_GREEN = { 0.f, 1.f, 0.f };
Vector3 VECTOR3_COLOR_BLUE = { 0.f, 0.f, 1.f };
Vector3 VECTOR3_COLOR_WHITE = { 1.f, 1.f, 1.f };
Vector3 VECTOR3_COLOR_ORANGE = { 1.0f, 0.64705884f, 0.0f };
Vector3 VECTOR3_COLOR_YELLOW = { 1.0f, 1.0f, 0.0f };
Vector3 VECTOR3_COLOR_INDIGO = { 0.29411766f, 0.0f, 0.50980395f };
Vector3 VECTOR3_COLOR_PURPLE = { 0.5019608f, 0.0f, 0.5019608f };
Vector3 VECTOR3_COLOR_MAGENTA = { 1.0f, 0.0f, 1.0f };
Vector3 VECTOR3_COLOR_PALE_PINK = { 1.0f, 0.8627451f, 0.8627451f };
Vector3 VECTOR3_COLOR_LIGHT_PINK = { 1.0f, 0.7137255f, 0.7568627f };
Vector3 VECTOR3_COLOR_LAVENDER = { 0.9019608f, 0.9019608f, 0.9803922f };
Vector3 VECTOR3_COLOR_LIGHT_PURPLE = { 0.7490196f, 0.7137255f, 0.8235294f };
Vector3 VECTOR3_COLOR_LIGHT_BLUE = { 0.6784314f, 0.8470588f, 0.9019608f };
Vector3 VECTOR3_COLOR_SKY_BLUE = { 0.5294118f, 0.80784315f, 0.92156863f };
Vector3 VECTOR3_COLOR_MINT_GREEN = { 0.59607846f, 0.9843137f, 0.59607846f };
Vector3 VECTOR3_COLOR_PALE_GREEN = { 0.6862745f, 0.93333334f, 0.93333334f };
Vector3 VECTOR3_COLOR_LIGHT_YELLOW = { 1.0f, 1.0f, 0.8784314f };
Vector3 VECTOR3_COLOR_LIGHT_PEACH = { 1.0f, 0.9372549f, 0.85882354f };

Vector3 vector3_set1(float v)
{
    Vector3 r = { v, v, v };
    return r;
}

Vector3 vector3_set3(float v1, float v2, float v3)
{
    Vector3 r = { v1, v2, v3 };
    return r;
}

Vector3 vector3_setp(const float* v)
{
    Vector3 r = { v[0], v[1], v[2] };
    return r;
}

Vector3 vector3_add(Vector3 a, Vector3 b)
{
    Vector3 r =
    {
        a.v[0] + b.v[0],
        a.v[1] + b.v[1],
        a.v[2] + b.v[2]
    };

    return r;
}

Vector3 vector3_sub(Vector3 a, Vector3 b)
{
    Vector3 r =
    {
        a.v[0] - b.v[0],
        a.v[1] - b.v[1],
        a.v[2] - b.v[2]
    };

    return r;
}

Vector3 vector3_mul_scalar(Vector3 a, float s)
{
    Vector3 r =
    {
        a.v[0] * s,
        a.v[1] * s,
        a.v[2] * s
    };

    return r;
}

Vector3 vector3_normalize(Vector3 v)
{
    // safe normalization
    float d = v.v[0] * v.v[0] + v.v[1] * v.v[1] + v.v[2] * v.v[2];

    if (d != 0.f)
    {
        d = 1.f / sqrtf(d);
        v.v[0] *= d;
        v.v[1] *= d;
        v.v[2] *= d;
    }

    return v;
}

Vector3 vector3_cross(Vector3 a, Vector3 b)
{
    Vector3 r =
    {
        a.v[1] * b.v[2] - a.v[2] * b.v[1],
        a.v[2] * b.v[0] - a.v[0] * b.v[2],
        a.v[0] * b.v[1] - a.v[1] * b.v[0]
    };

    return r;
}

float vector3_dot(Vector3 a, Vector3 b)
{
    return a.v[0] * b.v[0] + a.v[1] * b.v[1] + a.v[2] * b.v[2];
}

float vector3_distance(Vector3 a, Vector3 b)
{
    Vector3 r =
    {
        a.v[0] - b.v[0],
        a.v[1] - b.v[1],
        a.v[2] - b.v[2]
    };
    float d = r.v[0] * r.v[0] + r.v[1] * r.v[1] + r.v[2] * r.v[2];
    return sqrtf(d);
}

float vector3_distance_sq(Vector3 a, Vector3 b)
{
    Vector3 r =
    {
        a.v[0] - b.v[0],
        a.v[1] - b.v[1],
        a.v[2] - b.v[2]
    };
    float d = r.v[0] * r.v[0] + r.v[1] * r.v[1] + r.v[2] * r.v[2];
    return d;
}

float vector3_length(Vector3 v)
{
    float d = v.v[0] * v.v[0] + v.v[1] * v.v[1] + v.v[2] * v.v[2];
    return sqrtf(d);
}

float vector3_length_sq(Vector3 v)
{
    float d = v.v[0] * v.v[0] + v.v[1] * v.v[1] + v.v[2] * v.v[2];
    return d;
}

float vector3_max(Vector3 v)
{
    return v.v[0] > v.v[1] ?
        v.v[0] > v.v[2] ? v.v[0] : v.v[2] :
        v.v[1] > v.v[2] ? v.v[1] : v.v[2];
}

float vector3_min(Vector3 v)
{
    return v.v[0] < v.v[1] ?
        v.v[0] < v.v[2] ? v.v[0] : v.v[2] :
        v.v[1] < v.v[2] ? v.v[1] : v.v[2];
}

// https://github.com/hhoppe/Mesh-processing-library/blob/main/libHh/Geometry.h
// More robust than acos(dot()) for small angles!
float vector3_angle(Vector3 va, Vector3 vb)
{
    constexpr float TAU = 6.2831853071795864769f;    // Mathematica: N[2 Pi, 20]; see http://tauday.com/

    // va and vb should be normalized.
    float vdot = vector3_dot(va, vb);
    if (vdot > +.95f) 
    {
        return asin(vector3_length(vector3_cross(va, vb)));
    }
    else if (vdot < -.95f) 
    {
        return TAU / 2 - asin(vector3_length(vector3_cross(va, vb)));
    }
    else 
    {
        return acos(vdot);
    }
}

Vector4 vector4_set1(float v)
{
    Vector4 r = { v, v, v, v };
    return r;
}

Vector4 vector4_set4(float v1, float v2, float v3, float v4)
{
    Vector4 r = { v1, v2, v3, v4 };
    return r;
}

Vector4 vector4_setp(const float* v)
{
    Vector4 r = { v[0], v[1], v[2], v[3] };
    return r;
}

Vector4 vector4_add(Vector4 a, Vector4 b)
{
    Vector4 r = 
    { 
        a.v[0] + b.v[0], 
        a.v[1] + b.v[1], 
        a.v[2] + b.v[2], 
        a.v[3] + b.v[3] 
    };
    return r;
}

Vector4 vector4_sub(Vector4 a, Vector4 b)
{
    Vector4 r =
    {
        a.v[0] - b.v[0],
        a.v[1] - b.v[1],
        a.v[2] - b.v[2],
        a.v[3] - b.v[3]
    };
    return r;
}

Vector4 vector4_mul(Vector4 a, Vector4 b)
{
    Vector4 r = 
    { 
        a.v[0] * b.v[0], 
        a.v[1] * b.v[1], 
        a.v[2] * b.v[2], 
        a.v[3] * b.v[3] 
    };
    return r;
}

Vector4 vector4_mul_scalar(Vector4 a, float s)
{
    Vector4 r =
    {
        a.v[0] * s,
        a.v[1] * s,
        a.v[2] * s,
        a.v[3] * s
    };

    return r;
}