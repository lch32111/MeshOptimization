#include "geometry_algorithm.h"

#include <math.h>

#include "vector.h"

// Real-Time Collision Detection by Christer Ericson p141-142
Vector3 triangle_closest_point(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
{
    // Check if p in vertex region outside A
    Vector3 ab = vector3_sub(b, a);
    Vector3 ac = vector3_sub(c, a);
    Vector3 ap = vector3_sub(p, a);
    float d1 = vector3_dot(ab, ap);
    float d2 = vector3_dot(ac, ap);

    if (d1 <= 0.f && d2 <= 0.f) return a; // barycentric coordinate (1, 0, 0)

    // Check if p in vertex region outside B
    Vector3 bp = vector3_sub(p, b);
    float d3 = vector3_dot(ab, bp);
    float d4 = vector3_dot(ac, bp);
    if (d3 >= 0.f && d4 <= d3) return b; // barycentric coordintaes (0, 1, 0)

    // Check if p in edge region of AB, if so return projection of P onto AB
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        float v = d1 / (d1 - d3);
        return vector3_add(a, vector3_mul_scalar(ab, v)); // barycentric coordinates (1 - v, v, 0)
    }

    // Check if p in vertex region outside C
    Vector3 cp = vector3_sub(p, c);
    float d5 = vector3_dot(ab, cp);
    float d6 = vector3_dot(ac, cp);
    if (d6 >= 0.f && d5 <= d6) return c; // barycentric coordintaes (0, 0, 1)

    // Check if p in edge region of AC, if so return projection of P onto AC
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        float  w = d2 / (d2 - d6);
        return vector3_add(a, vector3_mul_scalar(ac, w)); // barycentric coordinates (1 - w, 0, w)
    }

    // Check if p in edge region of BC, if so return projection of p onto BC
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
    {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return vector3_add(b, vector3_mul_scalar(vector3_sub(c, b), w)); // barycentric coordintaes (0, 1 - w, w)
    }

    // P inside face region. Compute Q through its barycentric coordinates (u, v, w)
    float denom = 1.f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    return vector3_add(a, vector3_add(vector3_mul_scalar(ab, v), vector3_mul_scalar(ac, w)));
}

void triangle_closest_point_detail(Vector3 p, Vector3 a, Vector3 b, Vector3 c, TriangleClosestPointResult* out_result)
{
    // Check if p in vertex region outside A
    Vector3 ab = vector3_sub(b, a);
    Vector3 ac = vector3_sub(c, a);
    Vector3 ap = vector3_sub(p, a);
    float d1 = vector3_dot(ab, ap);
    float d2 = vector3_dot(ac, ap);

    if (d1 <= 0.f && d2 <= 0.f) // barycentric coordinate (1, 0, 0)
    {
        out_result->closest_point = a;
        out_result->barycentric = vector3_set3(1.f, 0.f, 0.f);
        out_result->dist_sq = vector3_distance_sq(p, a);
        return;
    }

    // Check if p in vertex region outside B
    Vector3 bp = vector3_sub(p, b);
    float d3 = vector3_dot(ab, bp);
    float d4 = vector3_dot(ac, bp);
    if (d3 >= 0.f && d4 <= d3) // barycentric coordintaes (0, 1, 0)
    {
        out_result->closest_point = b;
        out_result->barycentric = vector3_set3(0.f, 1.f, 0.f);
        out_result->dist_sq = vector3_distance(p, b);
        return;
    }

    // Check if p in edge region of AB, if so return projection of P onto AB
    float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
    {
        // barycentric coordinates (1 - v, v, 0)
        float v = d1 / (d1 - d3);
        out_result->closest_point = vector3_add(a, vector3_mul_scalar(ab, v));
        out_result->barycentric = vector3_set3(1.f - v, v, 0.f);
        out_result->dist_sq = vector3_distance_sq(p, out_result->closest_point);
        return;
    }

    // Check if p in vertex region outside C
    Vector3 cp = vector3_sub(p, c);
    float d5 = vector3_dot(ab, cp);
    float d6 = vector3_dot(ac, cp);
    if (d6 >= 0.f && d5 <= d6) // barycentric coordintaes (0, 0, 1)
    {
        out_result->closest_point = c;
        out_result->barycentric = vector3_set3(0.f, 0.f, 1.f);
        out_result->dist_sq = vector3_distance_sq(p, c);
        return;
    }

    // Check if p in edge region of AC, if so return projection of P onto AC
    float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
    {
        // barycentric coordinates (1 - w, 0, w)
        float  w = d2 / (d2 - d6);
        out_result->closest_point = vector3_add(a, vector3_mul_scalar(ac, w));
        out_result->barycentric = vector3_set3(1.f - w, 0.f, w);
        out_result->dist_sq = vector3_distance_sq(p, out_result->closest_point);
        return;
    }

    // Check if p in edge region of BC, if so return projection of p onto BC
    float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
    {
        // barycentric coordintaes (0, 1 - w, w)
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        out_result->closest_point = vector3_add(b, vector3_mul_scalar(vector3_sub(c, b), w));
        out_result->barycentric = vector3_set3(0.f, 1.f - w, w);
        out_result->dist_sq = vector3_distance_sq(p, out_result->closest_point);
        return;
    }

    // P inside face region. Compute Q through its barycentric coordinates (u, v, w)
    float denom = 1.f / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    out_result->closest_point = vector3_add(a, vector3_add(vector3_mul_scalar(ab, v), vector3_mul_scalar(ac, w)));
    out_result->barycentric = vector3_set3(1.f - v - w, v, w);
    out_result->dist_sq = out_result->dist_sq = vector3_distance_sq(p, out_result->closest_point);
}

float triangle_area(Vector3 a, Vector3 b, Vector3 c)
{
    Vector3 ab = vector3_sub(b, a);
    Vector3 ac = vector3_sub(c, a);
    Vector3 avec = vector3_cross(ab, ac);
    float area = vector3_length(avec) * 0.5;
    return area;
}

/*
* downward view
*    p2
*    /|\
*   / | \
*  /  |  \
*po1  |   po2
*  \  |  /
*   \ | /
*    \|/
*     p1
*
*     p1
*     /x\
*    /   \
*   /     \
*  /       \
*po1       po2
*
* cosine value of the dihedral angle triangle(p1, p2, po1) and triangle(p1, po2, p2)
*/
float dihedral_angle_cos(Vector3 p1, Vector3 p2, Vector3 po1, Vector3 po2)
{
    Vector3 ves1 = vector3_cross(vector3_sub(p2, p1), vector3_sub(po1, p1));
    float ves1_len_sq = vector3_length_sq(ves1);
    
    if (ves1_len_sq == 0.f) // degenerate triangle
        return -2.f;

    Vector3 ves2 = vector3_cross(vector3_sub(po2, p1), vector3_sub(p2, p1));
    float ves2_len_sq = vector3_length_sq(ves2);
    if (ves2_len_sq == 0.f)
        return -2.f;

    ves1 = vector3_mul_scalar(ves1, 1.f / sqrtf(ves1_len_sq));
    ves2 = vector3_mul_scalar(ves2, 1.f / sqrtf(ves2_len_sq));

    float d = vector3_dot(ves1, ves2);
    if (d < -1.f) d = -1.f;
    if (d > 1.f) d = 1.f;
    return d;
}