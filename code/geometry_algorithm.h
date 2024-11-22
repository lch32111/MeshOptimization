#ifndef __GEOMETRY_ALGORITHM_H__
#define __GEOMETRY_ALGORITHM_H__

#include "vector.h"

Vector3 triangle_closest_point(Vector3 p, Vector3 a, Vector3 b, Vector3 c);

struct TriangleClosestPointResult
{
    Vector3 closest_point;
    Vector3 barycentric;
    float dist_sq; // bewteen p and closest_point
};

void triangle_closest_point_detail(Vector3 p, Vector3 a, Vector3 b, Vector3 c, TriangleClosestPointResult* out_result);

float triangle_area(Vector3 a, Vector3 b, Vector3 c);

/*
* From Hugues Hoppe https://github.com/hhoppe/Mesh-processing-library
* return cosine value of dihedral signed angle defined by two triangles.
* the returned value is clipped to -1.f or 1.f if it's beyond the range.
* In the case that a triangle is degenerate, it return -2.f
*/
float dihedral_angle_cos(Vector3 p1, Vector3 p2, Vector3 po1, Vector3 po2);

#endif