#ifndef __AABB_H__
#define __AABB_H__

#include "vector.h"

struct Matrix4;

struct AABB
{
    Vector3 min_p;
    Vector3 max_p;
};

void aabb_set_infinity(AABB* aabb);
void aabb_set_min_max(AABB* aabb, float x, float y, float z);
void aabb_combine_aabb(AABB* dest, AABB* src);
void aabb_combine_float(AABB* dest, float* src);
void aabb_get_center(AABB* aabb, float* out_centers);
void aabb_get_extents(AABB* aabb, float* out_extents);
void aabb_get_half_extents(AABB* aabb, float* out_half_extents);
int aabb_get_logest_axis_index(AABB* aabb); // 0 - X, 1 - Y, 2 - Z
bool aabb_intersect_aabb(AABB* a, AABB* b);
bool aabb_contain_point(AABB* aabb, Vector3 query_point);
float aabb_distance_exterior_sq_point(AABB* aabb, Vector3 query_point);

Matrix4 aabb_get_transform_to_unit_cube(AABB* aabb);

#endif