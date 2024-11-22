#ifndef __OBJ_H__
#define __OBJ_H__

#include <vector>
#include <string>

#include "vector.h"

struct BVH;
struct AABB;


struct ObjShape
{
    ObjShape();
    ~ObjShape();

    std::string name;

    std::vector<float> positions;
    std::vector<float> normals;
    std::vector<int> indices;

    Vector3 min_position;
    Vector3 max_position;

    BVH* bvh;
};

void obj_get_shape(const char* path, std::vector<ObjShape>& out_shapes);
void obj_bvh_intersect_aabb_with_leaf(ObjShape* shape, AABB aabb, std::vector<int>* out_face_indices);
void obj_minimum_squared_distance(ObjShape* shape, Vector3 query_point, float* out_squared_distance, Vector3* out_closest_point, int* out_face_index);

#endif