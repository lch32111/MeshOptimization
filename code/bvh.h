#ifndef __BVH_H__
#define __BVH_H__

#include "aabb.h"

class PoolAllocator;

struct BV
{
    AABB aabb;
    Vector3 center;
    
    int left;
    int right;

    void* id;
};

struct BVH
{
    BV* bvs;
    int bv_count;
    int max_depth;
    PoolAllocator* allocator;
};

struct TriangleIndex
{
    int v[3];
};

// you can pass NULL to the allocator parameter, then the heap memory will be used.
BVH* bvh_create_from_triangles
(
    Vector3* __restrict positions,
    int position_count,
    TriangleIndex* __restrict indices,
    void** __restrict ids,
    int index_id_count,
    PoolAllocator* __restrict allocator
); 

typedef void(*BVHLeafCallback)(void* bvh_callback_context, void* user_callback_context, void* leaf_id);

void bvh_destroy(BVH* bvh);
void bvh_query_aabb(BVH* bvh, AABB aabb, BVHLeafCallback callback, void* user_callback_context);

/*
* bvh_query_point passes the pointer of `float closest_dist_sq;` through bvh_callback_context to query faster.
* You need to update this to accelerate the performance when you get the callback.
*/
void bvh_query_point(BVH* bvh, Vector3 query_point, BVHLeafCallback callback, void* user_callback_context);


#endif