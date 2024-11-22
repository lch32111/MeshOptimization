#include "obj.h"

#include <string>
#include <assert.h>

#include "common.h"
#include "geometry_algorithm.h"
#include "bvh.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

ObjShape::ObjShape()
    : bvh(NULL)
{ }

ObjShape::~ObjShape()
{
    if (bvh != NULL)
    {
        bvh_destroy(bvh);
    }
}

/*
static inline void push_obj_data(std::vector<ObjShape>& od_shapes, const std::vector<Shape>& shapes)
{
    od_shapes.clear();
    od_shapes.resize(shapes.size());

    std::vector<void*> bv_ids;

    for (size_t si = 0; si < od_shapes.size(); ++si)
    {
        const Shape& shape = shapes[si];
        ObjShape& od_shape = od_shapes[si];

        od_shape.name = shape.name;

        od_shape.min_position = vector3_set1(FLT_MAX);
        od_shape.max_position = vector3_set1(-FLT_MAX);

        od_shape.positions.resize(shape.positions.size());
        assert(shape.positions.size() % 3 == 0);
        for (size_t pi = 0; pi < shape.positions.size(); pi += 3)
        {
            od_shape.positions[pi] = shape.positions[pi];
            od_shape.positions[pi + 1] = shape.positions[pi + 1];
            od_shape.positions[pi + 2] = shape.positions[pi + 2];

            for (size_t pii = 0; pii < 3; ++pii)
            {
                if (shape.positions[pi + pii] < od_shape.min_position.v[pii])
                {
                    od_shape.min_position.v[pii] = shape.positions[pi + pii];
                }

                if (od_shape.max_position.v[pii] < shape.positions[pi + pii])
                {
                    od_shape.max_position.v[pii] = shape.positions[pi + pii];
                }
            }
        }

        od_shape.indices.resize(shape.pos_indices.size());
        memcpy(od_shape.indices.data(), shape.pos_indices.data(), sizeof(int) * shape.pos_indices.size());

        od_shape.normals.resize(shape.positions.size());
        bv_ids.resize(shape.pos_indices.size() / 3);
        for (size_t ni = 0; ni < shape.pos_indices.size(); ni += 3)
        {
            int vi[3] = { shape.pos_indices[ni] * 3, shape.pos_indices[ni + 1] * 3, shape.pos_indices[ni + 2] * 3 };

            Vector3 p0 = vector3_setp(&(shape.positions[vi[0]]));
            Vector3 p1 = vector3_setp(&(shape.positions[vi[1]]));
            Vector3 p2 = vector3_setp(&(shape.positions[vi[2]]));
            Vector3 normal = vector3_normalize(vector3_cross(vector3_sub(p1, p0), vector3_sub(p2, p0)));

            for (int i = 0; i < 3; ++i)
            {
                od_shape.normals[vi[i]] += normal.v[0];
                od_shape.normals[vi[i] + 1] += normal.v[1];
                od_shape.normals[vi[i] + 2] += normal.v[2];
            }

            bv_ids[ni / 3] = (void*)((int)ni);
        }

        for (size_t ni = 0; ni < od_shape.normals.size(); ni += 3)
        {
            float n[3] = { od_shape.normals[ni], od_shape.normals[ni + 1], od_shape.normals[ni + 2] };

            float inv_len = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
            if (inv_len > 0.f)
            {
                inv_len = 1.f / sqrtf(inv_len);

                od_shape.normals[ni] *= inv_len;
                od_shape.normals[ni + 1] *= inv_len;
                od_shape.normals[ni + 2] *= inv_len;
            }
        }

        od_shape.bvh = bvh_create_from_triangles
        (
            (Vector3*)(od_shape.positions.data()),
            od_shape.positions.size() / 3,
            (TriangleIndex*)(od_shape.indices.data()),
            bv_ids.data(),
            shape.pos_indices.size() / 3,
            NULL
        );
    }
}
*/

void obj_get_shape(const char* path, std::vector<ObjShape>& out_shapes)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    std::string warn, err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path);
    if (!warn.empty())
        printf("%s\n", warn.c_str());

    if (!err.empty())
        printf("%s\n", err.c_str());
    assert(ret == true);
    
    size_t shape_count = shapes.size();

    out_shapes.resize(shape_count);

    std::vector<void*> bv_ids;

    std::unordered_map<int, int> indices_map; // key original index, value reordered index

    for (size_t si = 0; si < shape_count; ++si)
    {
        tinyobj::shape_t& src_shape = shapes[si];
        ObjShape& dest_shape = out_shapes[si];

        tinyobj::mesh_t& mesh = src_shape.mesh;
        std::vector <tinyobj::index_t>& mesh_indices = mesh.indices;

        // first get unique indices for this shape
        // then fill the dest_shape.positions only with the unique indices.
        // The reason for doing this is that we want to get only vertices 
        // that are referenced by indices.
        // I believe tinyobjload already removes duplicate positions
        indices_map.clear();
        assert(mesh_indices.size() % 3 == 0);
        uint32_t vertex_count = 0;

        dest_shape.indices.resize(mesh_indices.size());
        for (size_t ii = 0; ii < mesh_indices.size(); ii += 3)
        {
            for (int i = 0; i < 3; ++i)
            {
                int vi = mesh_indices[ii + i].vertex_index;
                int rvi = 0;

                if (indices_map.count(vi) == 0)
                {
                    rvi = vertex_count;
                    indices_map[vi] = vertex_count++;
                }
                else
                {
                    rvi = indices_map[vi];
                }

                dest_shape.indices[ii + i] = rvi;
            }
        }

        dest_shape.positions.resize(vertex_count * 3);
        dest_shape.normals.resize(vertex_count * 3);
        
        dest_shape.min_position = vector3_set1(FLT_MAX);
        dest_shape.max_position = vector3_set1(-FLT_MAX);

        // fill positions
        for (std::pair<int, int> p : indices_map)
        {
            int original_index = p.first * 3;
            int reordered_index = p.second * 3;
            float* pos = attrib.vertices.data() + original_index;

            for (int i = 0; i < 3; ++i)
            {
                float pos_i = pos[i];
                dest_shape.positions[reordered_index + i] = pos_i;

                if (pos_i < dest_shape.min_position.v[i])
                    dest_shape.min_position.v[i] = pos_i;

                if (dest_shape.max_position.v[i] < pos_i)
                    dest_shape.max_position.v[i] = pos_i;
            }
        }


        bv_ids.clear();
        bv_ids.resize(mesh_indices.size() / 3);

        // evaluate normals, prepare for bvh
        for (size_t ii = 0; ii < dest_shape.indices.size(); ii += 3)
        {
            int vi[3] =
            {
                dest_shape.indices[ii] * 3 ,
                dest_shape.indices[ii + 1] * 3,
                dest_shape.indices[ii + 2] * 3
            };

            Vector3 p0 = vector3_setp(&dest_shape.positions[vi[0]]);
            Vector3 p1 = vector3_setp(&dest_shape.positions[vi[1]]);
            Vector3 p2 = vector3_setp(&dest_shape.positions[vi[2]]);
            Vector3 normal = vector3_normalize(vector3_cross(vector3_sub(p1, p0), vector3_sub(p2, p0)));

            for (int ni = 0; ni < 3; ++ni)
            {
                dest_shape.normals[vi[ni]] += normal.v[0];
                dest_shape.normals[vi[ni] + 1] += normal.v[1];
                dest_shape.normals[vi[ni] + 2] += normal.v[2];
            }

            bv_ids[ii / 3] = (void*)((int)ii);
        }

        // evalute mesh unit normal
        for (size_t ni = 0; ni < dest_shape.normals.size(); ni += 3)
        {
            float n[3] = { dest_shape.normals[ni], dest_shape.normals[ni + 1], dest_shape.normals[ni + 2] };

            float inv_len = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
            if (inv_len != 0.f)
            {
                inv_len = 1.f / sqrtf(inv_len);

                dest_shape.normals[ni] *= inv_len;
                dest_shape.normals[ni + 1] *= inv_len;
                dest_shape.normals[ni + 2] *= inv_len;
            }
        }

        dest_shape.bvh = bvh_create_from_triangles
        (
            (Vector3*)dest_shape.positions.data(),
            dest_shape.positions.size() / 3,
            (TriangleIndex*)(dest_shape.indices.data()),
            bv_ids.data(),
            dest_shape.indices.size() / 3,
            NULL
        );
    }
}

static void interset_aabb_callback(void* bvh_callback_context, void* callback_context, void* id)
{
    int face_index = (int)id;
    std::vector<int>* out_face_indices = (std::vector<int>*)callback_context;

    out_face_indices->push_back(face_index);
}

void obj_intersect_aabb_with_leaf(ObjShape* shape, AABB aabb, std::vector<int>* out_face_indices)
{
    bvh_query_aabb(shape->bvh, aabb, interset_aabb_callback, out_face_indices);
}

struct MimumDistanceResult
{
    float* out_signed_distance;
    Vector3* out_closest_point;
    int* out_face_index;

    ObjShape* shape;
    Vector3 query_point;
};

static void minimum_squared_distance_callback(void* bvh_callback_context, void* callback_context, void* id)
{
    float* bvh_closest_dist = (float*)bvh_callback_context;
    MimumDistanceResult* mdr = (MimumDistanceResult*)callback_context;
    
    ObjShape* shape = mdr->shape;
    int face_index = (int)id;
    int fi = face_index * 3;

    int tri_indices[3] = 
    { 
        shape->indices[fi] * 3 , 
        shape->indices[fi + 1] * 3, 
        shape->indices[fi + 2] * 3 
    };

    Vector3 tri_verts[3] =
    {
        vector3_setp(&(shape->positions[tri_indices[0]])),
        vector3_setp(&(shape->positions[tri_indices[1]])),
        vector3_setp(&(shape->positions[tri_indices[2]]))
    };

    Vector3 temp_point = triangle_closest_point(mdr->query_point, tri_verts[0], tri_verts[1], tri_verts[2]);
    float temp_dist = vector3_distance_sq(temp_point, mdr->query_point);

    if (temp_dist < *(bvh_closest_dist))
    {
        *bvh_closest_dist = temp_dist;
        *(mdr->out_signed_distance) = temp_dist;
        *(mdr->out_closest_point) = temp_point;
        *(mdr->out_face_index) = face_index;
    }
}

void obj_minimum_squared_distance(ObjShape* shape, Vector3 query_point, float* out_signed_distance, Vector3* out_closest_point, int* out_face_index)
{
    float temp_closest_dist = FLT_MAX;
    Vector3 temp_closest_point;
    int temp_face_index;

    MimumDistanceResult mdr;
    mdr.out_signed_distance = &temp_closest_dist;
    mdr.out_closest_point = &temp_closest_point;
    mdr.out_face_index = &temp_face_index;
    mdr.shape = shape;
    mdr.query_point = query_point;

    bvh_query_point(shape->bvh, query_point, minimum_squared_distance_callback, &mdr);
    if (temp_closest_dist != FLT_MAX)
    {
        *out_signed_distance = temp_closest_dist;
        *out_closest_point = temp_closest_point;
        *out_face_index = temp_face_index;
    }
}