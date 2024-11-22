#include "meshopt_internal.h"

#include <time.h>

#include "common.h"
#include "bvh.h"
#include "geometry_algorithm.h"

struct ClosestFaceCallbackContext
{
    Face* closest_face;
    Vector3 closest_point;

    Vector3* positions;
    TriangleIndex* indices;
    TUnorderedMap<uint32_t, int>* face_id_to_tri_index;

    Vector3 query_point;
};

static void closest_face_callback(void* bvh_callback_context, void* user_callback_context, void* leaf_id)
{
    float* bvh_closest_dist = (float*)bvh_callback_context;
    ClosestFaceCallbackContext* ucc = (ClosestFaceCallbackContext*)user_callback_context;
    Face* f = (Face*)leaf_id;

    TUnorderedMap<uint32_t, int>* fiti = ucc->face_id_to_tri_index;

    TriangleIndex tri_indices = ucc->indices[(*fiti)[f->id]];
    Vector3 tri_verts[3] =
    {
        ucc->positions[tri_indices.v[0]],
        ucc->positions[tri_indices.v[1]],
        ucc->positions[tri_indices.v[2]]
    };

    Vector3 temp_point = triangle_closest_point(ucc->query_point, tri_verts[0], tri_verts[1], tri_verts[2]);
    float temp_dist = vector3_distance_sq(temp_point, ucc->query_point);

    if (temp_dist < *bvh_closest_dist)
    {
        *bvh_closest_dist = temp_dist;
        ucc->closest_face = f;
        ucc->closest_point = temp_point;
    }
}

static void global_project_aux(OptimizeContext* oc)
{
    Mesh* mesh = oc->target_mesh;
    PoolAllocator* allocator = mesh->allocator;

    TUnorderedMap<uint32_t, int> vertex_id_to_pos_index(allocator);
    TUnorderedMap<uint32_t, int> face_id_to_tri_index(allocator);
    vertex_id_to_pos_index.reserve(mesh->vertex_map.size());
    face_id_to_tri_index.reserve(mesh->face_map.size());

    Vector3* positions = (Vector3*)allocator->Alloc(sizeof(Vector3) * mesh->vertex_map.size());
    TriangleIndex* triangle_indices = (TriangleIndex*)allocator->Alloc(sizeof(TriangleIndex) * mesh->face_map.size());
    Face** ids = (Face**)allocator->Alloc(sizeof(Face*) * mesh->face_map.size());

    int position_count = 0;
    for (TUnorderedMap<uint32_t, Vertex*>::iterator it = mesh->vertex_map.begin();
        it != mesh->vertex_map.end();
        ++it)
    {
        Vertex* v = it->second;

        vertex_id_to_pos_index[v->id] = position_count;
        positions[position_count] = v->point;

        ++position_count;
    }

    int index_count = 0;
    for (TUnorderedMap<uint32_t, Face*>::iterator it = mesh->face_map.begin();
        it != mesh->face_map.end();
        ++it)
    {
        Face* f = it->second;
        assert(face_is_triangle(f) == true);

        face_id_to_tri_index[f->id] = index_count;

        ids[index_count] = f;

        Vertex* va[3];
        face_get_triangle_vertices(f, va);

        triangle_indices[index_count].v[0] = vertex_id_to_pos_index[va[0]->id];
        triangle_indices[index_count].v[1] = vertex_id_to_pos_index[va[1]->id];
        triangle_indices[index_count].v[2] = vertex_id_to_pos_index[va[2]->id];
        ++index_count;
    }

    BVH* bvh = bvh_create_from_triangles
    (
        positions,
        position_count,
        triangle_indices,
        (void**)ids,
        index_count,
        allocator
    );

    ClosestFaceCallbackContext cfcc;
    cfcc.positions = positions;
    cfcc.indices = triangle_indices;
    cfcc.face_id_to_tri_index = &face_id_to_tri_index;
    for (int i = 0; i < (int)oc->data_points.size(); ++i)
    {
        cfcc.closest_face = NULL;
        cfcc.query_point = oc->data_points[i];

        bvh_query_point(bvh, cfcc.query_point, closest_face_callback, &cfcc);
        moi_change_data_point_face(oc, cfcc.closest_face, i);
        oc->data_projected_points[i] = cfcc.closest_point;
    }

    bvh_destroy(bvh);
    allocator->Free(ids);
    allocator->Free(triangle_indices);
    allocator->Free(positions);
}

struct ProjectNeighborResult
{
    Face* closest_face;
    Vector3 closest_point;
};

static ProjectNeighborResult project_point_neighbor
(
    OptimizeContext* oc,
    Face* current_face,
    Vector3 query_point,
    TSet<uint32_t>& setfvis,
    TSet<uint32_t>& setf
)
{
    Vertex* va[3];
    TriangleClosestPointResult tcpr;
    ProjectNeighborResult result;

    face_get_triangle_vertices(current_face, va);
    triangle_closest_point_detail(query_point, va[0]->point, va[1]->point, va[2]->point, &tcpr);

    result.closest_face = current_face;
    result.closest_point = tcpr.closest_point;
    float closest_dist_sq = tcpr.dist_sq;

    setfvis.clear();
    setfvis.insert(current_face->id);

    for (;;)
    {
        setf.clear();

        face_get_triangle_vertices(result.closest_face, va);
        for (int j = 0; j < 3; ++j)
        {
            for (HalfEdge* he : va[j]->arhe)
            {
                Face* vertex_face = he->face;
                if (setfvis.count(vertex_face->id) == 0)
                {
                    setf.insert(vertex_face->id);

                    face_get_triangle_vertices(vertex_face, va);
                    Vector3 cp = triangle_closest_point(query_point, va[0]->point, va[1]->point, va[2]->point);
                    float dist_sq = vector3_distance_sq(query_point, cp);

                    if (dist_sq < closest_dist_sq)
                    {
                        result.closest_face = vertex_face;
                        result.closest_point = cp;
                        closest_dist_sq = dist_sq;
                    }
                }
            }
        }

        if (setfvis.count(result.closest_face->id) > 0)
            break;

        for (uint32_t f_id : setf)
        {
            setfvis.insert(f_id);
        }
    }

    return result;
}

void local_project_aux(OptimizeContext* oc)
{
    PoolAllocator* allocator = oc->target_mesh->allocator;
    TSet<uint32_t> setfvis(allocator);
    TSet<uint32_t> setf(allocator);

    for (int i = 0; i < (int)oc->data_points.size(); ++i)
    {
        Vector3 query_point = oc->data_points[i];
        Face* f = oc->data_faces[i];

        assert(f != NULL);

        ProjectNeighborResult pnr = project_point_neighbor(oc, f, query_point, setfvis, setf);
        moi_change_data_point_face(oc, pnr.closest_face, i);
        oc->data_projected_points[i] = pnr.closest_point;
    }
}

void moi_global_projection(OptimizeContext* oc)
{
    /*
    * This function assumes that the topology of the target mesh changes with the algorithm.
    * So, If a data_face is NULL, we need to look for the data faces from scratch.
    */
    bool do_global_project = false;
    for (Face* f : oc->data_faces)
    {
        if (f == NULL)
        {
            do_global_project = true;
            break;
        }
    }

    if (do_global_project)
    {
        time_t t = clock();

        global_project_aux(oc);

        t = clock() - t;
        printf("Global Projection %f secs for %d vertices and %d data points\n", 
            (float)t / CLOCKS_PER_SEC, oc->target_mesh->num_vertex, (int)oc->data_points.size());
    }
    else // local_project_aux
    {
        time_t t = clock();

        local_project_aux(oc);

        t = clock() - t;

        printf("Local Projection %f secs for %d vertices and %d data points\n",
            (float)t / CLOCKS_PER_SEC, oc->target_mesh->num_vertex, (int)oc->data_points.size());
    }
}

void moi_reproject_locally
(
    OptimizeContext* oc,
    const TVector<int>& point_indices,
    const TVector<Face*>& faces
)
{
    if (point_indices.size() == 0)
        return;

    PoolAllocator* allocator = oc->target_mesh->allocator;

    int number_faces = (int)faces.size();
    AABB* boxes = (AABB*)allocator->Alloc(sizeof(AABB) * number_faces);
    float* box_distance_sqs = (float*)allocator->Alloc(sizeof(float) * number_faces);
    Vertex* va[3];

    for (int fi = 0; fi < number_faces; ++fi)
    {
        Face* f = faces[fi];
        AABB* aabb = &(boxes[fi]);

        aabb_set_infinity(aabb);

        HalfEdge* he = f->herep;
        HalfEdge* he0 = he;
        do
        {
            Vertex* v = he->vert;
            aabb_combine_float(aabb, v->point.v);

            he = he->next;
        } while (he != he0);
    }

    for (int point_index : point_indices)
    {
        Vector3 data_point = oc->data_points[point_index];

        for (int fi = 0; fi < number_faces; ++fi)
        {
            box_distance_sqs[fi] = aabb_distance_exterior_sq_point(&(boxes[fi]), data_point);
        }

        float min_distance_tri = FLT_MAX;
        Face* min_face = NULL;
        for (;;)
        {
            int min_distance_index = moi_get_min_value_index(box_distance_sqs, number_faces);
            float min_distance_sq = box_distance_sqs[min_distance_index];
            if (min_distance_sq == MOI_BIG_FLOAT) break;
            if (min_distance_sq >= min_distance_tri) break;
            box_distance_sqs[min_distance_index] = MOI_BIG_FLOAT;
            Face* f = faces[min_distance_index];
            face_get_triangle_vertices(f, va);

            Vector3 closest_point = triangle_closest_point(data_point, va[0]->point, va[1]->point, va[2]->point);
            float dist = vector3_distance_sq(data_point, closest_point);
            if (dist < min_distance_tri)
            {
                min_distance_tri = dist;
                min_face = f;
                oc->data_projected_points[point_index] = closest_point;
            }
        }

        moi_change_data_point_face(oc, min_face, point_index);
    }

    allocator->Free(box_distance_sqs);
    allocator->Free(boxes);
}

void moi_cleanup_neighborhood(OptimizeContext* oc, Vertex* v, int nri)
{
    if (nri)
    {
        VertexIteration vi = vertex_get_iteration(v);
        while (Vertex* w = vertex_iterate(&vi))
        {
            moi_local_fit_ring(oc, w, nri);
        }
        moi_local_fit_ring(oc, v, nri);
    }
}