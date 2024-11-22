#include "meshopt_internal.h"

#include "geometry_algorithm.h"

void moi_change_data_point_face(OptimizeContext* oc, Face* new_f, int point_index)
{
    Face* old_f = oc->data_faces[point_index];
    if (old_f == new_f)
    {
        return;
    }
    if (old_f != NULL)
    {
        oc->face_point_map[old_f->id].erase(point_index);
    }

    oc->data_faces[point_index] = new_f;

    if (new_f != NULL)
    {
        if (oc->face_point_map.count(new_f->id) == 0)
        {
            /*
            * I don't know why
            * `oc->face_point_map[new_f->id] = TUnordered_set(oc->target_mesh->allocator);`
            * does not work. But the below line works.
            */
            TUnorderedSet<int> initial_set_with_allocator(oc->target_mesh->allocator);
            oc->face_point_map[new_f->id] = initial_set_with_allocator;
        }

        oc->face_point_map[new_f->id].insert(point_index);
    }
}

/*
* make face's point project nowhere and remove from point data
*/
void moi_remove_data_point_face(OptimizeContext* oc, Face* f)
{
    if (oc->face_point_map.count(f->id) == 0)
        return;

    TUnorderedSet<int>& point_sets = oc->face_point_map[f->id];
    for (int point_index : point_sets)
        oc->data_faces[point_index] = NULL;
    oc->face_point_map.erase(f->id);
}

float moi_get_minimum_local_dihedral(const TVector<Vertex*>& wa, Vector3 new_p)
{
    int nw = (int)wa.size();
    const bool is_open = wa[0] != wa[nw - 1];

    float min_local_dihedral = 2.f;
    for (int i = 1; i < nw - is_open; ++i)
    {
        int i1 = i + 1;
        if (i1 == nw) i1 = 1;

        float local_dihedral = dihedral_angle_cos(new_p, wa[i]->point, wa[i - 1]->point, wa[i1]->point);
        if (local_dihedral < min_local_dihedral)
            min_local_dihedral = local_dihedral;
    }

    return min_local_dihedral;
} 

float moi_get_edge_dihedral(Edge* e)
{
    Vertex* v1 = edge_get_vertex1(e);
    Vertex* v2 = edge_get_vertex2(e);
    Face* f1 = edge_get_face1(e);
    Face* f2 = edge_get_face2(e);
    if (face_is_triangle(f1) && face_is_triangle(f2))
    {
        return dihedral_angle_cos
        (
            v1->point, v2->point,
            edge_get_side_vertex1(e)->point, edge_get_side_vertex2(e)->point
        );
    }
    else
    {
        Vertex* sv12 = vertex_get_ccw_from_fv(f1, v2);
        Vertex* sv11 = vertex_get_ccw_from_fv(f1, v1);
        Vertex* sv21 = vertex_get_ccw_from_fv(f2, v1);
        Vertex* sv22 = vertex_get_ccw_from_fv(f2, v2);
        return dihedral_angle_cos
        (
            v1->point, v2->point,
            vector3_add
            (
                vector3_mul_scalar(sv12->point, 0.5f),
                vector3_mul_scalar(sv11->point, 0.5f)
            ),
            vector3_add
            (
                vector3_mul_scalar(sv21->point, 0.5f),
                vector3_mul_scalar(sv22->point, 0.5f)
            )
        );
    }
}

int moi_get_min_value_index(const float* arr, int count)
{
    if (arr == NULL || count <= 0)
        return -1;

    int idx = 0;
    float f = arr[0];

    for (int i = 1; i < count; ++i)
    {
        float v = arr[i];
        if (v < f)
        {
            f = v;
            idx = i;
        }
    }

    return (int)idx;
}

uint32_t moi_get_hash_edge(Edge* e)
{
    uint32_t v1_id = e->herep->prev->vert->id;
    uint32_t v2_id = e->herep->vert->id;
    uint32_t hash = v1_id + v2_id * 76541;
    hash ^= 755324125;
    return hash;
}

float moi_get_spring_energy_vv(OptimizeContext* oc, Vertex* v1, Vertex* v2)
{
    float energy = vector3_distance_sq(v1->point, v2->point);
    energy *= edge_is_boundary(edge_query(v1, v2)) == true ? oc->kappa * oc->kappa_boundary_factor : oc->kappa;
    return energy;
}

float moi_get_spring_energy_edge(OptimizeContext* oc, Edge* e)
{
    float energy = vector3_distance_sq(edge_get_vertex1(e)->point, edge_get_vertex2(e)->point);
    energy *= edge_is_boundary(e) == true ? oc->kappa * oc->kappa_boundary_factor : oc->kappa;
    return energy;
}

void moi_transform_mesh(OptimizeContext* oc, Matrix4* transform)
{
    Mesh* m = oc->target_mesh;
    for (TUnorderedMap<uint32_t, Vertex*>::iterator it = m->vertex_map.begin();
        it != m->vertex_map.end();
        ++it)
    {
        Vertex* v = it->second;
        v->point = matrix4_mul_vec3(transform, v->point);
    }
}

void moi_export_mesh(OptimizeContext* oc, const char* name, int index)
{
    Mesh* mesh = oc->target_mesh;

    PoolString str(mesh->allocator);

    if(index >=0)
        str.AssignSPrintf("%s/%s%d.obj", oc->export_folder.data(), name, index);
    else
        str.AssignSPrintf("%s/%s.obj", oc->export_folder.data(), name);

    moi_transform_mesh(oc, &(oc->transform_to_original));

    mesh_export_obj(mesh, str.data());

    moi_transform_mesh(oc, &(oc->transform_to_unit));
}