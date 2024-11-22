#include "meshopt_internal.h"

#include "random.h"
#include "geometry_algorithm.h"

enum EdgeOperationResult
{
    EDGE_OP_SUCCESS,
    EDGE_OP_FAIL
};

/*
static inline void debug_vertex_neighbors(Vertex* v)
{
    const float LENGTH_THRESHOLD = 0.1f;
    VertexIteration vi = vertex_get_iteration(v);
    while (Vertex* vv = vertex_iterate(&vi))
    {
        float dist = vector3_distance_sq(v->point, vv->point);
        if (dist > LENGTH_THRESHOLD)
        {
            printf("Weird Vertex! %d from %d\n", vv->id, v->id);
        }
    }
}

static inline void debug_export_if_success(OptimizeContext* oc, EdgeOperation op, EdgeOperationResult r)
{
    if (r == EDGE_OP_SUCCESS)
    {
        PoolString str(oc->target_mesh->allocator);
        int index = oc->target_mesh->num_edges - oc->edge_candidates.size();

        str.AssignSPrintf("debug/%dm_", index);
        switch (op)
        {
        case EDGE_OP_COLLAPSE:
            str.Append("collapse");
            break;
        case EDGE_OP_SPLIT:
            str.Append("split");
            break;
        case EDGE_OP_SWAP:
            str.Append("swap");
            break;
        default:
            break;
        }
        str.AppendSPrintf(".obj");

        mesh_export_obj(oc->target_mesh, str.data());
    }
}
*/

static EdgeOperationResult try_edge_collapse
(
    Edge* edge, 
    OptimizeContext* oc, 
    int local_fit_iteration,
    int clean_up_iteration
)
{
    Mesh* m = oc->target_mesh;
    if (mesh_is_nice_edge_collapse(m, edge) == false)
        return EDGE_OP_FAIL;

    Vertex* v1 = edge_get_vertex1(edge);
    Vertex* v2 = edge_get_vertex2(edge);
    Face* f1 = edge_get_face1(edge);
    Face* f2 = edge_get_face2(edge);

    if (edge_is_sharp(edge) == false)
    {
        if (vertex_get_num_sharp_edges(v1) >= 1 && vertex_get_num_sharp_edges(v2) >= 1)
        {
            if (oc->verbose_warning)
                printf("Edge Collapse would offen sharp edges (a)\n");

            return EDGE_OP_FAIL;
        }
    }
    else
    {
        if (vertex_get_num_sharp_edges(v1) >= 3 && vertex_get_num_sharp_edges(v2) >= 3)
        {
            if (oc->verbose_warning)
                printf("Edge Collapse would offen sharp edges (b)\n");

            return EDGE_OP_FAIL;
        }

        Face* edge_faces[2];
        edge_get_both_faces(edge, edge_faces);
        for (int i = 0; i < 2; ++i)
        {
            if (edge_faces[i] == NULL)
                continue;

            Vertex* vs = vertex_get_opposite_from_ef(edge, edge_faces[i]);
            if (edge_is_sharp(edge_query(v1, vs)) == true && edge_is_sharp(edge_query(v2, vs)) == true)
            {
                if (oc->verbose_warning)
                    printf("Edge collapse would offen sharp edges (c)\n");

                return EDGE_OP_FAIL;
            }
        }
    }

    TVector<Vertex*> edge_rings(m->allocator);
    edge_get_ring(edge, edge_rings);

    int num_boundary_vertex = (int)vertex_is_boundary(v1) + (int)vertex_is_boundary(v2);
    float minb = std::min(mesh_get_min_dihedral_about_vertex(m, v1), mesh_get_min_dihedral_about_vertex(m, v2));
    double rssf = 0.0;

    TVector<Face*> ar_faces(m->allocator); // faces around the vertex v1 and v2 except for f1 and f2
    TVector<int> ar_pts(m->allocator); // data point indices around the above faces
    for (HalfEdge* he : v1->arhe)
    {
        Face* f = he->face;
        TUnorderedSet<int>& fp_set = oc->face_point_map[f->id];
        for (int pi : fp_set)
            ar_pts.push_back(pi);

        if (f == f1 || f == f2)
            continue;
        ar_faces.push_back(f);
    }

    for (HalfEdge* he : v2->arhe)
    {
        Face* f = he->face;
        if (f == f1 || f == f2)
            continue;

        TUnorderedSet<int>& fp_set = oc->face_point_map[f->id];
        for (int pi : fp_set)
            ar_pts.push_back(pi);

        ar_faces.push_back(f);
    }

    for (int pi : ar_pts)
        rssf += vector3_distance_sq(oc->data_points[pi], oc->data_projected_points[pi]);

    VertexIteration vi = vertex_get_iteration(v1);
    while (Vertex* v = vertex_iterate(&vi))
        rssf += moi_get_spring_energy_vv(oc, v1, v);

    vi = vertex_get_iteration(v2);
    while (Vertex* v = vertex_iterate(&vi))
        if (v != v1)
            rssf += moi_get_spring_energy_vv(oc, v2, v);

    // find the best collapsing position between v1 and v2
    double minrss1 = MOI_BIG_FLOAT;
    int minii = -1;
    Vector3 minp;
    for (int ii = 0; ii < 3; ++ii)
    {
        if (edge_is_sharp(edge) == false)
        {
            if (vertex_get_num_sharp_edges(v1) >= 1 && ii < 2) 
                continue;

            if (vertex_get_num_sharp_edges(v2) >= 1 && ii > 0)
                continue;
        }
        else
        {
            if (vertex_get_num_sharp_edges(v1) >= 3 && ii < 2)
                continue;

            if (vertex_get_num_sharp_edges(v2) >= 3 && ii > 0)
                continue;
        }

        float alpha = ii * 0.5f;
        Vector3 new_p = vector3_add
        (
            vector3_mul_scalar(v1->point, alpha),
            vector3_mul_scalar(v2->point, 1.f - alpha)
        );


        float mina = moi_get_minimum_local_dihedral(edge_rings, new_p);
        if (mina < MOI_MIN_COS && mina < minb)
            continue; // change disallowed

        LocalFitOptimizationResult local_fit = moi_local_fit_optimization
        (
            oc,
            ar_pts,
            edge_rings,
            1,
            new_p
        );

        mina = moi_get_minimum_local_dihedral(edge_rings, local_fit.new_fitted_pos);
        if (mina < MOI_MIN_COS && mina < minb)
            continue;

        if (local_fit.energy_after_refit < minrss1)
        {
            minrss1 = local_fit.energy_after_refit;
            minii = ii;
            minp = local_fit.new_fitted_pos;
        }
    }

    if (minii < 0)
    {
        if (oc->verbose_warning)
            printf("No Dihedrally admissible configuration\n");
            
        return EDGE_OP_FAIL;
    }

    float alpha = minii * 0.5f;
    
    if (local_fit_iteration)
    {
        LocalFitOptimizationResult local_fit = moi_local_fit_optimization
        (
            oc,
            ar_pts,
            edge_rings,
            local_fit_iteration,
            minp
        );
        float mina = moi_get_minimum_local_dihedral(edge_rings, local_fit.new_fitted_pos);
        if (mina < MOI_MIN_COS && mina < minb)
            return EDGE_OP_FAIL; // change disallowed

        minrss1 = local_fit.energy_after_refit;
        minp = local_fit.new_fitted_pos;
    }
    double drss = minrss1 - rssf - (num_boundary_vertex == 2 ? oc->crep_boundary_factor : 1) * double(oc->crep);
    if (drss >= 0)
        return EDGE_OP_FAIL; // energy function does not decrease
    if (oc->verbose_warning)
        printf("Edge Collapse : %f %f %f\n", rssf, minrss1, drss);

    Vertex* edge_vertices[2];
    edge_get_both_vertices(edge, edge_vertices);
    for (int i = 0; i < 2; ++i)
    {
        Vertex* v = edge_vertices[i];

        EdgeVertexIteration evi = edge_vertex_get_iteration(v);
        while (Edge* e = edge_vertex_iterate(&evi))
        {
            oc->edge_candidates.erase(moi_get_hash_edge(e));
        }
    }
    moi_remove_data_point_face(oc, f1);
    if (f2 != NULL)
        moi_remove_data_point_face(oc, f2);

    mesh_collapse_edge(m, edge); // v1 kept

    // add about 12 to 16 edges
    EdgeVertexIteration evi = edge_vertex_get_iteration(v1);
    while (Edge* e = edge_vertex_iterate(&evi))
    {
        oc->edge_candidates[moi_get_hash_edge(e)] = e;
    }

    for (HalfEdge* he : v1->arhe)
    {
        Face* f = he->face;
        Edge* e = edge_get_opposite_from_vf(v1, f);
        oc->edge_candidates[moi_get_hash_edge(e)] = e;
    }

    v1->point = minp;
    moi_reproject_locally(oc, ar_pts, ar_faces);
    moi_cleanup_neighborhood(oc, v1, clean_up_iteration);

    return EDGE_OP_SUCCESS;
}

static EdgeOperationResult try_edge_split
(
    Edge* edge,
    OptimizeContext* oc,
    int local_fit_iteration,
    int clean_up_iteration
)
{
    Mesh* m = oc->target_mesh;

    Vertex* v1 = edge_get_vertex1(edge);
    Vertex* v2 = edge_get_vertex2(edge);
    Vertex* vo1 = edge_get_side_vertex1(edge);
    Vertex* vo2 = edge_get_side_vertex2(edge);

    TVector<Vertex*> rings(m->allocator);
    rings.reserve(4);
    rings.push_back(v2);
    rings.push_back(vo1);
    rings.push_back(v1);
    if (vo2)
    {
        rings.push_back(vo2);
        rings.push_back(v2);
    }
    float minb = vo2 ? moi_get_edge_dihedral(edge) : 2.f;
    double rssf = moi_get_spring_energy_edge(oc, edge);

    TVector<int> ar_pts(m->allocator);
    TVector<Face*> edge_faces;
    edge_get_faces(edge, edge_faces);
    for (Face* f : edge_faces)
    {
        TUnorderedSet<int>& fp_set = oc->face_point_map[f->id];
        for (int pi : fp_set)
            ar_pts.push_back(pi);
    }

    for (int point_index : ar_pts)
        rssf += vector3_distance_sq(oc->data_points[point_index], oc->data_projected_points[point_index]);

    Vector3 newp = vector3_add
    (
        vector3_mul_scalar(rings[0]->point, 0.5f),
        vector3_mul_scalar(rings[2]->point, 0.5f)
    );

    LocalFitOptimizationResult local_fit = moi_local_fit_optimization
    (
        oc,
        ar_pts,
        rings,
        (local_fit_iteration ? local_fit_iteration : 1),
        newp
    );
    float mina = moi_get_minimum_local_dihedral(rings, local_fit.new_fitted_pos);
    if (mina < MOI_MIN_COS && mina < minb)
        return EDGE_OP_FAIL; // change disallowed
    double drss = local_fit.energy_after_refit - rssf + (vo2 ? 1.f : oc->crep_boundary_factor) * double(oc->crep);

    if (drss >= 0)
        return EDGE_OP_FAIL; // energy function does not decrease
    if (oc->verbose_warning)
        printf("Edge Split : %f %f %f\n", rssf, local_fit.energy_after_refit, drss);

    for (Face* f : edge_faces)
    {
        EdgeFaceIteration efi = edge_face_get_iteration(f);
        while (Edge* e = edge_face_iterate(&efi))
        {
            oc->edge_candidates.erase(moi_get_hash_edge(e));
        }

        moi_remove_data_point_face(oc, f);
    }

    Vertex* v = mesh_split_edge(m, edge);
    v->point = local_fit.new_fitted_pos;

    // add 8 edges (5 if boundary)
    for (HalfEdge* he : v->arhe)
    {
        Face* f = he->face;
        EdgeFaceIteration efi = edge_face_get_iteration(f);
        while (Edge* e = edge_face_iterate(&efi))
        {
            oc->edge_candidates[moi_get_hash_edge(e)] = e;
        }
    }

    // since ar_pts project onto f1 + f2 (which are still there), it is easy to update the projections:
    moi_local_fit_ring(oc, v, 2);
    moi_cleanup_neighborhood(oc, v, clean_up_iteration);

    return EDGE_OP_SUCCESS;
}

/*
* Correspondences on Edge e such as vertex1(e) == v1 may fail here!
* Try swapping edge (v1, v2) into edge (vo1, vo2), allowing vertex vo1 to move.
* To do this, gather points in current ring of vo1 plus points on f2.
*/
static EdgeOperationResult check_half_edge_swap
(
    Edge* edge,
    OptimizeContext* oc,
    Vertex* vo1,
    Vertex* v1,
    Vertex* vo2,
    Vertex* v2,
    Face* f2,
    Face* f1,
    int local_fit_iteration,
    int clean_up_iteration
)
{
    Mesh* m = oc->target_mesh;

    TVector<Vertex*> rings(m->allocator);
    assert(vertex_get_ccw(vo1, v1) == v2);
    Vertex* w = vertex_get_most_clw(vo1);
    Vertex* wf = w;
    for (;;)
    {
        rings.push_back(w);
        if (w == v1)
            rings.push_back(vo2);
        w = vertex_get_ccw(vo1, w);
        if (w == NULL || w == wf)
            break;
    }
    if (w)
        rings.push_back(w);

    TVector<int> ar_pts(m->allocator);
    TVector<Face*> ar_faces(m->allocator);
    for (HalfEdge* he : vo1->arhe)
    {
        Face* f = he->face;
        if (f == f1)
            continue;
        assert(f != f2);
        ar_faces.push_back(f);

        TUnorderedSet<int>& fp_set = oc->face_point_map[f->id];
        for (int pi : fp_set)
            ar_pts.push_back(pi);
    }
    {
        TUnorderedSet<int>& fp_set = oc->face_point_map[f1->id];
        for (int pi : fp_set)
            ar_pts.push_back(pi);
    }
    {
        TUnorderedSet<int>& fp_set = oc->face_point_map[f2->id];
        for (int pi : fp_set)
            ar_pts.push_back(pi);
    }

    double rssf = 0.0;
    for (int point_index : ar_pts)
        rssf += vector3_distance_sq(oc->data_points[point_index], oc->data_projected_points[point_index]);
    
    VertexIteration vi = vertex_get_iteration(vo1);
    while (Vertex* vv = vertex_iterate(&vi))
    {
        rssf += moi_get_spring_energy_vv(oc, vo1, vv);
    }

    Vector3 newp = vo1->point;
    float minb = std::min(mesh_get_min_dihedral_about_vertex(m, vo1), moi_get_edge_dihedral(edge));

    LocalFitOptimizationResult local_fit = moi_local_fit_optimization
    (
        oc,
        ar_pts,
        rings,
        (local_fit_iteration ? local_fit_iteration : 1),
        newp
    );
    float mina = moi_get_minimum_local_dihedral(rings, local_fit.new_fitted_pos);
    if (mina < MOI_MIN_COS && mina < minb)
        return EDGE_OP_FAIL; // change disallowed

    double drss = local_fit.energy_after_refit - rssf + oc->crep * oc->edge_swap_sym_factor;
    if (drss > 0)
        return EDGE_OP_FAIL;
    if(oc->verbose_warning)
        printf("Edge Swap : %f %f %f\n", rssf, local_fit.energy_after_refit, drss);

    /*
    * TODO : get string info??
    */

    oc->edge_candidates.erase(moi_get_hash_edge(edge));
    moi_remove_data_point_face(oc, f1);
    moi_remove_data_point_face(oc, f2);
    Edge* new_edge = mesh_swap_edge(m, edge);
    vo1->point = local_fit.new_fitted_pos;

    // add about 9 edges
    Edge* vo2v1 = edge_query(vo2, v1);
    oc->edge_candidates[moi_get_hash_edge(vo2v1)] = vo2v1;

    Edge* vo2v2 = edge_query(vo2, v2);
    oc->edge_candidates[moi_get_hash_edge(vo2v2)] = vo2v2;

    EdgeVertexIteration evi = edge_vertex_get_iteration(vo1);
    while (Edge* e = edge_vertex_iterate(&evi))
    {
        oc->edge_candidates[moi_get_hash_edge(e)] = e;
    }

    Face* edge_faces[2];
    edge_get_both_faces(new_edge, edge_faces);
    for (int i = 0; i < 2; ++i)
    {
        if (edge_faces[i] == NULL)
            continue;

        ar_faces.push_back(edge_faces[i]);
    }

    moi_reproject_locally(oc, ar_pts, ar_faces);

    if (clean_up_iteration)
    {
        moi_local_fit_ring(oc, vo2, clean_up_iteration);
        moi_local_fit_ring(oc, v1, clean_up_iteration);
        moi_local_fit_ring(oc, v2, clean_up_iteration);
        moi_local_fit_ring(oc, vo1, clean_up_iteration);
        moi_cleanup_neighborhood(oc, vo1, clean_up_iteration);
        moi_local_fit_ring(oc, vo2, clean_up_iteration);
        moi_local_fit_ring(oc, v1, clean_up_iteration);
        moi_local_fit_ring(oc, v2, clean_up_iteration);
        moi_local_fit_ring(oc, vo1, clean_up_iteration);
    }

    return EDGE_OP_SUCCESS;
}

static EdgeOperationResult try_edge_swap
(
    Edge* edge,
    OptimizeContext* oc,
    RandomGen* rg,
    int local_fit_iteration,
    int clean_up_iteration
)
{
    Mesh* m = oc->target_mesh;

    if (mesh_is_legal_edge_swap(m, edge) == false)
        return EDGE_OP_FAIL;

    if (edge_is_sharp(edge) == true)
        return EDGE_OP_FAIL;

    Vertex* v1 = edge_get_vertex1(edge);
    Vertex* v2 = edge_get_vertex2(edge);
    Face* f1 = edge_get_face1(edge);
    Face* f2 = edge_get_face2(edge);
    Vertex* vo1 = edge_get_side_vertex1(edge);
    Vertex* vo2 = edge_get_side_vertex2(edge);

    // compare angles immediately before and after swap
    float minb = moi_get_edge_dihedral(edge);
    float mina = dihedral_angle_cos(vo1->point, vo2->point, v1->point, v2->point);
    if (mina < MOI_MIN_COS && mina < minb)
        return EDGE_OP_FAIL;

    // Will try both cases, but randomly select which one to try first.
    EdgeOperationResult result;
    float random = randomgen_get_uniform(rg); // 0.0 ~ 1.0
    if (random < 0.5f)
    {
        result = check_half_edge_swap
        (
            edge, oc,
            vo1, v1, vo2, v2,
            f2, f1,
            local_fit_iteration, clean_up_iteration
        );

        if (result == EDGE_OP_SUCCESS)
            return result;

        result = check_half_edge_swap
        (
            edge, oc,
            vo2, v2, vo1, v1,
            f1, f2,
            local_fit_iteration, clean_up_iteration
        );
    }
    else
    {
        result = check_half_edge_swap
        (
            edge, oc,
            vo2, v2, vo1, v1,
            f1, f2,
            local_fit_iteration, clean_up_iteration
        );

        if (result == EDGE_OP_SUCCESS)
            return result;

        result = check_half_edge_swap
        (
            edge, oc,
            vo1, v1, vo2, v2,
            f2, f1,
            local_fit_iteration, clean_up_iteration
        );
    }    

    return result;
}

enum EdgeOperation
{
    EDGE_OP_COLLAPSE,
    EDGE_OP_SPLIT,
    EDGE_OP_SWAP
};

static EdgeOperationResult try_edge_operation(Edge* e, OptimizeContext* oc, EdgeOperation op, RandomGen* rg)
{
    EdgeOperationResult result;

    switch (op)
    {
    case EDGE_OP_COLLAPSE:
    {
        result = try_edge_collapse
        (
            e,
            oc,
            (int)(4.f * oc->edge_op_iter_factor + .5f),
            (int)(2.f * oc->edge_op_iter_factor + .5f)
        );
        break;
    }
    case EDGE_OP_SPLIT:
    {
        result = try_edge_split
        (
            e,
            oc,
            (int)(3.f * oc->edge_op_iter_factor + .5f),
            (int)(4.f * oc->edge_op_iter_factor + .5f)
        );
        break;
    }
    case EDGE_OP_SWAP:
    {
        result = try_edge_swap
        (
            e,
            oc,
            rg,
            (int)(3.f * oc->edge_op_iter_factor + .5f),
            (int)(2.f * oc->edge_op_iter_factor + .5f)
        );
        break;
    }
    default:
    {
        assert(0);
        break;
    }
    }

    return result;
}

void moi_simplicies_fit(OptimizeContext* oc)
{
    Mesh* m = oc->target_mesh;
    TVector<uint32_t> edge_linears(m->allocator);

    unsigned seed = 20230427;
    RandomGen* rg = randomgen_create(m->allocator, &seed);
    edge_linears.reserve(m->num_edges);
    oc->edge_candidates.reserve(m->num_edges);
    
    // gather edges into the edge candidates
    {
        EdgeIteration ei = edge_get_iteration(m);
        while (Edge* e = edge_iterate(&ei))
        {
            uint32_t hash = moi_get_hash_edge(e);
            edge_linears.push_back(hash);
            oc->edge_candidates[hash] = e;
        }
    }
    
    while (oc->edge_candidates.empty() == false)
    {
        float random_value = randomgen_get_uniform(rg);
        int random_index = (int)(random_value * oc->edge_candidates.size());
        if (random_index == (int)oc->edge_candidates.size())
            random_index = (int)oc->edge_candidates.size() - 1;

        uint32_t hash = edge_linears[random_index];
        
        Edge* edge = oc->edge_candidates[hash];

        // remove the pulled edge from map and vector
        oc->edge_candidates.erase(hash);
        edge_linears[random_index] = edge_linears.back();
        edge_linears.pop_back();
        
        EdgeOperationResult result;
        result = try_edge_operation(edge, oc, EDGE_OP_COLLAPSE, rg);

        if (result != EDGE_OP_SUCCESS && oc->edge_op_iter_factor)
        {
            result = try_edge_operation(edge, oc, EDGE_OP_SPLIT, rg);
        }

        if (result != EDGE_OP_SUCCESS)
        {
            result = try_edge_operation(edge, oc, EDGE_OP_SWAP, rg);
        }

        if (result == EDGE_OP_SUCCESS)
        {
            if (oc->export_every_step)
            {
                moi_export_mesh(oc, oc->export_name, oc->export_step);
                ++oc->export_step;
            }
        }

        // prepare edge_linears again
        edge_linears.clear();
        for (TUnorderedMap<uint32_t, Edge*>::iterator it = oc->edge_candidates.begin();
            it != oc->edge_candidates.end();
            ++it)
        {
            Edge* e = it->second;
            uint32_t hash = moi_get_hash_edge(e);
            edge_linears.push_back(hash);
        }
    }

    randomgen_destroy(rg);
}