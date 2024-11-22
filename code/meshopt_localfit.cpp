#include "meshopt_internal.h"
#include "aabb.h"
#include "geometry_algorithm.h"

struct PointLinearLeastSquares
{
    OptimizeContext* oc;
    Vector3* point;
    double utu[3];
    double utb[3];
    double btb[3];
    double rss0;

    PointLinearLeastSquares(OptimizeContext* oc, Vector3* point)
        : oc(oc)
        , point(point)
        , utu{ 0.0, 0.0, 0.0 }
        , utb{ 0.0, 0.0, 0.0 }
        , btb{ 0.0, 0.0, 0.0 }
        , rss0(0.0)
    {}

    void EnterProjection(Vector3 data_point, Vector3 tri_p1, Vector3 tri_p2, float bary_v, float bary_w)
    {
        double bary_u = 1.0 - bary_v - bary_w;
        for (int i = 0; i < 3; ++i)
        {
            double b = data_point.v[i] - bary_v * tri_p1.v[i] - bary_w * tri_p2.v[i];
            utu[i] += bary_u * bary_u;
            utb[i] += bary_u * b;
            btb[i] += b * b;

            double rss = bary_u * (*point).v[i] - b;
            rss0 += rss * rss;
        }
    }

    void EnterSpring(Vector3 ring_point, float sqrt_tension)
    {
        for (int i = 0; i < 3; ++i)
        {
            double b = double(ring_point.v[i]) * sqrt_tension;
            utu[i] += sqrt_tension * sqrt_tension;
            utb[i] += sqrt_tension * b;
            btb[i] += b * b;

            double rss = sqrt_tension * (*point).v[i] - b;
            rss0 += rss * rss;
        }
    }

    double Solve()
    {
        double rss1 = 0;
        for (int i = 0; i < 3; ++i)
        {
            double new_v = utu[i] != 0.f ? utb[i] / utu[i] : (*point).v[i];
            (*point).v[i] = new_v;

            double a = btb[i] - utu[i] * new_v * new_v;
            if (a > 0.0)
                rss1 += a;
        }

        /*
        if (oc->verbose_warning == true && rss1 - rss0 >= 1e-13)
        {
            printf("Too much different RSS\n");
        }
        */

        return rss1;
    }
};

LocalFitOptimizationResult moi_local_fit_optimization
(
    OptimizeContext* oc,
    const TVector<int>& data_point_indices,
    const TVector<Vertex*>& ring_points,
    int iteration,
    Vector3 initial_point
)
{
    LocalFitOptimizationResult result;
    result.new_fitted_pos = initial_point;
    result.energy_after_first_projection = 0.f;
    result.energy_after_refit = 0.f;

    if (data_point_indices.size() == 0 && oc->kappa == 0.f)
        return result;

    int number_ring_points = (int)ring_points.size();
    bool is_closed = ring_points[0] == ring_points[number_ring_points - 1];
    float sqrt_kappa = sqrtf(oc->kappa);
    float sqrt_kappa_boundary = sqrtf(oc->kappa * oc->kappa_boundary_factor);

    PoolAllocator* allocator = oc->target_mesh->allocator;
    AABB* bbox = (AABB*)allocator->Alloc(sizeof(AABB) * (number_ring_points - 1));
    float* ring_distance_sqs = (float*)allocator->Alloc(sizeof(float) * (number_ring_points - 1));
    double rss1 = 0.0;

    // each box contains the new fitted position and one ring point.
    for (int rpi = 0; rpi < number_ring_points - 1; ++rpi)
    {
        AABB* box = bbox + rpi;
        aabb_set_infinity(box);
        aabb_combine_float(box, result.new_fitted_pos.v);
        aabb_combine_float(box, ring_points[rpi]->point.v);
        aabb_combine_float(box, ring_points[rpi + 1]->point.v);
    }

    for (int iter = 0; iter < iteration; ++iter)
    {
        PointLinearLeastSquares lls(oc, &(result.new_fitted_pos));

        for (int point_index : data_point_indices)
        {
            Vector3 data_point = oc->data_points[point_index];

            // acceleration with aabb distance
            for (int rpi = 0; rpi < number_ring_points - 1; ++rpi)
            {
                ring_distance_sqs[rpi] = aabb_distance_exterior_sq_point(&(bbox[rpi]), data_point);
            }

            int min_index = -1;
            float min_distance_tri = MOI_BIG_FLOAT;
            Vector3 min_bary;

            // find the barycentric coordinates of the closest point to the triangle (new_v, ring_point1, ring_point2) for one data_point.
            for (;;)
            {
                int min_distance_index = moi_get_min_value_index(ring_distance_sqs, number_ring_points - 1);
                float min_distance_box = ring_distance_sqs[min_distance_index];

                if (min_distance_box == MOI_BIG_FLOAT) break; // all boxes consumed
                if (min_distance_box >= min_distance_tri) break; // already found the minimum distance
                ring_distance_sqs[min_distance_index] = MOI_BIG_FLOAT; // mark as used

                TriangleClosestPointResult tcpr; triangle_closest_point_detail
                (
                    data_point,
                    result.new_fitted_pos,
                    ring_points[min_distance_index]->point,
                    ring_points[min_distance_index + 1]->point,
                    &tcpr
                );

                if (tcpr.dist_sq < min_distance_tri)
                {
                    min_distance_tri = tcpr.dist_sq;
                    min_index = min_distance_index;
                    min_bary = tcpr.barycentric;
                }
            }

            lls.EnterProjection(data_point, ring_points[min_index]->point, ring_points[min_index + 1]->point, min_bary.v[1], min_bary.v[2]);
        }

        if (oc->kappa)
        {
            for (int rpi = 0; rpi < number_ring_points - (int)is_closed; ++rpi)
            {
                bool is_boundary = is_closed == false && (rpi == 0 || rpi == number_ring_points - 1);
                lls.EnterSpring(ring_points[rpi]->point, is_boundary == false ? sqrt_kappa : sqrt_kappa_boundary);
            }
        }

        rss1 = lls.Solve();
        if (iter == 0)
        {
            result.energy_after_first_projection = lls.rss0;
        }
    }
    
    result.energy_after_refit = rss1;

    allocator->Free(ring_distance_sqs);
    allocator->Free(bbox);

    return result;
}

void moi_local_fit_ring(OptimizeContext* oc, Vertex* v, int local_fit_iteration)
{
    PoolAllocator* allocator = oc->target_mesh->allocator;

    TVector<Vertex*> ring_points(allocator);
    TVector<Face*> faces(allocator);
    TVector<int> data_point_indices(allocator);

    vertex_get_ring(v, ring_points);
    for (HalfEdge* he : v->arhe)
    {
        Face* f = he->face;
        faces.push_back(f);

        if (oc->face_point_map.count(f->id) > 0)
        {
            TUnorderedSet<int>& set = oc->face_point_map[f->id];
            for (int pi : set)
            {
                data_point_indices.push_back(pi);
            }
        }
        
    }

    Vector3 before_new_point = v->point;
    float min_local_diheral_before = moi_get_minimum_local_dihedral(ring_points, before_new_point);

    LocalFitOptimizationResult opt_result = moi_local_fit_optimization(oc, data_point_indices, ring_points, local_fit_iteration, before_new_point);
    float min_local_dihedral_after = moi_get_minimum_local_dihedral(ring_points, opt_result.new_fitted_pos);
    if (min_local_dihedral_after < MOI_MIN_COS && min_local_dihedral_after < min_local_diheral_before)
        return;

    v->point = opt_result.new_fitted_pos;
    moi_reproject_locally(oc, data_point_indices, faces);
}

void moi_local_fit(OptimizeContext* oc)
{
    Mesh* mesh = oc->target_mesh;
    for (int i = 0; i < oc->localfit_outer_iteration; ++i)
    {
        for (TUnorderedMap<uint32_t, Vertex*>::iterator it = mesh->vertex_map.begin();
            it != mesh->vertex_map.end();
            ++it)
        {
            Vertex* v = it->second;
            moi_local_fit_ring(oc, v, oc->localfit_inner_iteration);
        }
    }
}