#include "meshopt.h"
#include "meshopt_internal.h"

#include "filebuffer.h"

#include <time.h>

static inline void compute_aabb_and_transform(OptimizeContext* oc)
{
    AABB aabb;
    aabb_set_infinity(&aabb);

    Mesh* m = oc->target_mesh;
    for (Vector3 dp : oc->data_points)
    {
        aabb_combine_float(&aabb, dp.v);
    }

    for (TUnorderedMap<uint32_t, Vertex*>::iterator it = m->vertex_map.begin();
        it != m->vertex_map.end();
        ++it)
    {
        Vertex* v = it->second;
        aabb_combine_float(&aabb, v->point.v);
    }

    oc->original_mesh_aabb = aabb;

    Vector3 unit_cube = vector3_set1(0.5f);
    aabb_set_infinity(&oc->target_mesh_aabb);

    aabb_combine_float(&oc->target_mesh_aabb, unit_cube.v);
    unit_cube = vector3_set1(-0.5f);
    aabb_combine_float(&oc->target_mesh_aabb, unit_cube.v);

    oc->transform_to_unit = aabb_get_transform_to_unit_cube(&aabb);
    oc->transform_to_original = matrix4_inverse(&(oc->transform_to_unit));
}

static inline void transform_data_point(OptimizeContext* oc, Matrix4* transform)
{
    for (Vector3& data_point : oc->data_points)
    {
        data_point = matrix4_mul_vec3(transform, data_point);
    }
}

static inline void check_edge_sharpness(OptimizeContext* oc)
{
    constexpr float DEG_2_RAD = 3.1415926f / 180.f;
    constexpr float SHARP_ANGLE = 52.f;
    constexpr float SHARP_COSANGLE = SHARP_ANGLE * DEG_2_RAD;

    EdgeIteration ei = edge_get_iteration(oc->target_mesh);
    while (Edge* e = edge_iterate(&ei))
    {
        if (edge_is_boundary(e) == true)
            continue;

        float angcos = moi_get_edge_dihedral(e);
        if (angcos < SHARP_COSANGLE)
        {
            e->is_sharp = true;
        }
    }
}

static inline OptimizeContext* create_optimize_context(Mesh* original_mesh, const std::vector<Vector3>& sampled_points)
{
    Mesh* target_mesh = mesh_clone(original_mesh);

    PoolAllocator* allocator = target_mesh->allocator;

    OptimizeContext* oc = (OptimizeContext*)allocator->Alloc(sizeof(OptimizeContext));
    new (oc) OptimizeContext(target_mesh->allocator);

    oc->original_mesh = original_mesh;
    oc->target_mesh = target_mesh;
    oc->data_points = sampled_points;
    oc->data_faces.resize(oc->data_points.size());
    oc->data_projected_points.resize(oc->data_points.size());

#ifdef USE_AMPL_SOLVER
    if (G_AMPL_ENV_PATH != NULL)
        oc->ampl_solver = ampl_solver_create(allocator);
    else
        oc->ampl_solver = NULL;
#endif

    oc->localfit_outer_iteration = 2;
    oc->localfit_inner_iteration = 3;

    oc->edge_op_iter_factor = 1;

    oc->kappa = 0.f;
    oc->kappa_boundary_factor = 1.f;

    oc->crep = 1e-2f;
    oc->crep_boundary_factor = 3.f;

    oc->edge_swap_sym_factor = 0.01f;

    oc->verbose_warning = false;

    oc->export_every_step = false;
    oc->export_name = "steps/op";
    oc->export_step = 0;

    time_t rawtime;
    tm* timeinfo;

    time(&rawtime);
    timeinfo = localtime(&rawtime);
    int yy = 1900 + timeinfo->tm_year;
    int mo = timeinfo->tm_mon + 1;
    int dd = timeinfo->tm_mday;
    int hh = timeinfo->tm_hour;
    int mi = timeinfo->tm_min;
    int ss = timeinfo->tm_sec;
    oc->export_folder.AssignSPrintf("Opt%d_%d_%d_%d_%d_%d", yy, mo, dd, hh, mi, ss);

    if (false == directory_is_exist(oc->export_folder.data()))
    {
        directory_create(oc->export_folder.data());
    }

    return oc;
}

static inline void destroy_optimize_context(OptimizeContext* oc, float* kappas, int kappa_scheme_size)
{
    PoolAllocator* allocator = oc->target_mesh->allocator;

#ifdef USE_AMPL_SOLVER
    if (G_AMPL_ENV_PATH != NULL)
    {
        ampl_solver_destroy(oc->ampl_solver);
    }
#endif

    {  // export option
        PoolString str(allocator);
        str.AssignSPrintf("%s/option.txt", oc->export_folder.data());

        FileBuffer fb;
        filebuffer_open(&fb, str.data(), "wb");

        str.AssignSPrintf("localfit_outer_iteration : %d\n", oc->localfit_outer_iteration);
        filebuffer_write_str(&fb, str.data());

        str.AssignSPrintf("localfit_inner_iteration : %d\n", oc->localfit_inner_iteration);
        filebuffer_write_str(&fb, str.data());

        for (int i = 0; i < kappa_scheme_size; ++i)
        {
            str.AssignSPrintf("kappa%d : %.3e\n", i + 1, kappas[i]);
            filebuffer_write_str(&fb, str.data());
        }

        str.AssignSPrintf("kappa_boundary_factor : %.3e\n", oc->kappa_boundary_factor);
        filebuffer_write_str(&fb, str.data());

        str.AssignSPrintf("crep : %.3e\n", oc->crep);
        filebuffer_write_str(&fb, str.data());

        str.AssignSPrintf("crep_boundary_factor : %.3e\n", oc->crep_boundary_factor);
        filebuffer_write_str(&fb, str.data());

        str.AssignSPrintf("edge_swap_sym_factor : %.3e\n", oc->edge_swap_sym_factor);
        filebuffer_write_str(&fb, str.data());

        str.AssignSPrintf("edge_op_iter_factor : %d\n", oc->edge_op_iter_factor);
        filebuffer_write_str(&fb, str.data());

        str.AssignSPrintf("time_duration_seconds : %f\n", oc->time_duration_seconds);
        filebuffer_write_str(&fb, str.data());

        filebuffer_close(&fb);
    }

    { // export random points
        transform_data_point(oc, &oc->transform_to_original);

        FileBuffer fb;
        PoolString str(allocator);
        str.AssignSPrintf("%s/randpts.txt", oc->export_folder.data());
        filebuffer_open(&fb, str.data(), "wb");

        filebuffer_write_int(&fb, (int)oc->data_points.size());
        filebuffer_write_str(&fb, "\n");
        for (int i = 0; i < (int)oc->data_points.size(); ++i)
        {
            Vector3 p = oc->data_points[i];

            for (int j = 0; j < 3; ++j)
            {
                filebuffer_write_float(&fb, p.v[j]);
                filebuffer_write_str(&fb, " ");
            }
            filebuffer_write_str(&fb, "\n");
        }

        filebuffer_close(&fb);
    }

    oc->~OptimizeContext();
    allocator->Free(oc);
}

Mesh* mesh_optimize
(
    Mesh* m,
    const std::vector<Vector3>& sampled_points,
    float rep_constant,
    float spring_constant
)
{
    constexpr bool EXPORT_INTERMEDIATE_OBJ = true;

    OptimizeContext* oc = create_optimize_context(m, sampled_points);
    
    // check_edge_sharpness(oc);

    if (EXPORT_INTERMEDIATE_OBJ)
    {
        PoolString origin_path(oc->target_mesh->allocator);
        origin_path.AssignSPrintf("%s/origin.obj", oc->export_folder.data());

        mesh_export_obj(oc->target_mesh, origin_path.data());
    }

    compute_aabb_and_transform(oc);

    moi_transform_mesh(oc, &oc->transform_to_unit);

    transform_data_point(oc, &oc->transform_to_unit);

    moi_global_projection(oc);

    float kappa_scheme[] = { 1e-2f, 1e-3f, 1e-4f, 1e-8f };
    int scheme_size = sizeof(kappa_scheme) / sizeof(kappa_scheme[0]);
    int scheme_index = 0;

    if (oc->export_every_step)
    {
        moi_export_mesh(oc, oc->export_name, oc->export_step);
        ++oc->export_step;
    }

    clock_t time_check = clock();

    while (scheme_index < scheme_size)
    {
        oc->kappa = kappa_scheme[scheme_index];
        ++scheme_index;

        moi_local_fit(oc);
        // moi_global_fit(oc, 30);

        if (EXPORT_INTERMEDIATE_OBJ)
            moi_export_mesh(oc, "localfit_after", scheme_index);

        if (oc->export_every_step)
        {
            moi_export_mesh(oc, oc->export_name, oc->export_step);
            ++oc->export_step;
        }

        moi_simplicies_fit(oc);

        if (EXPORT_INTERMEDIATE_OBJ)
            moi_export_mesh(oc, "simpliciesfit_after", scheme_index);

        if (oc->export_every_step)
        {
            moi_export_mesh(oc, oc->export_name, oc->export_step);
            ++oc->export_step;
        }

        moi_local_fit(oc);
        // moi_global_fit(oc, 30);

        if (oc->export_every_step)
        {
            moi_export_mesh(oc, oc->export_name, oc->export_step);
            ++oc->export_step;
        }
    }

    time_check = clock() - time_check;

    oc->time_duration_seconds = ((float)time_check) / CLOCKS_PER_SEC;
    
    Mesh* optimized_mesh = oc->target_mesh; // keep the optimized mesh

    moi_export_mesh(oc, "optimized");
    
    moi_transform_mesh(oc, &oc->transform_to_original); // transform back to the original space

    destroy_optimize_context(oc, kappa_scheme, scheme_size);
    
    return optimized_mesh;
}