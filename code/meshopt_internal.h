#ifndef __MESHOPT_INTERNAL_H__
#define __MESHOPT_INTERNAL_H__

#include "mesh.h"
#include "matrix.h"
#include "aabb.h"
#include "ampl_solver.h"
#include "string.h"
#include <unordered_set>


struct OptimizeContext
{
    OptimizeContext(PoolAllocator* allocator)
        : data_points(allocator)
        , data_faces(allocator)
        , data_projected_points(allocator)
        , face_point_map(allocator)
        , edge_candidates(allocator)
        , export_folder(allocator)
    {}

    Mesh* original_mesh;
    Mesh* target_mesh;

    TVector<Vector3> data_points; // sampled from a mesh
    TVector<Face*> data_faces;  // closest face for the sampling point
    TVector<Vector3> data_projected_points; // projected point to the closest face
    TUnorderedMap<uint32_t, TUnorderedSet<int>> face_point_map; // key : face id, value : point indices where the points are cloes to the face.
    TUnorderedMap<uint32_t, Edge*> edge_candidates; // key : edge hash

    AABB original_mesh_aabb;
    AABB target_mesh_aabb;
    Matrix4 transform_to_unit;
    Matrix4 transform_to_original;

#ifdef USE_AMPL_SOLVER
    AMPLSolver* ampl_solver;
#endif

    int localfit_outer_iteration;
    int localfit_inner_iteration;

    float kappa;  // constant for spring energy.
    float kappa_boundary_factor; // multiply kappa by this for the boundary vertices.

    float crep; // constant representation energy
    float crep_boundary_factor;

    float edge_swap_sym_factor;

    int edge_op_iter_factor;

    bool verbose_warning;

    bool export_every_step;
    const char* export_name;
    int export_step;

    PoolString export_folder;

    float time_duration_seconds;
};

// meshopt_common.cpp
void moi_change_data_point_face(OptimizeContext* oc, Face* new_f, int point_index);
void moi_remove_data_point_face(OptimizeContext* oc, Face* f);
float moi_get_minimum_local_dihedral(const TVector<Vertex*>& wa, Vector3 new_p);
float moi_get_edge_dihedral(Edge* e);
int moi_get_min_value_index(const float* arr, int count);
uint32_t moi_get_hash_edge(Edge* e);
float moi_get_spring_energy_vv(OptimizeContext* oc, Vertex* v1, Vertex* v2);
float moi_get_spring_energy_edge(OptimizeContext* oc, Edge* e);
void moi_transform_mesh(OptimizeContext* oc, Matrix4* transform);

// export mesh with the 'nameindex.obj'
// if index is negative, then the index is not append to the file name.
void moi_export_mesh(OptimizeContext* oc, const char* name, int index=-1); 

// meshopt_projection.cpp
void moi_global_projection(OptimizeContext* oc);
void moi_reproject_locally
(
    OptimizeContext* oc,
    const TVector<int>& point_indices,
    const TVector<Face*>& faces
);
void moi_cleanup_neighborhood(OptimizeContext* oc, Vertex* v, int nri);

// meshopt_localfit.cpp
void moi_local_fit(OptimizeContext* oc);
void moi_local_fit_ring(OptimizeContext* oc, Vertex* v, int local_fit_iteration);

struct LocalFitOptimizationResult
{
    Vector3 new_fitted_pos;
    float energy_after_first_projection;
    float energy_after_refit;
};
LocalFitOptimizationResult moi_local_fit_optimization
(
    OptimizeContext* oc,
    const TVector<int>& point_indices,
    const TVector<Vertex*>& ring_points,
    int iteration,
    Vector3 initial_point
);

// meshopt_globalfit.cpp
extern const char* G_MESH_OPT_GLOBAL_FIT_AMPL_MODEL;
void moi_global_fit(OptimizeContext* oc, int max_iteration);

// meshopt_simpliciesfit.cpp
void moi_simplicies_fit(OptimizeContext* oc);

#define MOI_BIG_FLOAT 1e30f
#define MOI_MIN_COS -1.f / 3.f // acos(109.471) == tetrahedron angle

#endif