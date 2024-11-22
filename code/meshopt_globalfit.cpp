#include "meshopt_internal.h"

#ifdef USE_AMPL_SOLVER
#include <ampl/ampl.h>
#endif

#include "geometry_algorithm.h"

const char* G_MESH_OPT_GLOBAL_FIT_AMPL_MODEL =
// linear least squares problem
"set M;\n"
"set N;\n"
"param A {M, N} default 0;\n"
"param b {M, 1..3} default 0;\n"
"var x {N, 1..3};\n"
"minimize linear_least_squares:\n"
"    sum{i in M, j in 1..3} (sum{c in N}(A[i, c] * x[c, j]) - b[i, j])^2\n"
;

void moi_global_fit(OptimizeContext* oc, int max_iteration)
{
    moi_global_projection(oc);

#ifdef USE_AMPL_SOLVER

    // TODO set max_iteration for the option of ampl
    ampl::AMPL* ampl_pointer = (ampl::AMPL*)ampl_solver_get_context(oc->ampl_solver);
    ampl::AMPL& ampl = *ampl_pointer;
    ampl.reset();

    ampl_solver_read_model(oc->ampl_solver, G_MESH_OPT_GLOBAL_FIT_AMPL_MODEL);
    ampl.setOption("solver", "couenne");
    // ampl.setOption("linear_solver", "ma97");

    Mesh* mesh = oc->target_mesh;
    PoolAllocator* allocator = mesh->allocator;

    const int data_point_count = (int)oc->data_points.size();
    const int edge_count = mesh->num_edges;
    const int m_count = data_point_count + edge_count;
    const int n_count = mesh->num_vertex;

    TVector<Vertex*> vertex_linear(allocator);
    TUnorderedMap<uint32_t, int> vertex_id_to_index(allocator); // starts from 1
    { // cache vertex id to index for constructing data
        vertex_linear.reserve(mesh->num_vertex);
        vertex_id_to_index.reserve(mesh->num_vertex);

        for (const TUnorderedMap<uint32_t, Vertex*>::value_type& value : mesh->vertex_map)
        {
            Vertex* v = value.second;

            vertex_linear.push_back(v);
            vertex_id_to_index[v->id] = (int)vertex_linear.size();
        }
    }

    { // set indexing data
        double* m_data = (double*)(malloc(sizeof(double) * m_count * n_count));
        double* n_data = m_data + m_count;

        for (int i = 1; i <= m_count; ++i)
            m_data[i - 1] = (double)i;

        for (int i = 1; i <= n_count; ++i)
            n_data[i - 1] = (double)i;

        ampl.getSet("M").setValues(m_data, m_count);
        ampl.getSet("N").setValues(n_data, n_count);

        free(m_data);
    }

    
    { // Set A Matrix
        ampl::DataFrame matrix_a(2, ampl::StringArgs("M", "N", "A"));
        matrix_a.reserve(data_point_count * 3 + edge_count * 2);

        TVector<double> rows(allocator);
        TVector<double> cols(allocator);
        TVector<double> values(allocator);

        rows.reserve(data_point_count * 3 + edge_count * 2);
        cols.reserve(data_point_count * 3 + edge_count * 2);
        values.reserve(data_point_count * 3 + edge_count * 2);
        
        // set the barycentric coordinates for the data points
        Vertex* vertices[3];
        int a_row_index = 1;
        for (int i = 0; i < oc->data_points.size(); ++i)
        {
            Vector3 data_point = oc->data_points[i];
            Face* closest_face = oc->data_faces[i];
            face_get_triangle_vertices(closest_face, vertices);

            TriangleClosestPointResult tcpr;
            triangle_closest_point_detail
            (
                data_point,
                vertices[0]->point,
                vertices[1]->point,
                vertices[2]->point,
                &tcpr
            );

            for (int j = 0; j < 3; ++j)
            {
                int col_index = vertex_id_to_index[vertices[j]->id];
                double barycentric = tcpr.barycentric.v[j];

                rows.push_back(a_row_index);
                cols.push_back(col_index);
                values.push_back(barycentric);
            }

            ++a_row_index;
        }

        float sqrt_kappa = sqrtf(oc->kappa);
        float sqrt_boundary_kappa = sqrtf(oc->kappa * oc->kappa_boundary_factor);

        // set the sqrt kappa for spring terms
        EdgeIteration ei = edge_get_iteration(mesh);
        while (Edge* e = edge_iterate(&ei))
        {
            float sqrt_tension = sqrt_kappa;
            if (edge_is_boundary(e) == true)
                sqrt_tension = sqrt_boundary_kappa;

            Vertex* v1 = edge_get_vertex1(e);
            Vertex* v2 = edge_get_vertex2(e);

            rows.push_back(a_row_index);
            cols.push_back(vertex_id_to_index[v1->id]);
            values.push_back(sqrt_tension);

            rows.push_back(a_row_index);
            cols.push_back(vertex_id_to_index[v2->id]);
            values.push_back(-sqrt_tension);

            ++a_row_index;
        }

        matrix_a.setColumn("M", rows.data(), rows.size());
        matrix_a.setColumn("N", cols.data(), cols.size());
        matrix_a.setColumn("A", values.data(), values.size());

        ampl.setData(matrix_a);
    }

    { // set b matrix
        ampl::DataFrame matrix_b(2, ampl::StringArgs("M", "DIM", "b"));
        matrix_b.reserve(data_point_count * 3);

        TVector<double> rows(allocator);
        TVector<double> cols(allocator);
        TVector<double> values(allocator);
        rows.reserve(data_point_count * 3);
        cols.reserve(data_point_count * 3);
        values.reserve(data_point_count * 3);

        for (int data_point_row = 1; data_point_row <= (int)oc->data_points.size(); ++data_point_row)
        {
            Vector3 data_point = oc->data_points[data_point_row - 1];

            for (int i = 1; i <= 3; ++i)
            {
                rows.push_back(data_point_row);
                cols.push_back(i);
                values.push_back(data_point.v[i - 1]);
            }
        }

        matrix_b.setColumn("M", rows.data(), rows.size());
        matrix_b.setColumn("DIM", cols.data(), cols.size());
        matrix_b.setColumn("b", values.data(), values.size());

        ampl.setData(matrix_b);
    }

    { // set x matrix
        ampl::DataFrame matrix_x(2, ampl::StringArgs("N", "DIM", "x"));
        matrix_x.reserve(n_count * 3);

        TVector<double> rows(allocator);
        TVector<double> cols(allocator);
        TVector<double> values(allocator);
        rows.reserve(n_count * 3);
        cols.reserve(n_count * 3);
        values.reserve(n_count * 3);

        for (int i = 0; i < vertex_linear.size(); ++i)
        {
            Vertex* v = vertex_linear[i];

            for (int j = 0; j < 3; ++j)
            {
                rows.push_back(i + 1);
                cols.push_back(j + 1);
                values.push_back(v->point.v[j]);
            }
        }

        matrix_x.setColumn("N", rows.data(), rows.size());
        matrix_x.setColumn("DIM", cols.data(), cols.size());
        matrix_x.setColumn("x", values.data(), values.size());

        ampl.setData(matrix_x);
    }
    
    // solve
    ampl.solve();

    // data fetch
    ampl::Objective objective = ampl.getObjective("linear_least_squares");
    double objective_value = objective.value();
    printf("Global Fit Objective Value %f\n", objective_value);

    ampl::Variable v = ampl.getVariable("x");
    ampl::DataFrame vdf = v.getValues();

    ampl::DataFrame::Column col = vdf.getColumn("x.val");
    ampl::DataFrame::Column::iterator it = col.begin();
    for (int i = 0; ; ++i)
    {
        if (it == col.end())
            break;

        vertex_linear[i]->point.v[0] = it->dbl();
        ++it;

        vertex_linear[i]->point.v[1] = it->dbl();
        ++it;

        vertex_linear[i]->point.v[2] = it->dbl();
        ++it;
    }
    
    ampl.reset();

    moi_global_projection(oc);
#endif
}