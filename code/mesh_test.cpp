#include "mesh_test.h"

#include "mesh.h"
#include "string.h"

#define TEST_FOLDER_WITHOUT_SEPARATOR "mesh_test"
#define TEST_FOLDER "mesh_test/"

static const Vector3 s_default_colors[] =
{
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
    {0.2f, 0.2f, 0.2f},
};

static const Vector3 s_vertices[] =
{
    {0.f, 0.f, 0.f}, // 0
    {0.5f, 0.3f, 0.f},
    {0.f, 1.f, 0.f},
    {-0.5f, 0.3f, 0.f},
    {-0.5f, -0.3f, 0.f}, 
    {0.f, -1.f, 0.f},
    {0.5f, -0.3f, 0.f}, // 6
    {1.f, -0.3f, 0.f}, // 7
    {1.f, 0.3f, 0.f},
    {0.7f, 1.2f, 0.f},
    {0.f, 1.5f, 0.f}, // 10
    {-0.5f, 1.2f, 0.f}, 
    {-0.8f, 1.f, 0.f},
    {-1.f, 0.f, 0.f}, // 13
    {-0.8f, -1.f, 0.f},
    {0.f, -1.5f, 0.f},
    {0.7f, -1.2f, 0.f},
};

static const int s_faces[] =
{
    0, 1, 2, // 0
    0, 2, 3,
    0, 3, 4,
    0, 4, 5,
    0, 5, 6,
    0, 6, 1, // 5

    1, 6, 7, 
    1, 7, 8,
    1, 8, 9,
    1, 9, 2,
    2, 9, 10, // 10
    2, 10, 11,
    2, 11, 3,
    3, 11, 12,
    3, 12, 13,
    3, 13, 4, // 15
    4, 13, 14, 
    4, 14, 5,
    5, 14, 15,
    5, 15, 16,
    6, 5, 16, // 20
    6, 16, 7
};

static const int s_vertex_count = (int)(sizeof(s_vertices) / sizeof(s_vertices[0]));
static const int s_face_arr_count = (int)(sizeof(s_faces) / sizeof(s_faces[0]));
static const int s_face_count = s_face_arr_count / 3;

static void test_default_obj();
static void test_halfedge_get_clw();
static void test_halfedge_get_ccw();
static void test_halfedge_get_most_clw();
static void test_edge_get_side_vertex();
static void test_edge_vertex_iteration();
static void test_edge_get_ring();
static void test_vertex_get_most_clw();
static void test_vertex_iteration();
static void test_vertex_boundary();
static void test_vertex_get_ring();
static void test_edge_collapse();
static void test_edge_split();


typedef void (*TestFunc)();

static TestFunc s_tests[] =
{
    test_default_obj,
    test_halfedge_get_clw,
    test_halfedge_get_ccw,
    test_halfedge_get_most_clw,
    test_edge_get_side_vertex,
    test_vertex_get_most_clw,
    test_vertex_iteration,
    test_vertex_boundary,
    test_vertex_get_ring,
    test_edge_vertex_iteration,
    test_edge_get_ring,
    test_edge_collapse,
    test_edge_split,
};

void mesh_test()
{
    if (directory_is_exist(TEST_FOLDER_WITHOUT_SEPARATOR) == false)
    {
        directory_create(TEST_FOLDER_WITHOUT_SEPARATOR);
    }

    int test_count = sizeof(s_tests) / sizeof(s_tests[0]);

    for (int i = 0; i < test_count; ++i)
    {
        s_tests[i]();
    }
}

static Mesh* create_default_mesh()
{
    return mesh_create_vertices_faces(s_vertices, s_vertex_count, s_faces, s_face_arr_count);
}

static void color_halfedge(Vector3* colors, HalfEdge* he, Vector3 from_color, Vector3 to_color)
{
    colors[he->prev->vert->id] = from_color;
    colors[he->vert->id] = to_color;
}

static void test_default_obj()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];
    
    memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
    mesh_export_obj_with_colors(mesh, TEST_FOLDER"default.obj", t_colors, s_vertex_count);

    mesh_destroy(mesh);
}

static void test_halfedge_get_clw()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    Vertex* center_v = mesh->vertex_map[0];
    HalfEdge* he = center_v->arhe[0]->prev;

    int index = 1;
    for (HalfEdge* hef = he;;)
    {
        if (index == 7)
            break;

        HalfEdge* he_clw = halfedge_get_clw(he);
        if (he_clw == NULL || he_clw == he)
            break;

        PoolString output_path(mesh->allocator);
        output_path.AssignSPrintf(TEST_FOLDER"halfedge_get_clw%d.obj", index);

        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        color_halfedge(t_colors, he, VECTOR3_COLOR_RED, VECTOR3_COLOR_BLUE);
        color_halfedge(t_colors, he_clw, VECTOR3_COLOR_GREEN, VECTOR3_COLOR_PALE_PINK);
        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);

        he = he_clw;
        ++index;
    }

    mesh_destroy(mesh);
}

static void test_halfedge_get_ccw()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    Vertex* center_v = mesh->vertex_map[0];
    HalfEdge* he = center_v->arhe[0]->prev;

    int index = 1;
    for (HalfEdge* hef = he;;)
    {
        if (index == 7)
            break;

        HalfEdge* he_clw = halfedge_get_ccw(he);
        if (he_clw == NULL || he_clw == he)
            break;

        PoolString output_path(mesh->allocator);
        output_path.AssignSPrintf(TEST_FOLDER"halfedge_get_ccw%d.obj", index);

        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        color_halfedge(t_colors, he, VECTOR3_COLOR_RED, VECTOR3_COLOR_BLUE);
        color_halfedge(t_colors, he_clw, VECTOR3_COLOR_GREEN, VECTOR3_COLOR_PALE_PINK);
        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);

        he = he_clw;
        ++index;
    }

    mesh_destroy(mesh);
}

static void test_halfedge_get_most_clw()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    for (int i = 0; i < s_vertex_count; ++i)
    {
        Vertex* v = mesh->vertex_map[i];
        HalfEdge* he = halfedge_get_most_clw(v);

        if (he == NULL)
            break;

        PoolString output_path(mesh->allocator);
        output_path.AssignSPrintf(TEST_FOLDER"halfedge_get_most_clw%d.obj", i);

        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        t_colors[i] = VECTOR3_COLOR_RED;
        color_halfedge(t_colors, he, VECTOR3_COLOR_GREEN, VECTOR3_COLOR_PALE_PINK);

        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);
    }

    mesh_destroy(mesh);
}

static void test_edge_get_side_vertex()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    {
        TVector<Edge*> edges(mesh->allocator);
        EdgeIteration ei = edge_get_iteration(mesh);
        while (Edge* e = edge_iterate(&ei))
        {
            edges.push_back(e);
        }

        PoolString output_path(mesh->allocator);
        output_path.Reserve(64);
        for (int i = 0; i < (int)edges.size(); ++i)
        {
            output_path.AssignSPrintf(TEST_FOLDER"edge_get_side_vertex_%d.obj", i);

            Edge* e = edges[i];
            Vertex* sv1 = edge_get_side_vertex1(e);
            Vertex* sv2 = edge_get_side_vertex2(e);
            HalfEdge* he = e->herep;

            memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
            color_halfedge(t_colors, he, VECTOR3_COLOR_RED, VECTOR3_COLOR_BLUE);
            t_colors[sv1->id] = VECTOR3_COLOR_YELLOW;
            if (sv2)
                t_colors[sv2->id] = VECTOR3_COLOR_PURPLE;

            mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);
        }
    }

    mesh_destroy(mesh);
}

static void test_edge_vertex_iteration()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    {
        Vertex* v = mesh->vertex_map[0];
        EdgeVertexIteration evi = edge_vertex_get_iteration(v);
        int index = 0;

        PoolString output_path(mesh->allocator);
        output_path.Reserve(64);
        while (Edge* ie = edge_vertex_iterate(&evi))
        {
            output_path.AssignSPrintf(TEST_FOLDER"edge_vertex_iteration_%d.obj", index);

            memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
            color_halfedge(t_colors, ie->herep, VECTOR3_COLOR_RED, VECTOR3_COLOR_BLUE);

            mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);

            ++index;
        }
    }

    mesh_destroy(mesh);
}

static void test_edge_get_ring()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    {
        Vertex* v = mesh->vertex_map[0];
        Edge* e = v->arhe[0]->edge;

        TVector<Vertex*> vertices(mesh->allocator);
        edge_get_ring(e, vertices);

        color_halfedge(t_colors, e->herep, VECTOR3_COLOR_RED, VECTOR3_COLOR_BLUE);

        for (Vertex* ring_v : vertices)
        {
            t_colors[ring_v->id] = VECTOR3_COLOR_GREEN;
        }

        mesh_export_obj_with_colors(mesh, TEST_FOLDER"edge_get_ring.obj", t_colors, s_vertex_count);
    }

    mesh_destroy(mesh);
}

static void test_vertex_get_most_clw()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    {
        PoolString output_path(mesh->allocator);
        output_path.Reserve(64);

        int mesh_vertices[] = { 0, 4, 9 };

        for (int i = 0; i < 3; ++i)
        {
            output_path.AssignSPrintf(TEST_FOLDER"vertex_get_most_clw_%d.obj", i);

            Vertex* v = mesh->vertex_map[mesh_vertices[i]];
            Vertex* mclwv = vertex_get_most_clw(v);

            memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
            if (v == mclwv)
                t_colors[v->id] = VECTOR3_COLOR_INDIGO;
            else
            {
                t_colors[v->id] = VECTOR3_COLOR_RED;
                t_colors[mclwv->id] = VECTOR3_COLOR_BLUE;
            }

            mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);
        }
    }

    mesh_destroy(mesh);
}

static void test_vertex_iteration()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    {
        Vertex* v = mesh->vertex_map[0];
        VertexIteration vi = vertex_get_iteration(v);
        int index = 0;

        PoolString output_path(mesh->allocator);
        output_path.Reserve(64);
        while (Vertex* iv = vertex_iterate(&vi))
        {
            output_path.AssignSPrintf(TEST_FOLDER"vertex_iteration_%d.obj", index);

            memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
            t_colors[v->id] = VECTOR3_COLOR_RED;
            t_colors[iv->id] = VECTOR3_COLOR_BLUE;
            mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);

            ++index;
        }
    }

    mesh_destroy(mesh);
}

static void test_vertex_boundary()
{
    Mesh* mesh = create_default_mesh();
    int boundary = 0;

    for (TUnorderedMap<uint32_t, Vertex*>::iterator it = mesh->vertex_map.begin();
        it != mesh->vertex_map.end();
        ++it)
    {
        if (vertex_is_boundary(it->second) == true)
        {
            ++boundary;
        }
    }

    mesh_destroy(mesh);
}

static void test_vertex_get_ring()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    {
        Vertex* v = mesh->vertex_map[0];

        TVector<Vertex*> ring(mesh->allocator);
        vertex_get_ring(v, ring);

        Vertex* w = vertex_get_most_clw(v);

        PoolString output_path(mesh->allocator);
        output_path.Reserve(64);
        output_path.Assign(TEST_FOLDER"vertex_get_ring.obj");

        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        t_colors[v->id] = VECTOR3_COLOR_RED;
        for (Vertex* vv : ring)
            t_colors[vv->id] = VECTOR3_COLOR_BLUE;
        t_colors[w->id] = VECTOR3_COLOR_YELLOW;

        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);
    }

    mesh_destroy(mesh);
}

static void test_edge_collapse()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    {
        PoolString output_path(mesh->allocator);
        output_path.Reserve(64);


        Vertex* v = mesh->vertex_map[0];
        Edge* e = v->arhe[0]->edge;

        output_path.Assign(TEST_FOLDER"edge_collapse_0.obj");
        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        color_halfedge(t_colors, e->herep, VECTOR3_COLOR_RED, VECTOR3_COLOR_BLUE);
        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);

        bool is_nice_collapse = mesh_is_nice_edge_collapse(mesh, e);

        mesh_collapse_edge(mesh, e);
        output_path.Assign(TEST_FOLDER"edge_collapse_1.obj");
        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, mesh->vertex_map.size());
    }
    mesh_destroy(mesh);

    mesh = create_default_mesh();
    {
        PoolString output_path(mesh->allocator);
        output_path.Reserve(64);
        
        Vertex* v = mesh->vertex_map[9];
        Edge* e = v->arhe[2]->edge;
        output_path.Assign(TEST_FOLDER"edge_collapse_2.obj");
        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        color_halfedge(t_colors, e->herep, VECTOR3_COLOR_RED, VECTOR3_COLOR_BLUE);

        bool is_nice_collapse = mesh_is_nice_edge_collapse(mesh, e);

        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);

        mesh_collapse_edge(mesh, e);
        output_path.Assign(TEST_FOLDER"edge_collapse_3.obj");
        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, mesh->vertex_map.size());
    }
    mesh_destroy(mesh);


    // simplest 4 vertex K.
    const Vector3 vertices[] =
    {
        {-1.f, 0.f, 0.f},
        {0.f, -1.f, 0.f},
        {0.f, 1.f, 0.f},
        {1.f, 0.f, 0.f},
    };
    const int faces[] =
    {
        0, 1, 2,
        2, 1, 3
    };
    mesh = mesh_create_vertices_faces(vertices, 4, faces, 6);
    {
        PoolString output_path(mesh->allocator);
        output_path.Reserve(64);

        Vertex* v = mesh->vertex_map[1];
        Edge* e = v->arhe[0]->edge;

        output_path.Assign(TEST_FOLDER"edge_collapse_not_nice.obj");
        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        color_halfedge(t_colors, e->herep, VECTOR3_COLOR_RED, VECTOR3_COLOR_BLUE);
        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, 4);

        v = mesh->vertex_map[0];
        e = v->arhe[0]->edge;

        output_path.Assign(TEST_FOLDER"edge_collapse_4.obj");
        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        color_halfedge(t_colors, e->herep, VECTOR3_COLOR_RED, VECTOR3_COLOR_BLUE);
        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, 4);

        bool is_nice_collapse = mesh_is_nice_edge_collapse(mesh, e);

        mesh_collapse_edge(mesh, e);
        output_path.Assign(TEST_FOLDER"edge_collapse_5.obj");
        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, mesh->vertex_map.size());

    }
    mesh_destroy(mesh);
}

static void test_edge_split()
{
    Mesh* mesh = create_default_mesh();
    Vector3 t_colors[s_vertex_count];

    {
        PoolString output_path(mesh->allocator);
        output_path.Reserve(64);

        Vertex* v = mesh->vertex_map[0];
        Edge* e = v->arhe[0]->edge;

        output_path.Assign(TEST_FOLDER"edge_split_0.obj");
        memcpy(t_colors, s_default_colors, sizeof(s_default_colors));
        color_halfedge(t_colors, e->herep, VECTOR3_COLOR_RED, VECTOR3_COLOR_BLUE);
        mesh_export_obj_with_colors(mesh, output_path.data(), t_colors, s_vertex_count);

        Vertex* split_v = mesh_split_edge(mesh, e);

        output_path.Assign(TEST_FOLDER"edge_split_1.obj");
        Vector3 tt_colors[s_vertex_count + 1];
        for (int i = 0; i < s_vertex_count + 1; ++i)
            tt_colors[i] = s_default_colors[0];
        tt_colors[split_v->id] = VECTOR3_COLOR_RED;
        VertexIteration vi = vertex_get_iteration(split_v);
        while (Vertex* vv = vertex_iterate(&vi))
        {
            tt_colors[vv->id] = VECTOR3_COLOR_BLUE;
        }

        mesh_export_obj_with_colors(mesh, output_path.data(), tt_colors, mesh->vertex_map.size());
    }

    mesh_destroy(mesh);
}