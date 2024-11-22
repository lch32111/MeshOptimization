#include "mesh.h"

#include "obj.h"
#include "geometry_algorithm.h"
#include "random.h"
#include "filebuffer.h"

#include <assert.h>
#include <set>

static inline Vertex* alloc_vertex(Mesh* m)
{
    Vertex* v = (Vertex*)m->allocator->Alloc(sizeof(Vertex));
    new (v) Vertex(m->allocator);
    return v;
}

static inline void free_vertex(Mesh* m, Vertex* v)
{
    v->~Vertex();
    m->allocator->Free(v);
}

static inline Face* alloc_face(Mesh* m)
{
    Face* v = (Face*)m->allocator->Alloc(sizeof(Face));
    memset(v, 0, sizeof(Face));
    return v;
}

static inline void free_face(Mesh* m, Face* f)
{
    m->allocator->Free(f);
}

static inline Edge* alloc_edge(Mesh* m)
{
    Edge* e = (Edge*)m->allocator->Alloc(sizeof(Edge));
    memset(e, 0, sizeof(Edge));
    return e;
}

static inline void free_edge(Mesh* m, Edge* e)
{
    m->allocator->Free(e);
}

static inline HalfEdge* alloc_halfedge(Mesh* m)
{
    HalfEdge* he = (HalfEdge*)m->allocator->Alloc(sizeof(HalfEdge));
    memset(he, 0, sizeof(HalfEdge));
    return he;
}

static inline void free_halfedge(Mesh* m, HalfEdge* he)
{
    m->allocator->Free(he);
}

static inline bool check_valid_face(Mesh* m, Vertex** va, int va_count)
{
    if (va_count == 3)
    {
        if (va[0] == va[1] || va[1] == va[2] || va[0] == va[2])
            return false;
    }
    else
    {
        TSet<uint32_t> setv(m->allocator);
        for (int i = 0; i < va_count; ++i)
        {
            if (setv.count(va[i]->id) == 0)
            {
                setv.insert(va[i]->id);
            }
            else
            {
                return false;
            }
        }
    }

    Vertex* vo = va[va_count - 1];
    for (int i = 0; i < va_count; ++i)
    {
        if (NULL != halfedge_query(vo, va[i]))
            return false;

        vo = va[i];
    }

    return true;
}

Mesh* mesh_create(ObjShape* shape)
{
    PoolAllocator* allocator = new PoolAllocator();
    Mesh* mesh = (Mesh*)allocator->Alloc(sizeof(Mesh));
    new (mesh) Mesh(allocator);

    // do convert ObjShape to Mesh
    mesh->num_vertex = 0;
    mesh->num_face = 0;
    mesh->num_edges = 0;

    int position_count = (int)shape->positions.size();

    TVector<Vertex*> vertex_linear(allocator);
    vertex_linear.resize(position_count / 3);
    
    for (int pi = 0; pi < position_count; pi += 3)
    {
        Vertex* v = mesh_create_vertex(mesh);
        v->point = vector3_setp(shape->positions.data() + pi);

        vertex_linear[pi / 3] = v;
    }

    int face_count = (int)shape->indices.size();
    for (int fi = 0; fi < face_count; fi += 3)
    {
        int indices[3] = { shape->indices[fi], shape->indices[fi + 1], shape->indices[fi + 2] };
        Vertex* va[3] = { vertex_linear[indices[0]], vertex_linear[indices[1]], vertex_linear[indices[2]] };

        if (check_valid_face(mesh, va, 3) == true)
        {
            Face* f = mesh_create_face(mesh, va, 3);
        }
        else
        {
            printf("Invalid Face %d", fi / 3);
        }
    }

    for (size_t i = 0; i < vertex_linear.size(); ++i)
    {
        Vertex* v = vertex_linear[i];
        assert(v->arhe.size() != 0); // There should not be vertices that are not referred by a face
    }

    return mesh;
}

Mesh* mesh_create_vertices_faces(const Vector3* vertices, int vertex_count, const int* faces, int face_index_count)
{
    PoolAllocator* allocator = new PoolAllocator();
    Mesh* mesh = (Mesh*)allocator->Alloc(sizeof(Mesh));
    new (mesh) Mesh(allocator);

    mesh->num_vertex = 0;
    mesh->num_face = 0;
    mesh->num_edges = 0;

    TVector<Vertex*> vertex_linear(allocator);
    vertex_linear.resize(vertex_count);

    for (int pi = 0; pi < vertex_count; ++pi)
    {
        Vertex* v = mesh_create_vertex(mesh);
        v->point = vertices[pi];

        vertex_linear[pi] = v;
    }

    for (int fi = 0; fi < face_index_count; fi += 3)
    {
        int indices[3] = { faces[fi], faces[fi + 1], faces[fi + 2] };
        Vertex* va[3] = { vertex_linear[indices[0]], vertex_linear[indices[1]], vertex_linear[indices[2]] };

        if (check_valid_face(mesh, va, 3) == true)
        {
            Face* f = mesh_create_face(mesh, va, 3);
        }
        else
        {
            printf("Invalid Face %d", fi / 3);
        }
    }

    return mesh;
}

Mesh* mesh_clone(Mesh* mesh)
{
    PoolAllocator* allocator = new PoolAllocator();
    Mesh* clone = (Mesh*)allocator->Alloc(sizeof(Mesh));
    new (clone) Mesh(allocator);

    for (TUnorderedMap<uint32_t, Vertex*>::iterator it = mesh->vertex_map.begin();
        it != mesh->vertex_map.end();
        ++it)
    {
        Vertex* cv = alloc_vertex(clone);
        cv->id = it->second->id;
        cv->point = it->second->point;

        clone->num_vertex += 1;
        clone->vertex_map[cv->id] = cv;
    }

    TVector<Vertex*> va(allocator);
    va.reserve(6);
    for (TUnorderedMap<uint32_t, Face*>::iterator it = mesh->face_map.begin();
        it != mesh->face_map.end();
        ++it)
    {
        Face* f = it->second;
        face_get_vertices(f, va);

        for (int i = 0; i < (int)va.size(); ++i)
        {
            va[i] = clone->vertex_map[va[i]->id];
        }

        Face* cf = mesh_create_face(clone, va.data(), (int)va.size());
    }

    return clone;
}

void mesh_destroy(Mesh* mesh)
{
    PoolAllocator* allocator = mesh->allocator;

    for (TUnorderedMap<uint32_t, Vertex*>::iterator it = mesh->vertex_map.begin(); 
        it != mesh->vertex_map.end();
        ++it)
    {
        free_vertex(mesh, it->second);
    }
    mesh->vertex_map.clear();
    mesh->num_vertex = 0;

    while (mesh->face_map.empty() == false)
    {
        mesh_destroy_face(mesh, mesh->face_map.begin()->second);
    }

    mesh->~Mesh();
    allocator->Free(mesh);
    delete allocator;
}

Vertex* mesh_create_vertex(Mesh* mesh)
{
    Vertex* v = alloc_vertex(mesh);

    PointerKey pk = { v };
    v->id = std::hash<PointerKey>()(pk);

    assert(mesh->vertex_map.count(v->id) == 0);

    mesh->num_vertex += 1;
    mesh->vertex_map[v->id] = v;

    return v;
}

void mesh_destroy_vertex(Mesh* mesh, Vertex* v)
{
    mesh->num_vertex -= 1;
    mesh->vertex_map.erase(v->id);
    free_vertex(mesh, v);
}

Face* mesh_create_face(Mesh* mesh, Vertex** va, int va_count)
{
    Face* f = alloc_face(mesh);
    
    // use this pointer hash as id because the author says using the face id can be unsafe.
    PointerKey pk = { f };
    f->id = std::hash<PointerKey>()(pk); 

    HalfEdge* he_prev = NULL;
    for (int vi = 0; vi < va_count; ++vi)
    {
        Vertex* v2 = va[vi + 1 == va_count ? 0 : vi + 1];
        HalfEdge* he = alloc_halfedge(mesh);

        he->prev = he_prev;
        // he->next is set below
        he->vert = v2;
        mesh_halfedge_enter(mesh, he, va[vi]); // he->sym and he->edge are set in here
        he->face = f;
        he_prev = he;
    }

    HalfEdge* he_last = he_prev;
    for (;;)
    {
        HalfEdge* he_pprev = he_prev->prev;
        if (he_pprev == NULL)
            break;
        he_pprev->next = he_prev;
        he_prev = he_pprev;
    }
    he_prev->prev = he_last;
    he_last->next = he_prev;

    f->herep = he_last; // such that f->herep->vert == va[0]

    assert(mesh->face_map.count(f->id) == 0);

    mesh->face_map[f->id] = f;
    mesh->num_face += 1;

    return f;
}

void mesh_destroy_face(Mesh* mesh, Face* f)
{
    assert(f->herep != NULL);

    HalfEdge* he = f->herep;
    HalfEdge* hef = he;
    Vertex* v1 = he->prev->vert;
    for (;;)
    {
        HalfEdge* hen = he->next;
        Vertex* v1n = he->vert;
        mesh_halfedge_remove(mesh, he, v1);
        free_halfedge(mesh, he);
        he = hen;
        v1 = v1n;
        if (he == hef) break;
    }

    mesh->face_map.erase(f->id);
    free_face(mesh, f);

    mesh->num_face -= 1;
}

void mesh_halfedge_enter(Mesh* mesh, HalfEdge* he, Vertex* v1)
{
    Vertex* v2 = he->vert;

    v1->arhe.push_back(he);

    HalfEdge* hes = he->sym = halfedge_query(v2, v1);
    if (hes != NULL)
    {
        hes->sym = he;
        Edge* e = hes->edge;
        he->edge = e;
        if (he->vert->id > hes->vert->id)
        {
            e->herep = he;
        }
    }
    else
    {
        Edge* e = alloc_edge(mesh);
        e->herep = he;
        he->edge = e;

        mesh->num_edges += 1;
    }
}

void mesh_halfedge_remove(Mesh* mesh, HalfEdge* he, Vertex* v1)
{
    Vertex* v2 = he->vert;
    Edge* e = he->edge;
    HalfEdge* hes = he->sym;

    if (hes)
    {
        hes->sym = NULL;
        if (e->herep == he) e->herep = hes;
    }
    else
    {
        mesh->num_edges -= 1;
        free_edge(mesh, e);
    }
    
    for (TVector<HalfEdge*>::iterator it = v1->arhe.begin();
        it != v1->arhe.end();
        ++it)
    {
        if (*it == he)
        {
            v1->arhe.erase(it);
            break;
        }
    }
}

float mesh_face_get_area(Mesh* m, Face* f)
{
    if (face_is_triangle(f))
    {
        Vertex* va[3];
        face_get_triangle_vertices(f, va);
        return triangle_area(va[0]->point, va[1]->point, va[2]->point);
    }
    else
    {
        Polygon* p = polygon_create_from_face(m, f);
        float area = polygon_get_area(p);
        polygon_destroy(p);

        return area;
    }
}

void mesh_create_bogus_halfedges(Mesh* m, TVector<HalfEdge*>& ar_he)
{
    for (int i = 0; i < (int)ar_he.size(); ++i)
    {
        HalfEdge*& he = ar_he[i];

        if (halfedge_is_boundray(he) == true)
        {
            HalfEdge* heo = he;

            Vertex* v1 = heo->vert;

            he = alloc_halfedge(m);
            he->vert = heo->prev->vert;
            he->prev = NULL;
            he->next = NULL;
            he->face = (Face*)v1; // temporary overload
            mesh_halfedge_enter(m, he, v1);
        }
        else
        {
            he = NULL;
        }
    }
}

void mesh_destroy_bogus_halfedge(Mesh* m, TVector<HalfEdge*>& ar_he)
{
    for (HalfEdge* he : ar_he)
    {
        if (he == NULL)
            continue;

        Vertex* v1 = (Vertex*)he->face; // temporary overload
        mesh_halfedge_remove(m, he, v1); 
        free_halfedge(m, he);
    }
}

void mesh_collapse_edge(Mesh* m, Edge* e)
{
    Vertex* vs = edge_get_vertex1(e);
    Vertex* vt = vertex_get_opposite_from_edge(vs, e);
    int is_boundary_vs = vertex_is_boundary(vs);
    int is_boundary_vt = vertex_is_boundary(vt);
    int sum_boundary = is_boundary_vs + is_boundary_vt;
    Vector3 p = sum_boundary == 0 || sum_boundary == 2 ?
        vector3_add
        (
            vector3_mul_scalar(vs->point, 0.5f),
            vector3_mul_scalar(vt->point, 0.5f)
        ) :
        is_boundary_vs != 0 ? vs->point : vt->point;

    /* add this after eflag_sharp
    TUnorderedSet<uint32_t> vsharp(m->allocator);
    EdgeVertexIteration evi = edge_vertex_get_iteration(vt);
    while (Edge* ee = edge_vertex_iterate(&evi))
    {
        if (ee != e // edge sharp)
        {
            
        }
    }*/

    assert(mesh_is_legal_edge_collapse(m, e) == true);
    HalfEdge* he1 = halfedge_get_from_ev1(e, vs);
    HalfEdge* he2 = halfedge_get_from_ev2(e, vs);
    assert(he1 == NULL ? he2 != NULL : true);

    vt = he1 ? he1->vert : he2->prev->vert;
    assert(vt == vertex_get_opposite_from_edge(vs, e));

    // Create bogus halfedges if boundaries
    TVector<HalfEdge*> ar_he(m->allocator);
    if (he1)
        ar_he.push_back(he1->prev);
    if (he2)
        ar_he.push_back(he2->next);
    mesh_create_bogus_halfedges(m, ar_he);
    if (he1)
    {
        assert(face_is_triangle(he1->face) == true);
        mesh_destroy_face(m, he1->face);
    }

    if (he2)
    {
        assert(face_is_triangle(he2->face) == true);
        mesh_destroy_face(m, he2->face);
    }

    // Change remaining faces around vt to have vs instead
    TVector<HalfEdge*> ar_corners(m->allocator);
    ar_corners.resize(vt->arhe.size());
    for (int i = 0; i < (int)vt->arhe.size(); ++i)
    {
        HalfEdge* corner = vt->arhe[i]->prev;
        ar_corners[i] = corner;
    }

    for (HalfEdge* he : ar_corners)
    {
        // ends up deleting and recreating MEdge structures, which is great.
        mesh_halfedge_remove(m, he, he->prev->vert);
        mesh_halfedge_remove(m, he->next, he->vert);
        he->vert = vs;
        mesh_halfedge_enter(m, he, he->prev->vert);
        mesh_halfedge_enter(m, he->next, he->vert);
    }
    mesh_destroy_vertex(m, vt);
    mesh_destroy_bogus_halfedge(m, ar_he);

    vs->point = p;

    /*
    * add this after eflag_sharp
    * for(Vertex* v : vsharp)
    * {
    * }
    * 
    * edge...
    */
}

Vertex* mesh_split_edge(Mesh* m, Edge* e)
{
    Vertex* v1 = edge_get_vertex1(e);
    Vertex* v2 = edge_get_vertex2(e);
    Face* f1 = edge_get_face1(e);
    Face* f2 = edge_get_face2(e); // f2 could be NULL
    Vertex* vo1 = edge_get_side_vertex1(e); // implies triangles
    Vertex* vo2 = edge_get_side_vertex2(e); // implies triangles

    // create bogus hedges if boundaries
    TVector<HalfEdge*> ar_he(m->allocator);
    CornerFaceIteration cfi = corner_face_get_iteration(f1);
    while (HalfEdge* he = corner_face_iterate(&cfi))
    {
        if (he->edge != e)
            ar_he.push_back(he);
    }

    if (f2)
    {
        cfi = corner_face_get_iteration(f2);
        while (HalfEdge* he = corner_face_iterate(&cfi))
        {
            if (he->edge != e)
                ar_he.push_back(he);
        }
    }

    mesh_create_bogus_halfedges(m, ar_he);
    mesh_destroy_face(m, f1);
    if (f2)
    {
        mesh_destroy_face(m, f2);
    }

    Vertex* vn = mesh_create_vertex(m);
    Vertex* tri_vertices[3];
    
    tri_vertices[0] = vn, tri_vertices[1] = v2, tri_vertices[2] = vo1;
    mesh_create_face(m, tri_vertices, 3);

    tri_vertices[0] = vn, tri_vertices[1] = vo1, tri_vertices[2] = v1;
    mesh_create_face(m, tri_vertices, 3);

    if (vo2)
    {
        tri_vertices[0] = vn, tri_vertices[1] = v1, tri_vertices[2] = vo2;
        mesh_create_face(m, tri_vertices, 3);

        tri_vertices[0] = vn, tri_vertices[1] = vo2, tri_vertices[2] = v2;
        mesh_create_face(m, tri_vertices, 3);
    }

    mesh_destroy_bogus_halfedge(m, ar_he);

    vn->point = vector3_add
    (
        vector3_mul_scalar(v1->point, 0.5f),
        vector3_mul_scalar(v2->point, 0.5f)
    );

    return vn;
}

Edge* mesh_swap_edge(Mesh* m, Edge* e)
{
    Vertex* v1 = edge_get_vertex1(e);
    Vertex* v2 = edge_get_vertex2(e);
    Face* f1 = edge_get_face1(e);
    Face* f2 = edge_get_face2(e);
    
    // TODO : fstring processing?

    assert(mesh_is_legal_edge_swap(m, e) == true);
    Vertex* vo1 = edge_get_side_vertex1(e); // implies triangles
    Vertex* vo2 = edge_get_side_vertex2(e); 

    // Create bogus half edges if boundaries
    TVector<HalfEdge*> ar_he(m->allocator);
    CornerFaceIteration cfi = corner_face_get_iteration(f1);
    while (HalfEdge* corner = corner_face_iterate(&cfi))
        ar_he.push_back(corner);
    cfi = corner_face_get_iteration(f2);
    while (HalfEdge* corner = corner_face_iterate(&cfi))
        ar_he.push_back(corner);

    mesh_create_bogus_halfedges(m, ar_he);

    mesh_destroy_face(m, f1);
    mesh_destroy_face(m, f2);

    Vertex* tri_verts[3];
    tri_verts[0] = v1, tri_verts[1] = vo2, tri_verts[2] = vo1;
    mesh_create_face(m, tri_verts, 3);

    tri_verts[0] = v2, tri_verts[1] = vo1, tri_verts[2] = vo2;
    mesh_create_face(m, tri_verts, 3);

    mesh_destroy_bogus_halfedge(m, ar_he);

    Edge* new_edge = edge_query(vo1, vo2);
    return new_edge;
}

static inline int binary_search(const TVector<float>& arr, int low, int high, float value)
{
    while ((high - low) > 1)
    {
        int mid = (low + high) / 2;
        float mid_v = arr[mid];

        if (mid_v >= value)
            high = mid;
        else
            low = mid;
    }

    return low;
}

static inline Vector3 interpolate_barycentric(Vector3 a, Vector3 b, Vector3 c, Vector3 bary)
{
    Vector3 p;
    for (int i = 0; i < 3; ++i)
    {
        p.v[i] = bary.v[0] * a.v[i] + bary.v[1] * b.v[i] + bary.v[2] * c.v[i];
    }

    return p;
}

void mesh_sample_random_points
(
    Mesh* mesh, 
    int sample_count, 
    unsigned* seed_pointer,
    std::vector<Vector3>& output_points, 
    std::vector<Vector3>& output_normals
)
{
    output_points.clear();
    output_points.resize(sample_count);

    output_normals.clear();
    output_normals.resize(sample_count);

    int nf = mesh->num_face;
    
    TVector<Face*> faces(mesh->allocator);
    TVector<float> face_areas(mesh->allocator);
    TVector<float> cumulative_areas(mesh->allocator);
    float sum_area = 0.f;
    faces.reserve(nf);
    face_areas.reserve(nf);
    cumulative_areas.reserve(nf + 1);

    for (TUnorderedMap<uint32_t, Face*>::iterator it = mesh->face_map.begin();
        it != mesh->face_map.end();
        ++it)
    {
        Face* f = it->second;
        if (face_is_triangle(f) == false)
        {
            continue;
        }

        faces.push_back(f);

        float face_area = mesh_face_get_area(mesh, f);
        face_areas.push_back(face_area);

        sum_area += face_area;
    }

    double double_cumulative_area = 0.0;
    for (int i = 0; i < nf; ++i)
    {
        cumulative_areas.push_back((float)double_cumulative_area);
        double_cumulative_area += face_areas[i] / sum_area;
    }
    cumulative_areas.push_back(1.00001f);


    TUnorderedMap<uint32_t, VertexNormal*> vertex_to_normals(mesh->allocator);
    for (TUnorderedMap<uint32_t, Vertex*>::iterator it = mesh->vertex_map.begin();
        it != mesh->vertex_map.end();
        ++it)
    {
        uint32_t vertex_id = it->first;
        Vertex* v = it->second;

        VertexNormal* vn = vertexnormal_create(mesh, v, VERTEX_NORMAL_ANGLE);
        vertex_to_normals[vertex_id] = vn;
    }

    RandomGen* rg = randomgen_create(mesh->allocator, seed_pointer);
    Vertex* va[3];
    Vector3 bary;
    for (int i = 0; i < sample_count; ++i)
    {
        int fi = binary_search(cumulative_areas, 0, nf, randomgen_get_uniform(rg));
        Face* f = faces[fi];
        face_get_triangle_vertices(f, va);

        bary.v[0] = randomgen_get_uniform(rg);
        bary.v[1] = randomgen_get_uniform(rg);
        if (bary.v[0] + bary.v[1] > 1.f)
        {
            bary.v[0] = 1.f - bary.v[0];
            bary.v[1] = 1.f - bary.v[1];
        }
        bary.v[2] = 1.f - bary.v[0] - bary.v[1];

        Vector3 p = interpolate_barycentric(va[0]->point, va[1]->point, va[2]->point, bary);
        Vector3 normal = vector3_set1(0.f);

        for (int j = 0; j < 3; ++j)
        {
            VertexNormal* vn = vertex_to_normals[va[j]->id];
            if (vn == NULL)
                continue;

            Vector3 face_normal = vn->face_to_normal[f->id];

            normal = vector3_add(normal, vector3_mul_scalar(face_normal, bary.v[j]));
        }
        normal = vector3_normalize(normal);

        output_points[i] = p;
        output_normals[i] = normal;
    }

    randomgen_destroy(rg);
    for (TUnorderedMap<uint32_t, VertexNormal*>::iterator it = vertex_to_normals.begin();
        it != vertex_to_normals.end();
        ++it)
    {
        if (it->second)
            vertexnormal_destroy(it->second);
    }
}

void mesh_export_obj(Mesh* mesh, const char* path)
{
    TUnorderedMap<uint32_t, int> vertex_id_to_pos_index(mesh->allocator);

    PoolAllocator* allocator = mesh->allocator;
    Vector3* positions = (Vector3*)allocator->Alloc(sizeof(Vector3) * mesh->vertex_map.size());
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

    int* triangle_indices = (int*)allocator->Alloc(sizeof(int) * mesh->face_map.size() * 3);
    int index_count = 0;
    for (TUnorderedMap<uint32_t, Face*>::iterator it = mesh->face_map.begin();
        it != mesh->face_map.end();
        ++it)
    {
        Face* f = it->second;
        assert(face_is_triangle(f) == true);

        Vertex* va[3];
        face_get_triangle_vertices(f, va);

        triangle_indices[index_count] = vertex_id_to_pos_index[va[0]->id];
        triangle_indices[index_count + 1] = vertex_id_to_pos_index[va[1]->id];
        triangle_indices[index_count + 2] = vertex_id_to_pos_index[va[2]->id];
        
        index_count += 3;
    }

    FileBuffer fb;
    filebuffer_open(&fb, path, "wb");
    for (int i = 0; i < position_count; ++i)
    {
        filebuffer_write_str(&fb, "v ");

        Vector3 pos = positions[i];
        filebuffer_write_float_str(&fb, pos.v[0]);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_float_str(&fb, pos.v[1]);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_float_str(&fb, pos.v[2]);
        filebuffer_write_str(&fb, "\n");
    }

    for (int i = 0; i < index_count; i += 3)
    {
        filebuffer_write_str(&fb, "f ");
        
        filebuffer_write_int_str(&fb, triangle_indices[i] + 1);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_int_str(&fb, triangle_indices[i + 1] + 1);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_int_str(&fb, triangle_indices[i + 2] + 1);
        filebuffer_write_str(&fb, "\n");
    }

    filebuffer_close(&fb);

    allocator->Free(triangle_indices);
    allocator->Free(positions);
}

void mesh_export_obj_with_colors(Mesh* mesh, const char* path, Vector3* colors, int color_count)
{
    assert(mesh->vertex_map.size() == (size_t)color_count);

    TUnorderedMap<uint32_t, int> vertex_id_to_pos_index(mesh->allocator);

    PoolAllocator* allocator = mesh->allocator;
    Vector3* positions = (Vector3*)allocator->Alloc(sizeof(Vector3) * mesh->vertex_map.size());
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

    int* triangle_indices = (int*)allocator->Alloc(sizeof(int) * mesh->face_map.size() * 3);
    int index_count = 0;
    for (TUnorderedMap<uint32_t, Face*>::iterator it = mesh->face_map.begin();
        it != mesh->face_map.end();
        ++it)
    {
        Face* f = it->second;
        assert(face_is_triangle(f) == true);

        Vertex* va[3];
        face_get_triangle_vertices(f, va);

        triangle_indices[index_count] = vertex_id_to_pos_index[va[0]->id];
        triangle_indices[index_count + 1] = vertex_id_to_pos_index[va[1]->id];
        triangle_indices[index_count + 2] = vertex_id_to_pos_index[va[2]->id];

        index_count += 3;
    }

    FileBuffer fb;
    filebuffer_open(&fb, path, "wb");
    for (int i = 0; i < position_count; ++i)
    {
        Vector3 pos = positions[i];
        Vector3 color = colors[i];

        filebuffer_write_str(&fb, "v ");

        filebuffer_write_float_str(&fb, pos.v[0]);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_float_str(&fb, pos.v[1]);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_float_str(&fb, pos.v[2]);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_float_str(&fb, color.v[0]);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_float_str(&fb, color.v[1]);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_float_str(&fb, color.v[2]);
        filebuffer_write_str(&fb, "\n");
    }


    for (int i = 0; i < index_count; i += 3)
    {
        filebuffer_write_str(&fb, "f ");

        filebuffer_write_int_str(&fb, triangle_indices[i] + 1);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_int_str(&fb, triangle_indices[i + 1] + 1);
        filebuffer_write_str(&fb, " ");

        filebuffer_write_int_str(&fb, triangle_indices[i + 2] + 1);
        filebuffer_write_str(&fb, "\n");
    }

    filebuffer_close(&fb);

    allocator->Free(triangle_indices);
    allocator->Free(positions);
}

bool mesh_is_sharp_ve(Mesh* mesh, Vertex* v, Edge* e)
{
    if (edge_is_boundary(e) == true)
        return true;
    // TODO: 
    // edge flag check.
    // string key check
    return false;
}

bool mesh_is_legal_edge_collapse(Mesh* mesh, Edge* e)
{
    /*
    * Chan : check whether there are duplicated faces adjacent to the edge.
    */
    Vertex* v1 = edge_get_vertex1(e);
    Vertex* v2 = edge_get_vertex2(e);
    Vertex* vo1 = edge_get_side_vertex1(e);
    Vertex* vo2 = edge_get_side_vertex2(e); // vo2 may be NULL
    /*
    * Check that substituting v2 to v1 will not duplicate an edge in any faces adjacent to v2 (besides f1 and f2)
    * (case of Face vertices being duplicated cannot happen here
    * since only f1 and f2 can have both v1 and v2)
    */
    VertexIteration vi = vertex_get_iteration(v2);
    while (Vertex* v = vertex_iterate(&vi))
    {
        if (v == v1 || v == vo1 || v == vo2)
            continue;

        if (edge_query(v, v1))
            return false;
    }

    return true;
}

bool mesh_is_nice_edge_collapse(Mesh* mesh, Edge* e)
{
    if (mesh_is_legal_edge_collapse(mesh, e) == false)
        return false;

    Vertex* v1 = edge_get_vertex1(e);
    Vertex* v2 = edge_get_vertex2(e);
    Face* f1 = edge_get_face1(e);
    Face* f2 = edge_get_face2(e);
    assert(face_is_triangle(f1) == true);
    assert(f2 ? face_is_triangle(f2) : true);

    /*
    * Chan : Second Condition of Legal Edge Collapse 
    * If {i} and {j} are both boundary vertices, {i, j} is a boundary edge.
    */
    if (edge_is_boundary(e) == false && (vertex_is_boundary(v1) && vertex_is_boundary(v2)))
        return false;

    /*
    * Chan : First Condition of Legal Edge Collapse
    */
    Vertex* vo1 = edge_get_side_vertex1(e);
    Vertex* vo2 = edge_get_side_vertex2(e);
    TSet<uint32_t> set(mesh->allocator);
    
    VertexIteration vi = vertex_get_iteration(v1);
    while (Vertex* vv = vertex_iterate(&vi))
    {
        if (vv != vo1 && vv != vo2)
            set.insert(vv->id);
    }

    vi = vertex_get_iteration(v2);
    while (Vertex* vv = vertex_iterate(&vi))
    {
        if (vv != vo1 && vv != vo2)
        {
            std::pair<TSet<uint32_t>::iterator, bool> r = set.insert(vv->id);
            if (r.second == false)
                return false;
        }
    }

    /*
    * Chan : Third Condition of Legal Edge Collapse
    * K has more than 4 vertices if neither {i} nor {j} are boundary vertices,
    * or K has more than 3 vertices if either {i} or {j} are boundary vertices.
    */ 
    /*
    * if set.size() is 2, it means the set only includes the v1 and v2.
    */
    if (set.size() == 2 && edge_is_boundary(e) == true) // single face case 
        return false;
    if (set.size() == 2 && vertex_is_boundary(v1) == false && vertex_is_boundary(v2) == false) // tetrahedron
        return false; // vertex_is_boundary just checks that the halfedge has the symmetrical half edge.

    return true;
}

bool mesh_is_legal_edge_swap(Mesh* mesh, Edge* e)
{
    if (edge_is_boundary(e) == true)
        return false;

    // illegal if cross edge already exists (as in tetrahedron)
    if (edge_query(edge_get_side_vertex1(e), edge_get_side_vertex2(e)) != NULL)
        return false;

    return true;
}

float mesh_get_min_dihedral_about_vertex(Mesh* m, Vertex* v)
{
    TVector<Vertex*> vertex_rings(m->allocator);
    vertex_get_ring(v, vertex_rings);

    int nw = (int)vertex_rings.size();
    bool open = vertex_rings[0] != vertex_rings[nw - 1];
    float min_dic = 2.f;
    for (int i = 1; i < nw - open; ++i)
    {
        int i1 = i + 1;
        if (i1 == nw) 
            i1 = 1;

        float dic = dihedral_angle_cos(v->point, vertex_rings[i]->point, vertex_rings[i - 1]->point, vertex_rings[i1]->point);
        
        if (dic < min_dic)
            min_dic = dic;
    }

    return min_dic;
}

Vertex* vertex_get_opposite_from_edge(Vertex* v, Edge* e)
{
    Vertex* v1 = e->herep->prev->vert;
    Vertex* v2 = e->herep->vert;
    if (v == v1) return v2;
    if (v == v2) return v1;

    assert(true);
    return NULL;
}

Vertex* vertex_get_opposite_from_ef(Edge* e, Face* f)
{
    HalfEdge* he = halfedge_get_from_ef(e, f);
    assert(he->next->next->next == he); // mesh face is not triangle
    return he->next->vert;
}

Vertex* vertex_get_most_clw(Vertex* v)
{
    HalfEdge* he = halfedge_get_most_clw(v);
    return he ? he->next->vert : NULL;
}

Vertex* vertex_get_clw(Vertex* v, Vertex* next)
{
    HalfEdge* he = halfedge_query(next, v);
    if (he)
        return he->next->vert;
    else
        return NULL;
}

Vertex* vertex_get_ccw(Vertex* v, Vertex* next)
{
    HalfEdge* he = halfedge_query(v, next);
    if (he)
        return he->prev->prev->vert;
    else
        return NULL;
}

Vertex* vertex_get_clw_from_fv(Face* f, Vertex* v)
{
    return halfedge_get_from_vf(v, f)->prev->vert;
}

Vertex* vertex_get_ccw_from_fv(Face* f, Vertex* v)
{
    return halfedge_get_from_vf(v, f)->next->vert;
}

int vertex_get_num_boundaries(Vertex* v)
{
    int n = 0;
    for (HalfEdge* he : v->arhe)
    {
        if (halfedge_is_boundray(he) == true)
        {
            ++n;
        }
    }

    return n;
}

int vertex_get_num_sharp_edges(Vertex* v)
{
    int n = 0;
    EdgeVertexIteration evi = edge_vertex_get_iteration(v);
    while (Edge* e = edge_vertex_iterate(&evi))
    {
        if (edge_is_sharp(e) == true)
            ++n;
    }

    return n;
}

bool vertex_is_boundary(Vertex* v)
{
    assert(v->arhe.size() != 0); // isolated vertex
    int nb = vertex_get_num_boundaries(v);
    assert(nb < 2); // non-nice vertex. hmm I am not sure about this assert.
    return nb > 0;
}

void vertex_get_ring(Vertex* v, TVector<Vertex*>& out_vertex_points)
{
    out_vertex_points.clear();

    Vertex* w = vertex_get_most_clw(v);
    Vertex* wf = w;

    for (;;)
    {
        out_vertex_points.push_back(w);
        w = vertex_get_ccw(v, w);
        if (w == NULL || w == wf)
            break;
    }

    if (w != NULL)
        out_vertex_points.push_back(w);
}

VertexIteration vertex_get_iteration(Vertex* v)
{
    VertexIteration vi;
    vi.cur = v->arhe.begin();
    vi.end = v->arhe.end();
    vi.extra_vertex = NULL;

    return vi;
}

Vertex* vertex_iterate(VertexIteration* vi)
{
    if (vi->extra_vertex)
    {
        Vertex* v = vi->extra_vertex;
        vi->extra_vertex = NULL;
        return v;
    }

    if (vi->cur == vi->end)
    {
        return NULL;
    }

    if ((*vi->cur)->prev->sym == NULL)
        vi->extra_vertex = (*vi->cur)->prev->prev->vert;

    Vertex* v = (*vi->cur)->vert;
    ++(vi->cur);
    return v;
}

HalfEdge* halfedge_query(Vertex* v1, Vertex* v2)
{
    for (HalfEdge* he : v1->arhe)
    {
        if (he->vert == v2)
            return he;
    }

    return NULL;
}

HalfEdge* halfedge_get_corner(Vertex* v, Face* f)
{
    for (HalfEdge* he : v->arhe)
    {
        if (he->prev->face == f)
            return he->prev;
    }

    return NULL;
}

HalfEdge* halfedge_get_clw(HalfEdge* he)
{
    return he->next->sym;
}

HalfEdge* halfedge_get_ccw(HalfEdge* he)
{
    return he->sym ? he->sym->prev : NULL;
}

HalfEdge* halfedge_get_most_clw(Vertex* v)
{
    if (v->arhe.size() == 0)
        return NULL;

    HalfEdge* he = v->arhe[0]->prev;
    for (HalfEdge* hef = he;;)
    {
        HalfEdge* hen = halfedge_get_clw(he);
        if (hen == NULL || hen == hef)
            break;
        he = hen;
    }
    return he;
}

HalfEdge* halfedge_get_from_vf(Vertex* v, Face* f)
{
    for (HalfEdge* he : v->arhe)
    {
        HalfEdge* corner = he->prev;
        if (corner->face == f)
            return he;
    }

    assert(0);
    return NULL;
}

HalfEdge* halfedge_get_from_ef(Edge* e, Face* f)
{
    if (edge_get_face1(e) == f) return e->herep;
    if (edge_get_face2(e) == f) return e->herep->sym;
    assert(0);
    return NULL;
}

HalfEdge* halfedge_get_from_ev1(Edge* e, Vertex* v)
{
    if (edge_get_vertex1(e) == v) 
        return e->herep;

    if (edge_get_vertex2(e) == v)
        return e->herep->sym;

    assert(0);
    return NULL;
}

HalfEdge* halfedge_get_from_ev2(Edge* e, Vertex* v)
{
    if (edge_get_vertex1(e) == v)
        return e->herep->sym;

    if (edge_get_vertex2(e) == v)
        return e->herep;

    assert(0);
    return NULL;
}

bool halfedge_is_boundray(HalfEdge* he)
{
    return he->sym == NULL;
}

CornerFaceIteration corner_face_get_iteration(Face* f)
{
    CornerFaceIteration cfi;
    cfi.cur = f->herep;
    cfi.end = f->herep;
    cfi.first = true;

    return cfi;
}

HalfEdge* corner_face_iterate(CornerFaceIteration* cfi)
{
    if (cfi->first == false && cfi->cur == cfi->end)
        return NULL;

    if (cfi->first == true)
        cfi->first = false;

    HalfEdge* cur = cfi->cur;
    cfi->cur = cur->next;

    return cur;
}

bool face_is_triangle(Face* f)
{
    HalfEdge* he = f->herep;
    return he->next->next->next == he;
}

void face_get_vertices(Face* f, TVector<Vertex*>& va)
{
    HalfEdge* he = f->herep;
    HalfEdge* he0 = he;

    va.clear();
    do
    {
        va.push_back(he->vert);
        he = he->next;
    } while (he != he0);
}

void face_get_triangle_vertices(Face* f, Vertex** arr)
{
    HalfEdge* he = f->herep;
    HalfEdge* he0 = he;

    arr[0] = he->vert; 
    he = he->next;
    arr[1] = he->vert;
    he = he->next;
    arr[2] = he->vert;

    assert(he->next == he0);
}

Face* face_get_opposite_from_edge(Face* f, Edge* e)
{
    Face* f1 = e->herep->face;
    Face* f2 = e->herep->sym ? e->herep->sym->face : NULL;
    if (f == f1) return f2;
    if (f == f2) return f1;
    assert(true);
    return NULL;
}

bool edge_is_boundary(Edge* e)
{
    return e->herep->sym == NULL;
}

bool edge_is_sharp(Edge* e)
{
    /*
    * TODO : edge sharp flag
    * Refer to the record_sharpe() function from Filtermesh.cpp
    */
    return edge_is_boundary(e) || e->is_sharp;
}

Edge* edge_query(Vertex* v, Vertex* w)
{
    HalfEdge* he = halfedge_query(v, w);
    if (he)
        return he->edge;
    he = halfedge_query(w, v);
    if (he)
        return he->edge;
    return NULL;
}

Edge* edge_get_clw(Face* f, Vertex* v)
{
    HalfEdge* corner_he = halfedge_get_corner(v, f);
    return corner_he->edge;
}

Edge* edge_get_ccw(Face* f, Vertex* v)
{
    HalfEdge* corner_he = halfedge_get_corner(v, f);
    return corner_he->next->edge;
}

Vertex* edge_get_vertex1(Edge* e)
{
    return e->herep->prev->vert;
}

Vertex* edge_get_vertex2(Edge* e)
{
    return e->herep->vert;
}

Vertex* edge_get_side_vertex1(Edge* e)
{
    return vertex_get_opposite_from_ef(e, edge_get_face1(e));
}

Vertex* edge_get_side_vertex2(Edge* e)
{
    Face* f2 = edge_get_face2(e);
    return f2 ? vertex_get_opposite_from_ef(e, f2) : NULL;
}

Face* edge_get_face1(Edge* e)
{
    return e->herep->face;
}

Face* edge_get_face2(Edge* e)
{
    HalfEdge* he = e->herep;
    return he->sym ? he->sym->face : NULL;
}

Edge* edge_get_opposite_from_vf(Vertex* v, Face* f)
{
    assert(face_is_triangle(f) == true);
    return halfedge_get_from_vf(v, f)->prev->edge;
}

void edge_get_ring(Edge* e, TVector<Vertex*>& out_vertex_points)
{
    out_vertex_points.clear();

    Vertex* v1 = edge_get_vertex1(e);
    Vertex* v2 = edge_get_vertex2(e);
    assert(edge_is_boundary(e) ? vertex_get_most_clw(v2) != v1 : true);

    Vertex* cv = v2;
    if (edge_is_boundary(e) == false && vertex_is_boundary(v1) == true)
        cv = v1;

    Vertex* ov = vertex_get_opposite_from_edge(cv, e);
    Vertex* w = vertex_get_most_clw(cv);
    if (w == ov)
    {
        w = vertex_get_clw(cv, w);
        
        assert(vertex_is_boundary(cv) == false);
    }
    assert(w != NULL);

    Vertex* wf = w;
    
    for (;;)
    {
        out_vertex_points.push_back(w);
        Vertex* w2 = vertex_get_ccw(cv, w);
        if (w2 == ov)
        {
            ov = cv;
            cv = w2;
            w2 = vertex_get_ccw(cv, w);
        }
        w = w2;

        if (w == NULL || w == wf)
            break;
    }

    if (w == wf)
        out_vertex_points.push_back(w);

    assert(out_vertex_points.size() >= (out_vertex_points[0] == out_vertex_points.back() ? 4 : 2));
}

void edge_get_faces(Edge* e, TVector<Face*>& out_faces)
{
    out_faces.push_back(e->herep->face);
    
    if (e->herep->sym != NULL)
        out_faces.push_back(e->herep->sym->face);
}

void edge_get_both_faces(Edge* e, Face** out_faces)
{
    out_faces[0] = e->herep->face;
    out_faces[1] = e->herep->sym ? e->herep->sym->face : NULL;
}

void edge_get_both_vertices(Edge* e, Vertex** out_vertices)
{
    out_vertices[0] = e->herep->vert;
    out_vertices[1] = e->herep->prev->vert;
}

EdgeIteration edge_get_iteration(Mesh* m)
{
    EdgeIteration ei;
    ei.vcur = m->vertex_map.begin();
    ei.vend = m->vertex_map.end();
    ei.hcur = NULL;
    ei.hend = NULL;
    return ei;
}

Edge* edge_iterate(EdgeIteration* input)
{
    if (input->hcur)
        ++(input->hcur);

    if (input->vcur == input->vend)
    {
        return NULL;
    }

    for (;;)
    {
        if (input->hcur != input->hend)
        {
            if ((*input->hcur)->edge->herep != *input->hcur)
            {
                ++input->hcur;
                continue;
            }

            return (*input->hcur)->edge; // found the edge!
        }

        if (input->vcur == input->vend)
        {
            return NULL;
        }

        input->hcur = &(input->vcur->second->arhe[0]);
        input->hend = input->hcur + input->vcur->second->arhe.size();
        ++input->vcur;
    }

    return NULL;
}

EdgeVertexIteration edge_vertex_get_iteration(Vertex* v)
{
    EdgeVertexIteration evi;
    evi.cur = v->arhe.begin();
    evi.end = v->arhe.end();
    evi.extra_edge = NULL;
    return evi;
}

Edge* edge_vertex_iterate(EdgeVertexIteration* evi)
{
    if (evi->extra_edge)
    {
        Edge* e = evi->extra_edge;
        evi->extra_edge = NULL;
        return e;
    }

    if (evi->cur == evi->end)
        return NULL;

    if ((*evi->cur)->prev->sym == NULL)
        evi->extra_edge = (*evi->cur)->prev->edge;

    Edge* e = (*evi->cur)->edge;
    ++(evi->cur);

    return e;
}

EdgeFaceIteration edge_face_get_iteration(Face* f)
{
    EdgeFaceIteration efi;
    efi.cur = f->herep;
    efi.end = f->herep;
    efi.first = true;

    return efi;
}

Edge* edge_face_iterate(EdgeFaceIteration* efi)
{
    if (efi->first == false && efi->cur == efi->end)
        return NULL;

    if(efi->first == true)
        efi->first = false;

    Edge* e = efi->cur->edge;

    efi->cur = efi->cur->next;

    return e;
}

Polygon* polygon_create_from_face(Mesh* mesh, Face* f)
{
    Polygon* p = (Polygon*)mesh->allocator->Alloc(sizeof(Polygon));
    memset(p, 0, sizeof(Polygon));

    p->allocator = mesh->allocator;
    p->vertices = NULL;
    p->vertex_count = 0;

    HalfEdge* he = f->herep;
    HalfEdge* he0 = he;
    do
    {
        p->vertex_count += 1;
        he = he->next;
    } while (he != he0);

    p->vertices = (Vector3*)mesh->allocator->Alloc(sizeof(Vector3) * p->vertex_count);
    int i = 0;
    do
    {
        p->vertices[i] = he->vert->point;
        he = he->next;
        ++i;
    } while (he != he0);

    return p;
}

void polygon_destroy(Polygon* polygon)
{
    if (polygon->vertex_count > 0)
    {
        polygon->allocator->Free(polygon->vertices);
    }

    polygon->allocator->Free(polygon);
}

float polygon_get_area(Polygon* p)
{
    assert(p->vertex_count >= 3);

    float sum = 0.f;
    for (int i = 1; i < p->vertex_count - 1; ++i)
    {
        sum += triangle_area(p->vertices[0], p->vertices[i], p->vertices[i + 1]);
    }

    return sum;
}

Vector3 polygon_get_normal_sum(Polygon* p)
{
    if (p->vertex_count == 3)
        return vector3_cross
        (
            vector3_sub(p->vertices[1], p->vertices[0]),
            vector3_sub(p->vertices[2], p->vertices[0])
        );

    Vector3 normal = vector3_set1(0.f);
    for (int i = 1; i < p->vertex_count - 1; ++i)
    {
        normal = vector3_add
        (
            normal,
            vector3_cross
            (
                vector3_sub(p->vertices[i], p->vertices[0]),
                vector3_sub(p->vertices[i + 1], p->vertices[0])
            )
        );
    }

    return normal;
}

Vector3 polygon_get_normal(Polygon* p)
{
    Vector3 normal = vector3_normalize(polygon_get_normal_sum(p));
    return normal;
}

VertexNormal* vertexnormal_create(Mesh* mesh, Vertex* v, VertexNormalType type)
{
    VertexNormal* vn = (VertexNormal*)mesh->allocator->Alloc(sizeof(VertexNormal));
    new (vn) VertexNormal(mesh->allocator);
    vn->type = type;

    Vector3 vp = v->point;
    
    int nfaces = 0;
    TSet<uint32_t> setfvis(mesh->allocator);
    TVector<Vertex*> av(mesh->allocator);
    TVector<Face*> af(mesh->allocator);
    for (HalfEdge* he : v->arhe)
    {
        uint32_t face_key = he->face->id;
        Face* vf = he->face;

        vn->face_to_normal[face_key] = vector3_set1(0.f);

        ++nfaces;
    }

    if (nfaces == 0)
    {
        vn->~VertexNormal();
        mesh->allocator->Free(vn);
        return NULL;
    }

    for(HalfEdge* he : v->arhe)
    {
        uint32_t face_key = he->face->id;
        Face* frep = he->face;
        if (setfvis.count(face_key) != 0)
            continue;

        bool closed = false;
        av.clear();
        af.clear();
        Face* f = frep;

        for (;;) // find face in clockwise or frep if closed.
        {
            Edge* e = edge_get_ccw(f, v);
            if (mesh_is_sharp_ve(mesh, v, e) == true)
                break;
            f = face_get_opposite_from_edge(f, e);
            if (f == frep)
            {
                closed = true;
                break;
            }
        }

        for (;;) // now go ccw
        {
            av.push_back(vertex_get_opposite_from_edge(v, edge_get_ccw(f, v)));
            af.push_back(f);
            Edge* e = edge_get_clw(f, v);

            // TODO : is_cusp && nsharpte < 2 test

            if (closed == false && mesh_is_sharp_ve(mesh, v, e) == true)
            {
                Vertex* ov = vertex_get_opposite_from_edge(v, e);
                if (ov != av[0])
                    av.push_back(ov);
                break;
            }

            f = face_get_opposite_from_edge(f, e);
            if (closed && f == frep) break;
        }

        int anv = (int)av.size();
        Vector3 vec = vector3_set1(0.f);

        if (type == VERTEX_NORMAL_ANGLE)
        {
            for (int i = 0; i < (int)af.size(); ++i)
            {
                int i1 = i + 1;
                if (i1 == (int)av.size()) i1 = 0;
                float ang = vector3_angle
                (
                    vector3_normalize(vector3_sub(av[i]->point, vp)),
                    vector3_normalize(vector3_sub(av[i1]->point, vp))
                );

                Polygon* p = polygon_create_from_face(mesh, af[i]);
                vec = vector3_add(vec, vector3_mul_scalar(polygon_get_normal(p), ang));
                polygon_destroy(p);
            }
        }
        else if (type == VERTEX_NORMAL_SUM)
        {
            for (int i = 0; i < (int)af.size(); ++i)
            {
                Polygon* p = polygon_create_from_face(mesh, af[i]);
                vec = vector3_add(vec, polygon_get_normal(p));
                polygon_destroy(p);
            }
        }
        else if (type == VERTEX_NORMAL_AREA)
        {
            for (int i = 0; i < (int)af.size(); ++i)
            {
                Polygon* p = polygon_create_from_face(mesh, af[i]);
                vec = vector3_add(vec, vector3_mul_scalar(polygon_get_normal(p), polygon_get_area(p)));
                polygon_destroy(p);
            }
        }

        vec = vector3_normalize(vec);
        if (af.size() == nfaces)
        {
            vn->normal = vec;
        }

        for (Face* ff : af)
        {
            setfvis.insert(ff->id);
            vn->face_to_normal[ff->id] = vec;
        }
    }

    return vn;
}

void vertexnormal_destroy(VertexNormal* vn)
{
    PoolAllocator* p = vn->allocator;
    vn->~VertexNormal();
    p->Free(vn);
}