#ifndef __MESH_H__
#define __MESH_H__

/*
* https://github.com/hhoppe/Mesh-processing-library
* copy the code from the above repository to study and understand the Mesh Optimization paper.
*/

#include "common.h"
#include "vector.h"


struct ObjShape;

struct HalfEdge;

struct Vertex
{
    Vertex() = delete;
    Vertex(PoolAllocator* allocator)
        : arhe(allocator)
    {}


    TVector<HalfEdge*> arhe;
    uint32_t id;
    Vector3 point;
};

struct Face
{
    HalfEdge* herep;
    uint32_t id;
};

struct Edge
{
    HalfEdge* herep;
    bool is_sharp;
};

struct HalfEdge
{
    HalfEdge* prev;  // previous half edge in ring around face
    HalfEdge* next;  // next half edge in ring around face
    HalfEdge* sym;   // pointer to symmetric half edge (or 0)
    Vertex* vert;    // vertex to which this half edge is pointing
    Face* face;
    Edge* edge;
};

struct Mesh
{
    Mesh(PoolAllocator* allocator)
        : num_vertex(0)
        , num_face(0)
        , num_edges(0)
        , vertex_map(allocator)
        , face_map(allocator)
        , allocator(allocator)
    {}


    uint32_t num_vertex;     // id to assign to next new vertex
    uint32_t num_face;       // id to assign to next new face
    uint32_t num_edges;
    TUnorderedMap<uint32_t, Vertex*> vertex_map;
    TUnorderedMap<uint32_t, Face*> face_map;
    PoolAllocator* allocator;
};

Mesh* mesh_create(ObjShape* shape);
Mesh* mesh_create_vertices_faces(const Vector3* vertices, int vertex_count, const int* faces, int face_index_count);
Mesh* mesh_clone(Mesh* mesh);
void mesh_destroy(Mesh* mesh);

Vertex* mesh_create_vertex(Mesh* mesh);
void mesh_destroy_vertex(Mesh* mesh, Vertex* v);

Face* mesh_create_face(Mesh* mesh, Vertex** va, int va_count);
void mesh_destroy_face(Mesh* mesh, Face* f);

/*
* Define the sym and edge variable of half edge. The v1 is the starting vertex of the halfedge.
* The he should have its vertex pointer.
*/
void mesh_halfedge_enter(Mesh* mesh, HalfEdge* he, Vertex* v1);

/*
* 
*/
void mesh_halfedge_remove(Mesh* mesh, HalfEdge* he, Vertex* v1);

float mesh_face_get_area(Mesh* m, Face* f);

void mesh_create_bogus_halfedges(Mesh* m, TVector<HalfEdge*>& ar_he);
void mesh_destroy_bogus_halfedge(Mesh* m, TVector<HalfEdge*>& ar_he);
void mesh_collapse_edge(Mesh* m, Edge* e);
Vertex* mesh_split_edge(Mesh* m, Edge* e);
Edge* mesh_swap_edge(Mesh* m, Edge* e);

// pass NULL to seed_pointer for fully randomness.
void mesh_sample_random_points
(
    Mesh* mesh, 
    int sample_count, 
    unsigned* seed_pointer,
    std::vector<Vector3>& output_points,
    std::vector<Vector3>& output_normals
);

void mesh_export_obj(Mesh* mesh, const char* path);
void mesh_export_obj_with_colors(Mesh* mesh, const char* path, Vector3* colors, int color_count); // the number of colors should be same as the number of vertices

bool mesh_is_sharp_ve(Mesh* mesh, Vertex* v, Edge* e);
bool mesh_is_legal_edge_collapse(Mesh* mesh, Edge* e);
bool mesh_is_nice_edge_collapse(Mesh* mesh, Edge* e);
bool mesh_is_legal_edge_swap(Mesh* mesh, Edge* e);
float mesh_get_min_dihedral_about_vertex(Mesh* m, Vertex* v);


/*
* The parameter v is v1 or v2 from the edge.
* return v2 if v is v1.
* return v1 if v is v2.
*/
Vertex* vertex_get_opposite_from_edge(Vertex* v, Edge* e);
/*
* return the other vertex of a triangle(face) which is located in the opposite place of the edge
*/
Vertex* vertex_get_opposite_from_ef(Edge* e, Face* f);
Vertex* vertex_get_clw(Vertex* v, Vertex* next);
Vertex* vertex_get_ccw(Vertex* v, Vertex* next);
Vertex* vertex_get_clw_from_fv(Face* f, Vertex* v);
Vertex* vertex_get_ccw_from_fv(Face* f, Vertex* v);
Vertex* vertex_get_most_clw(Vertex* v);
int vertex_get_num_boundaries(Vertex* v);
int vertex_get_num_sharp_edges(Vertex* v);
bool vertex_is_boundary(Vertex* v);
void vertex_get_ring(Vertex* v, TVector<Vertex*>& out_vertex_points);

/*
* iterate all vertices connected to vertex v
*/
struct VertexIteration
{
    TVector<HalfEdge*>::iterator cur;
    TVector<HalfEdge*>::iterator end;
    Vertex* extra_vertex;
};
VertexIteration vertex_get_iteration(Vertex* v);
Vertex* vertex_iterate(VertexIteration* vi);

/*
* return the halfedge of v1 that points to v2, otherwise NULL.
*/
HalfEdge* halfedge_query(Vertex* v1, Vertex* v2);
HalfEdge* halfedge_get_corner(Vertex* v, Face* f);
HalfEdge* halfedge_get_clw(HalfEdge* he);
HalfEdge* halfedge_get_ccw(HalfEdge* he);
/*
* return the most clockwise halfedge of the vertex v.
*/
HalfEdge* halfedge_get_most_clw(Vertex* v);
HalfEdge* halfedge_get_from_vf(Vertex* v, Face* f);
HalfEdge* halfedge_get_from_ef(Edge* e, Face* f);
HalfEdge* halfedge_get_from_ev1(Edge* e, Vertex* v);
HalfEdge* halfedge_get_from_ev2(Edge* e, Vertex* v);
bool halfedge_is_boundray(HalfEdge* he);

struct CornerFaceIteration
{
    HalfEdge* cur;
    HalfEdge* end;
    bool first;
};
CornerFaceIteration corner_face_get_iteration(Face* f);
HalfEdge* corner_face_iterate(CornerFaceIteration* cfi);


bool face_is_triangle(Face* f);
void face_get_vertices(Face* f, TVector<Vertex*>& va);
void face_get_triangle_vertices(Face* f, Vertex** arr);
Face* face_get_opposite_from_edge(Face* f, Edge* e);

bool edge_is_boundary(Edge* e);
bool edge_is_sharp(Edge* e);
/*
* return the edge which points from v to w or points from w to v
*/
Edge* edge_query(Vertex* v, Vertex* w);
Edge* edge_get_clw(Face* f, Vertex* v);
Edge* edge_get_ccw(Face* f, Vertex* v);
Vertex* edge_get_vertex1(Edge* e);
Vertex* edge_get_vertex2(Edge* e);
/*
* return the vertex in the face1 of edge, where the vertex is in the opposite place of the edge.
*/
Vertex* edge_get_side_vertex1(Edge* e);
Vertex* edge_get_side_vertex2(Edge* e);
Face* edge_get_face1(Edge* e);
Face* edge_get_face2(Edge* e);
Edge* edge_get_opposite_from_vf(Vertex* v, Face* f);

/*
* return the vertices around the edge. The first and last vertex can be same
*/
void edge_get_ring(Edge* e, TVector<Vertex*>& out_vertex_points);

void edge_get_faces(Edge* e, TVector<Face*>& out_faces);

/*
* out_faces[0] = e->herep->face;
* out_faces[1] = e->herep->sym ? e->herep->sym->face : NULL;
*/
void edge_get_both_faces(Edge* e, Face** out_faces);

/*
* out_vertices[0] = e->herep->vert;
* out_vertices[1] = e->herep->prev->vert;
*/
void edge_get_both_vertices(Edge* e, Vertex** out_vertices);

/*
* EdgeIteration ei = edge_get_iteration(mesh);
* while (Edge* e = edge_iterate(&ei))
* {
*    do something with edge
* }
*/
struct EdgeIteration
{
    TUnorderedMap<uint32_t, Vertex*>::iterator vcur;
    TUnorderedMap<uint32_t, Vertex*>::iterator vend;
    HalfEdge** hcur;
    HalfEdge** hend;
};
EdgeIteration edge_get_iteration(Mesh* m);
Edge* edge_iterate(EdgeIteration* input);

struct EdgeVertexIteration
{
    TVector<HalfEdge*>::iterator cur;
    TVector<HalfEdge*>::iterator end;
    Edge* extra_edge;
};
EdgeVertexIteration edge_vertex_get_iteration(Vertex* v);
Edge* edge_vertex_iterate(EdgeVertexIteration* evi);

struct EdgeFaceIteration
{
    HalfEdge* cur;
    HalfEdge* end;
    bool first;
};
EdgeFaceIteration edge_face_get_iteration(Face* f);
Edge* edge_face_iterate(EdgeFaceIteration* efi);

struct Polygon
{
    PoolAllocator* allocator;
    Vector3* vertices;
    int vertex_count;
};

Polygon* polygon_create_from_face(Mesh* mesh, Face* f);
void polygon_destroy(Polygon* polygon);

float polygon_get_area(Polygon* p);
Vector3 polygon_get_normal_sum(Polygon* p);
Vector3 polygon_get_normal(Polygon* p);

enum VertexNormalType
{
    VERTEX_NORMAL_ANGLE,
    VERTEX_NORMAL_SUM,
    VERTEX_NORMAL_AREA
};

struct VertexNormal
{
    VertexNormal(PoolAllocator* allocator)
        : allocator(allocator)
        , face_to_normal(allocator)
    {}

    VertexNormalType type;
    PoolAllocator* allocator;
    TUnorderedMap<uint32_t, Vector3> face_to_normal;
    Vector3 normal;
};

VertexNormal* vertexnormal_create(Mesh* m, Vertex* v, VertexNormalType type);
void vertexnormal_destroy(VertexNormal* vn);

#endif