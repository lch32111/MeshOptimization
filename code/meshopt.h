#ifndef __MESHOPT_H__
#define __MESHOPT_H__

#include <vector>
#include "vector.h"


struct Mesh;

Mesh* mesh_optimize
(
    Mesh* m, 
    const std::vector<Vector3>& sampled_points,
    float rep_constant = 1e-5f,
    float spring_constant = 1.f
);

#endif