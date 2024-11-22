#include <stdio.h>
#include <stdlib.h>

#include "common.h"
#include "mesh.h"
#include "meshopt.h"
#include "obj.h"
#include "filebuffer.h"

#ifdef USE_AMPL_SOLVER
#include "ampl_solver.h"
#include "ampl_test.h"
#endif

#include "mesh_test.h"

/*
* argv[1] is the path of an obj file.
* argv[2] is the shape index of the obj file
*/
int main(int argc, char* argv[])
{
    if (argc < 3)
    {
        printf("ex) *.exe abc.obj 3 (3 is the index of shapes in the obj)\n");
        return -1;
    }
    
    const char* path = argv[1];
    int shape_index = atoi(argv[2]);

    /*
    ampl_test(argv);
    return -1;
    */

    /*
    mesh_test();
    return -1;
    */
    Mesh* mesh = NULL;
    {
        std::vector<ObjShape> shapes;
        obj_get_shape(path, shapes);

        if (shapes.size() == 0)
        {
            printf("Fail to read the obj file %s\n", path);
            return -1;
        }

        if (shape_index < 0 || shape_index >= shapes.size())
        {
            printf("Wrong Shape Index %d for %d shapes\n", shape_index, (int)shapes.size());
            return -1;
        }

        mesh = mesh_create(&(shapes[shape_index]));
    }
    
    printf("Start Mesh Optimization!\n");

    std::vector<Vector3> random_points;
    std::vector<Vector3> random_point_normals;
    unsigned seed = 20241121;
    mesh_sample_random_points(mesh, 10000, &seed, random_points, random_point_normals);

    Mesh* optimized = mesh_optimize(mesh, random_points);

    printf("End Mesh Optimization!\n");


    mesh_destroy(optimized);
    mesh_destroy(mesh);

    return 0;
}