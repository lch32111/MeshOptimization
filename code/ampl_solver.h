#ifndef __AMPL_SOLVER_H__
#define __AMPL_SOLVER_H__

#include "allocator.h"

// set in the main
extern const char* G_AMPL_ENV_PATH;

struct AMPLSolver;

AMPLSolver* ampl_solver_create(PoolAllocator* allocator);
void ampl_solver_destroy(AMPLSolver* solver);
void* ampl_solver_get_context(AMPLSolver* solver);

/*
* AMPL only reads a model from a file.
* So, this function write the model file, and then remove it after reading it.
*/
void ampl_solver_read_model(AMPLSolver* solver, const char* model);

#endif
