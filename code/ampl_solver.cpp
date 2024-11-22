#include "ampl_solver.h"

#include <ampl/ampl.h>

#include "common.h"
#include "filebuffer.h"

const char* G_AMPL_ENV_PATH = NULL;

struct AMPLSolver
{
    PoolAllocator* allocator;
    ampl::Environment* env;
    ampl::AMPL* ampl;
};

AMPLSolver* ampl_solver_create(PoolAllocator* allocator)
{
    const size_t alloc_size = sizeof(AMPLSolver) + sizeof(ampl::Environment) + sizeof(ampl::AMPL);

    uint8_t* buffer = (uint8_t*)allocator->Alloc(alloc_size);

    AMPLSolver* solver = (AMPLSolver*)buffer;
    ampl::Environment* env = (ampl::Environment*)(buffer + sizeof(AMPLSolver));
    ampl::AMPL* ampl = (ampl::AMPL*)(buffer + sizeof(AMPLSolver) + sizeof(ampl::Environment));

    new (env) ampl::Environment(G_AMPL_ENV_PATH);
    new (ampl) ampl::AMPL(*env);

    solver->allocator = allocator;
    solver->env = env;
    solver->ampl = ampl;

    return solver;
}

void ampl_solver_destroy(AMPLSolver* solver)
{
    PoolAllocator* allocator = solver->allocator;

    solver->ampl->~AMPL();
    solver->env->~Environment();

    allocator->Free(solver);
}

void* ampl_solver_get_context(AMPLSolver* solver)
{
    return (void*)solver->ampl;
}

void ampl_solver_read_model(AMPLSolver* solver, const char* model)
{
    const char* path = "AMPL_TEMP_MODEL_20230407xpzlasmenjhfj.mod";

    FileBuffer fb;
    filebuffer_open(&fb, path, "wb");
    filebuffer_write_str(&fb, model);
    filebuffer_close(&fb);

    solver->ampl->read(path);

    file_remove(path);
}