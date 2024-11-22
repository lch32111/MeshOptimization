#include "random.h"

#include "allocator.h"
#include <random>
#include <chrono>

struct RandomGen
{
    RandomGen(unsigned seed)
        : allocator(NULL)
        , gen(seed)
        , d(0.0, 1.0)

    {}

    ~RandomGen()
    {}

    PoolAllocator* allocator;
    std::default_random_engine gen;
    std::uniform_real_distribution<double> d;
};

RandomGen* randomgen_create(PoolAllocator* p, unsigned* seed_pointer)
{
    RandomGen* rg = (RandomGen*)p->Alloc(sizeof(RandomGen));

    unsigned seed;
    if (seed_pointer == NULL)
        seed = std::chrono::system_clock::now().time_since_epoch().count();
    else
        seed = *seed_pointer;

    new (rg) RandomGen(seed);
    rg->allocator = p;

    return rg;
}

void randomgen_destroy(RandomGen* rg)
{
    PoolAllocator* p = rg->allocator;
    rg->~RandomGen();
    p->Free(rg);
}

float randomgen_get_uniform(RandomGen* rg)
{
    return (float)rg->d(rg->gen);
}
