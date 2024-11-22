#ifndef __RANDOM_H__
#define __RANDOM_H__

class PoolAllocator;
struct RandomGen;

// pass NULL to seed_pointer for fully randomness.
RandomGen* randomgen_create(PoolAllocator* p, unsigned* seed_pointer);
void randomgen_destroy(RandomGen* rg);

float randomgen_get_uniform(RandomGen* rg); // [0.0, 1.0)

#endif