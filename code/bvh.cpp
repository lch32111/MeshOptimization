#include "bvh.h"

#include "common.h"
#include "allocator.h"

// from godot/sort_array.h
template<class T, class Comparator>
class SortArray
{
public:
    Comparator compare;

    inline int bitlog(int n)
    {
        int k;
        for (k = 0; n != 1; n >>= 1)
            ++k;

        return k;
    }

    const T& median_of_3(const T& a, const T& b, const T& c)
    {
        if (compare(a, b))
        {
            if (compare(b, c))
                return b;
            else if (compare(a, c))
                return c;
            else
                return a;
        }
        else if (compare(a, c))
        {
            return a;
        }
        else if (compare(b, c))
        {
            return c;
        }
        else
        {
            return b;
        }
    }

    inline int partitioner(int first, int last, T pivot, T* arr)
    {
        const int unmodified_first = first;
        const int unmodified_last = last;

        while (true)
        {
            while (compare(arr[first], pivot))
            {
                ++first;
            }
            --last;

            while (compare(pivot, arr[last]))
            {
                --last;
            }

            if (!(first < last))
            {
                return first;
            }

            T temp = arr[first];
            arr[first] = arr[last];
            arr[last] = temp;

            ++first;
        }
    }

    inline void unguarded_linear_insert(int last, T value, T* p)
    {
        int next = last - 1;
        while (compare(value, p[next]))
        {
            p[last] = p[next];
            last = next;
            --next;
        }
        p[last] = value;
    }

    inline void linear_insert(int first, int last, T* p)
    {
        T val = p[last];
        if (compare(val, p[first]))
        {
            for (int i = last; i > first; --i)
            {
                p[i] = p[i - 1];
            }

            p[first] = val;
        }
        else
        {
            unguarded_linear_insert(last, val, p);
        }
    }

    inline void insertion_sort(int first, int last, T* p)
    {
        if (first == last)
            return;

        for (int i = first + 1; i != last; ++i)
        {
            linear_insert(first, i, p);
        }
    }

    inline void introselect(int first, int nth, int last, T* p, int max_depth)
    {
        while (last - first > 3)
        {
            if (max_depth == 0)
            {
                return;
            }

            int cut = partitioner(first, last,
                median_of_3
                (
                    p[first],
                    p[first + (last - first) / 2],
                    p[last - 1]
                ),
                p
            );

            if (cut <= nth)
            {
                first = cut;
            }
            else
            {
                last = cut;
            }
        }

        insertion_sort(first, last, p);
    }


    inline void nth_element(int first, int last, int nth, T* p)
    {
        if (first == last || last == nth)
        {
            return;
        }
        introselect(first, nth, last, p, bitlog(last - first));
    }
};

template<int axis>
struct BVCompare
{
    bool operator()(const BV* left, const BV* right)
    {
        return left->center.v[axis] < right->center.v[axis];
    }
};

// from godot/triangle_mesh.cpp
static inline int create_bvh(BV* p_bvh, BV** p_bb, int p_from, int p_size, int p_depth, int& r_max_depth, int& r_max_alloc)
{
    if (p_depth > r_max_depth)
        r_max_depth = p_depth;

    if (p_size == 1)
    {
        return (int)(p_bb[p_from] - p_bvh);
    }
    else if (p_size == 0)
    {
        return -1;
    }

    AABB aabb;
    aabb = p_bb[p_from]->aabb;
    for (int i = 1; i < p_size; ++i)
    {
        aabb_combine_aabb(&aabb, &(p_bb[p_from + i]->aabb));
    }

    int li = aabb_get_logest_axis_index(&aabb);

    switch (li)
    {
    case 0:
        SortArray<BV*, BVCompare<0>> sort_x;
        sort_x.nth_element(0, p_size, p_size / 2, &p_bb[p_from]);
        break;
    case 1:
        SortArray<BV*, BVCompare<1>> sort_y;
        sort_y.nth_element(0, p_size, p_size / 2, &p_bb[p_from]);
        break;
    case 2:
        SortArray<BV*, BVCompare<2>> sort_z;
        sort_z.nth_element(0, p_size, p_size / 2, &p_bb[p_from]);
        break;
    }

    int left = create_bvh(p_bvh, p_bb, p_from, p_size / 2, p_depth + 1, r_max_depth, r_max_alloc);
    int right = create_bvh(p_bvh, p_bb, p_from + p_size / 2, p_size - p_size / 2, p_depth + 1, r_max_depth, r_max_alloc);

    int index = r_max_alloc++;
    BV* new_bv = &(p_bvh[index]);
    new_bv->aabb = aabb;
    aabb_get_center(&aabb, new_bv->center.v);
    new_bv->id = NULL;
    new_bv->left = left;
    new_bv->right = right;

    return index;
}

BVH* bvh_create_from_triangles
(
    Vector3* __restrict positions,
    int position_count,
    TriangleIndex* __restrict indices,
    void** __restrict ids,
    int index_id_count,
    PoolAllocator* __restrict allocator
)
{
    BVH* bvh = NULL;
    BV** bv_ps = NULL;
    int max_alloc = index_id_count;

    if (allocator)
    {
        bvh = (BVH*)allocator->Alloc(sizeof(BVH));
        bvh->bvs = (BV*)allocator->Alloc(sizeof(BV) * index_id_count * 3);

        bv_ps = (BV**)allocator->Alloc(sizeof(BV*) * max_alloc);
    }
    else
    {
        bvh = (BVH*)malloc(sizeof(BVH));
        bvh->bvs = (BV*)malloc(sizeof(BV) * index_id_count * 3);

        bv_ps = (BV**)malloc(sizeof(BV) * max_alloc);
    }

    bvh->bv_count = index_id_count * 3;
    bvh->max_depth = 0;
    bvh->allocator = allocator;
    
    for (int i = 0; i < index_id_count; ++i)
    {
        TriangleIndex ti = indices[i];

        Vector3 p0 = positions[ti.v[0]];
        Vector3 p1 = positions[ti.v[1]];
        Vector3 p2 = positions[ti.v[2]];


        BV* bv = &(bvh->bvs[i]);
        AABB* aabb = &(bv->aabb);

        aabb_set_min_max(aabb, p0.v[0], p0.v[1], p0.v[2]);
        aabb_combine_float(aabb, p1.v);
        aabb_combine_float(aabb, p2.v);
        aabb_get_center(aabb, bv->center.v);
        bv->left = -1;
        bv->right = -1;
        bv->id = ids[i];
    }

    for (int i = 0; i < max_alloc; ++i)
    {
        bv_ps[i] = &(bvh->bvs[i]);
    }

    create_bvh(bvh->bvs, bv_ps, 0, index_id_count, 1, bvh->max_depth, max_alloc);

    if (allocator)
    {
        allocator->Free(bv_ps);
    }
    else
    {
        free(bv_ps);
    }

    bvh->bv_count = max_alloc;

    return bvh;
}

void bvh_destroy(BVH* bvh)
{
    if (bvh->allocator)
    {
        PoolAllocator* allocator = bvh->allocator;
        allocator->Free(bvh->bvs);
        allocator->Free(bvh);
    }
    else
    {
        free(bvh->bvs);
        free(bvh);
    }
}

static inline bool is_bvh_leaf(BV* bv)
{
    return (bv->left == -1 && bv->right == -1);
}

void bvh_query_aabb(BVH* bvh, AABB aabb, BVHLeafCallback callback, void* user_callback_context)
{
    int* stack = (int*)ALLOCA(sizeof(int) * bvh->max_depth);
    int stack_index = 0;
    BV* bvptr = bvh->bvs;
    BV* bv = NULL;

    stack[stack_index] = bvh->bv_count - 1;
    ++stack_index;
    while (true)
    {
        if (stack_index <= 0)
            break;

        int bvh_index = stack[stack_index - 1];
        --stack_index;

        bv = &(bvptr[bvh_index]);

        if (is_bvh_leaf(bv) == true)
        {
            callback(NULL, user_callback_context, bv->id);
        }
        else
        {
            if (aabb_intersect_aabb(&aabb, &(bv->aabb)) == false)
                continue;

            if (bv->left >= 0)
            {
                stack[stack_index] = bv->left;
                ++stack_index;
            }

            if (bv->right >= 0)
            {
                stack[stack_index] = bv->right;
                ++stack_index;
            }
        }
    }
}

void bvh_query_point(BVH* bvh, Vector3 query_point, BVHLeafCallback callback, void* user_callback_context)
{
    int* stack = (int*)ALLOCA(sizeof(int) * bvh->max_depth);
    int stack_index = 0;
    BV* bvptr = bvh->bvs;
    BV* bv = NULL;
    BV* temp_bv;

    stack[stack_index] = bvh->bv_count - 1;
    ++stack_index;

    float closest_dist_sq = FLT_MAX;

    bool is_add_left;
    bool is_add_right;
    float sq_left_ex_dist;
    float sq_right_ex_dist;

    while (true)
    {
        if (stack_index <= 0)
            break;

        int bvh_index = stack[stack_index - 1];
        --stack_index;

        bv = &(bvptr[bvh_index]);
        if (is_bvh_leaf(bv) == true)
        {
            callback(&closest_dist_sq, user_callback_context, bv->id);
        }
        else
        {
            is_add_left = false;
            is_add_right = false;
            sq_left_ex_dist = FLT_MAX - 1.f;
            sq_right_ex_dist = FLT_MAX - 1.f;

            if (bv->left >= 0)
            {
                temp_bv = &(bvptr[bv->left]);

                sq_left_ex_dist = aabb_distance_exterior_sq_point(&(temp_bv->aabb), query_point);

                if (aabb_contain_point(&(temp_bv->aabb), query_point))
                {
                    stack[stack_index] = bv->left;
                    ++stack_index;

                    is_add_left = true;
                }
            }

            if (bv->right >= 0)
            {
                temp_bv = &(bvptr[bv->right]);

                sq_right_ex_dist = aabb_distance_exterior_sq_point(&(temp_bv->aabb), query_point);

                if (aabb_contain_point(&(temp_bv->aabb), query_point))
                {
                    stack[stack_index] = bv->right;
                    ++stack_index;

                    is_add_right = true;
                }
            }

            if (bv->left >= 0 && is_add_left == false && sq_left_ex_dist < closest_dist_sq)
            {
                stack[stack_index] = bv->left;
                ++stack_index;
            }

            if (bv->right >= 0 && is_add_right == false && sq_right_ex_dist < closest_dist_sq)
            {
                stack[stack_index] = bv->right;
                ++stack_index;
            }
        }
    }
}