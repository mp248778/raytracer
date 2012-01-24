#ifndef Math_SAH
#define Math_SAH

#include <Math/AABB.h>
#include <stdbool.h>
#include <stdint.h>

void SAH(float *cost, bool *parallelLeft, const AABB aabb, float plane, int axis, uint64_t countLeft, uint64_t countParallel, uint64_t countRight);


#endif
