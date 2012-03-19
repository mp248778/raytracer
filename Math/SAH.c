/*
Copyright (c) 2012 Michal Pachucki

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef Math_SAH
#define Math_SAH

#include <Math/AABB.h>
#include <stdbool.h>
#include <stdint.h>

static float AABBSurface(const AABB *aabb)
{
  float a = aabb->higher[0] - aabb->lower[0];
  float b = aabb->higher[1] - aabb->lower[1];
  float c = aabb->higher[2] - aabb->lower[2];

  return 2.0f * (a * b + b * c + c * a);
}

void SAH(float *cost, bool *parallelLeft, const AABB *aabb, float plane, int axis, uint64_t countLeft, uint64_t countParallel, uint64_t countRight)
{
  AABB left, right;
  splitAABB(aabb, plane, axis, &left, &right);

  float surface = AABBSurface(aabb);
  float leftSurface = AABBSurface(&left);
  float rightSurface = AABBSurface(&right);

  float sah1 = ((countLeft + countParallel) * leftSurface) + (countRight * rightSurface);
  float sah2 = (countLeft * leftSurface) + ((countRight + countParallel) * rightSurface);
  float sah;

  if (sah1 < sah2) {
    *parallelLeft = true;
    *cost = sah1;
  } else {
    *parallelLeft = false;
    *cost = sah2;
  }
}

#endif
