/*
Copyright (c) 2012 Michal Pachucki

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "AABB.h"
#include <float.h>
#include <string.h>

void initAABB(AABB *aabb, Triangle *triangles, uint64_t trianglesCount)
{
  Vector vecMin = {FLT_MIN, FLT_MIN, FLT_MIN};
  Vector vecMax = {FLT_MAX, FLT_MAX, FLT_MAX};
  memcpy(aabb->lower, vecMax, sizeof(vecMax));
  memcpy(aabb->higher, vecMin, sizeof(vecMin));
  for (uint64_t i = 0; i < trianglesCount; i++) {
    for (int j = 0; j < 3; j++) {
      for (int axis = 0; axis < 4; axis++) {
        float v = triangles[i].vertice[j][axis];
        if (aabb->lower[axis] > v) {
          aabb->lower[axis] = v;
        } else if (aabb->higher[axis] < v) {
          aabb->higher[axis] = v;
        }
      }
    }
  }
}

void splitAABB(AABB *aabb, float splitPlane, int splitAxis, AABB *leftAABB, AABB *rightAABB)
{
  memcpy(leftAABB, aabb, sizeof(*aabb));
  memcpy(rightAABB, aabb, sizeof(*aabb));
  leftAABB->higher[splitAxis] = splitPlane;
  rightAABB->lower[splitAxis] = splitPlane;
}
