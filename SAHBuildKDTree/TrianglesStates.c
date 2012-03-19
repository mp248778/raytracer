/*
Copyright (c) 2012 Michal Pachucki

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "TrianglesStates.h"
#include <Math/Vector.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void initTrianglesStates(TrianglesStates *trianglesStates, const char* fileName)
{
  memset(trianglesStates, 0, sizeof(*trianglesStates));
  FILE *file = fopen(fileName, "r");
  uint64_t vectorsCount = 0;
  Vector* vectors = NULL;
  Triangle* triangles;
  if (fscanf(file, "%lu", &vectorsCount) != 1)
    goto fin;

  vectors = malloc(sizeof(Vector) * vectorsCount);
  for (uint64_t i = 0; i < vectorsCount; i++) {
    if (fscanf(file, "%f %f %f", &vectors[i][0], &vectors[i][1], &vectors[i][2]) != 3)
      goto fin;
  }

  uint64_t trianglesCount = 0;
  if (fscanf(file, "%lu", &trianglesCount) != 1)
    goto fin;

  triangles = malloc(sizeof(Triangle) * trianglesCount);
  for (uint64_t i = 0; i < trianglesCount; i++) {
    uint64_t index0 = 0, index1 = 0, index2 = 0;
    if (fscanf(file, "%lu, %lu %lu", &index0, &index1, &index2) != 3)
      goto fin;
    memcpy(triangles[i].vertice[0], vectors[index0], sizeof(Vector));
    memcpy(triangles[i].vertice[1], vectors[index1], sizeof(Vector));
    memcpy(triangles[i].vertice[2], vectors[index2], sizeof(Vector));
  }


  trianglesStates->count = trianglesCount;
  trianglesStates->side = malloc(sizeof(TriangleStateSide) * trianglesCount);
  trianglesStates->triangles = triangles;

fin:
  if (file)
    fclose(file);
  free(vectors);
}


void deinitTrianglesStates(TrianglesStates* trianglesStates)
{
  free(trianglesStates->triangles);
  free(trianglesStates->side);
}


