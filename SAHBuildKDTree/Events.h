/*
Copyright (c) 2012 Michal Pachucki

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef SAHBuildKDTree_Events
#define SAHBuildKDTree_Events

#include <Math/AABB.h>
#include "TrianglesStates.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct Event Event;

typedef struct {
  uint64_t count[3];
  Event* firstEvent[3];
  Event* lastEvent[3];
  void *pool;
} Events;

void findSplitPlane(Events events, float *splitCost, float *splitPlane, int *splitAxis, bool *splitParallelLeft, const AABB aabb);

void generateSortedEventsForSplitTriangles(Events* events, TrianglesStates trianglesStates);

void initSortedEvents(Events *events, TrianglesStates trianglesStates);

void splitEvents(Events *leftEvents, Events *rightEvents, Events events, TrianglesStates trianglesStates);

void mergeEvents(Events *events, Events newEvents);

void deinitEvents(Events events);

#endif
