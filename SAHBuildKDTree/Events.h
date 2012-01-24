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
