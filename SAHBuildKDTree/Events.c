/*
Copyright (c) 2012 Michal Pachucki

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Events.h"
#include <Math/SAH.h>
#include <stdlib.h>
#include <string.h>

void findSplitPlane(Events events, float *splitCost, float *splitPlane, int *splitAxis, bool *splitParallelLeft, const AABB aabb)
{
  int bestAxis = -1;
  float bestCost = -1;
  bool bestParallelLeft;
  float bestPlane;

  for(int axis = 0; axis < 3; axis++) {
    uint64_t countLeft = 0;
    uint64_t countParallel = 0;
    uint64_t countRight = events.count[axis];
    for(Event* currentEvent = events.firstEvent[axis]; currentEvent; currentEvent = currentEvent->next) {
      uint64_t planeStarts = 0;
      uint64_t planeEnds = 0;
      uint64_t planeParallels = 0;
      float plane = currentEvent->plane;
      while (currentEvent && currentEvent->type == EventType_end && currentEvent->plane == plane) {
        planeEnds++;
        currentEvent = currentEvent->next;
      }
      while (currentEvent && currentEvent->type == EventType_parallel && currentEvent->plane == plane) {
        planeParallels++;
        currentEvent = currentEvent->next;
      }
      while (currentEvent && currentEvent->type == EventType_start && currentEvent->plane == plane) {
        planeStarts++;
        currentEvent = currentEvent->next;
      }

      countParallel = planeParallels;
      countRight -= planeParallels;
      countRight -= planeEnds;

      float cost;
      bool parallelLeft;
      SAH(&cost, &parallelLeft, aabb, plane, axis, countLeft, countParallel, countRight);
      if (cost < bestCost || bestAxis == -1) {
        bestAxis = axis;
        bestCost = cost;
        bestParallelLeft = parallelLeft;
        bestPlane = plane;
      }
      countLeft += planeStarts;
      countLeft += planeParallels;
    }
  }

  *splitCost = bestCost;
  *splitPlane = bestPlane;
  *splitAxis = bestAxis;
  *splitParallelLeft = bestParallelLeft;
}

static inline void swapFloats(float *a, float *b)
{
  float t = *a;
  *a = *b;
  *b = t;
}

static void sort3Floats(float *a, float *b, float *c)
{
  if (*a < *b)
    swapFloats(a, b);
  if (*a < *c)
    swapFloats(a, c);
  if (*b < *c)
    swapFloats(b, c);
}

static int compareEvents(const void *AEvent_, const void *BEvent_)
{
  const Event* AEvent = AEvent_;
  const Event* BEvent = BEvent_;

  if (AEvent->plane != BEvent->plane) {
    if (AEvent->plane < BEvent->plane)
      return 1;
    else
      return -1;
  }

  if (AEvent->type != BEvent->type)
    return BEvent->type - AEvent->type;

  if (AEvent->triangle > BEvent->triangle)
    return 1;
  else
    return -1;
}

static void prepareEventsFromPool(Event *eventPool, uint64_t eventCnt)
{
  eventPool = realloc(eventPool, eventCnt * sizeof(Event));
  qsort(eventPool, eventCnt, sizeof(Event), compareEvents);
  for (uint64_t i = 0; i < eventCnt - 1; i++)
    eventPool[i].next = &eventPool[i + 1];
  eventPool[eventCnt - 1].next = NULL;
}

void generateSortedEventsForSplitTriangles(Events events, TrianglesStates trianglesStates, float splitPlane, int splitAxis, bool splitParallelLeft, Events* newLeftEvents, Events* newRightEvents)
{
  for (int axis = 0; axis < 3; axis++) {
    Event* leftEventPool = malloc(sizeof(Event) * events.count[axis]);
    Event* rightEventPool = malloc(sizeof(Event) * events.count[axis]);
    uint64_t leftEventIdx = 0;
    uint64_t rightEventIdx = 0;
    for(Event* currentEvent = events.firstEvent[axis]; currentEvent; currentEvent = currentEvent->next) {
      if (currentEvent->type == EventType_end)
        continue;
      if (trianglesStates.side[currentEvent->triangle] != TriangleState_Side_Both)
        continue;

      Triangle *t = &trianglesStates.triangles[currentEvent->triangle];
      float a = t->vertice[0][axis];
      float b = t->vertice[1][axis];
      float c = t->vertice[2][axis];
      sort3Floats(&a, &b, &c);
      if (a == c) {
        Event* event = NULL;
        if (splitParallelLeft) {
          event = &leftEventPool[leftEventIdx];
          leftEventIdx++;
        } else {
          event = &rightEventPool[rightEventIdx];
          rightEventIdx++;
        }
        *event = *currentEvent;
      } else {
        float as = t->vertice[0][splitAxis];
        float bs = t->vertice[1][splitAxis];
        float cs = t->vertice[2][splitAxis];

        sort3Floats(&as, &bs, &cs);
        float ratio = (splitPlane - as) / (cs - as);    //cs - as cannot be zero as the triangle is on both sides of the split plane
        float d = as + ratio * (cs - as);

        Event *event = &leftEventPool[leftEventIdx];
        event->axis = axis;
        event->plane = a;
        event->triangle = currentEvent->triangle;
        event->type = EventType_start;

        event = &leftEventPool[leftEventIdx + 1];
        event->axis = axis;
        event->plane = d;
        event->triangle = currentEvent->triangle;
        event->type = EventType_end;
        leftEventIdx += 2;

        event = &rightEventPool[rightEventIdx];
        event->axis = axis;
        event->plane = d;
        event->triangle = currentEvent->triangle;
        event->type = EventType_start;

        event = &rightEventPool[rightEventIdx + 1];
        event->axis = axis;
        event->plane = c;
        event->triangle = currentEvent->triangle;
        event->type = EventType_end;
        rightEventIdx += 2;
      }
    }
    prepareEventsFromPool(leftEventPool, leftEventIdx);
    newLeftEvents->pool[axis] = leftEventPool;
    newLeftEvents->firstEvent[axis] = leftEventPool;
    newLeftEvents->count[axis] = leftEventIdx;
    newLeftEvents->lastEvent[axis] = &leftEventPool[leftEventIdx - 1];

    prepareEventsFromPool(rightEventPool, rightEventIdx);
    newRightEvents->pool[axis] = rightEventPool;
    newRightEvents->firstEvent[axis] = rightEventPool;
    newRightEvents->count[axis] = rightEventIdx;
    newRightEvents->lastEvent[axis] = &rightEventPool[rightEventIdx - 1];
  }
}


void initSortedEvents(Events *events, TrianglesStates trianglesStates)
{
  for (int axis = 0; axis < 3; axis++) {
    Event* eventPool = malloc(sizeof(Event) * trianglesStates.count * 2);
    uint64_t eventIdx = 0;
    for (uint64_t i = 0; i < trianglesStates.count; i++) {

      float a = trianglesStates.triangles[i].vertice[0][axis];
      float b = trianglesStates.triangles[i].vertice[1][axis];
      float c = trianglesStates.triangles[i].vertice[2][axis];

      if (a < b)
        swapFloats(&a, &b);
      if (a < c)
        swapFloats(&a, &c);
      if (b < c)
        swapFloats(&b, &c);
      //a <= b <= c
      if (a == c) {
        Event* event = &eventPool[eventIdx];
        event->plane = a;
        event->type = EventType_parallel;
        event->triangle = i;

        eventIdx++;
      } else {
        Event* event = &eventPool[eventIdx];
        event->plane = a;
        event->type = EventType_start;
        event->triangle = i;

        event = &eventPool[eventIdx + 1];
        event->plane = c;
        event->type = EventType_end;
        event->triangle = i;

        eventIdx += 2;
      }
    }

    eventPool = realloc(eventPool, eventIdx * sizeof(Event));
    qsort(eventPool, eventIdx, sizeof(Event), compareEvents);

    events->count[axis] = eventIdx;
    events->pool[axis] = eventPool;
    events->firstEvent[axis] = eventPool;
    events->lastEvent[axis] = &eventPool[events->count[axis] - 1];
    for (uint64_t i = 0; i < events->count[axis] - 1; i++)
      eventPool[i].next = &eventPool[i + 1];
    events->lastEvent[axis]->next = NULL;
  }
}

void splitEvents(Events *leftEvents, Events *rightEvents, Events events, TrianglesStates trianglesStates)
{
  memset(leftEvents, 0, sizeof(Events));
  memset(rightEvents, 0, sizeof(Events));

  for (int axis = 0; axis < 3; axis++) {
    Event *currentEvent = events.firstEvent[axis];
    while (currentEvent) {
      Event *nextEvent = currentEvent->next;
      currentEvent->next = NULL;

      Events *sidedEvents;
      switch (trianglesStates.side[currentEvent->triangle]) {
        case TriangleState_Side_Left:
          sidedEvents = leftEvents;
          break;
        case TriangleState_Side_Right:
          sidedEvents = rightEvents;
          break;
        default:
          currentEvent = nextEvent;
          continue;
      }
      if (!sidedEvents->firstEvent[axis])
        sidedEvents->firstEvent[axis] = sidedEvents->lastEvent[axis] = currentEvent;
      else {
        sidedEvents->lastEvent[axis] = sidedEvents->lastEvent[axis]->next = currentEvent;
      }
    }
  }
}

void mergeEvents(Events *events, Events newEvents)
{
  for (int axis = 0; axis < 3; axis++) {
    if (events->firstEvent[axis])
      events->lastEvent[axis] = events->lastEvent[axis]->next = newEvents.firstEvent[axis];
    else
      events->lastEvent[axis] = newEvents.firstEvent[axis];
  }
}

void deinitEvents(Events events)
{
  for (int axis = 0; axis < 3; axis++)
    free(events.pool);
}

void categorizeTriangles(TrianglesStates trianglesStates, Events events, float splitPlane, int splitAxis, bool splitParallelLeft)
{
  for (Event* currentEvent = events.firstEvent[splitAxis]; currentEvent; currentEvent = currentEvent->next) {
    if (currentEvent->type == EventType_end)
      continue;
    trianglesStates.side[currentEvent->triangle] = TriangleState_Side_Both;
  }
  for (Event* currentEvent = events.firstEvent[splitAxis]; currentEvent; currentEvent = currentEvent->next) {
    switch(currentEvent->type) {
      case EventType_start:
        if (currentEvent->plane > splitPlane) {
          trianglesStates.side[currentEvent->triangle] = TriangleState_Side_Right;
        }
        break;
      case EventType_end:
        if (currentEvent->plane <= splitPlane) {
          trianglesStates.side[currentEvent->triangle] = TriangleState_Side_Left;
        }
        break;
      case EventType_parallel:
        if (currentEvent->plane == splitPlane) {
          trianglesStates.side[currentEvent->triangle] = splitParallelLeft ? TriangleState_Side_Left : TriangleState_Side_Right;
        }
        break;

    }
  }
}
