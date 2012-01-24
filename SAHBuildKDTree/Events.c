#include "Events.h"
#include <Math/SAH.h>
#include <stdlib.h>
#include <string.h>

typedef enum {
  EventType_end,
  EventType_parallel,
  EventType_start,
} EventType;

struct Event {
  struct Event* next;
  float plane;
  int axis;
  EventType type;
  uint64_t triangle;
};

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

void generateEvents(Events *events, TrianglesStates trianglesStates, bool onlyTrianglesOnBothSides)
{
  for (int axis = 0; axis < 3; axis++) {
    Event* eventPool = malloc(sizeof(Event) * trianglesStates.count);
    uint64_t eventIdx = 0;
    for (uint64_t i = 0; i < trianglesStates.count; i++) {
      if (onlyTrianglesOnBothSides && trianglesStates.side[i] != TriangleState_Side_Both)
        continue;

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

        eventIdx += 1;
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
    events->firstEvent[axis] = eventPool;
    events->lastEvent[axis] = &eventPool[eventIdx - 1];
    for (uint64_t i = 0; i < eventIdx - 1; i++)
      eventPool[i].next = &eventPool[i + 1];
    events->lastEvent[axis]->next = NULL;
    events->pool = eventPool;
  }
}


void generateSortedEventsForSplitTriangles(Events* events, TrianglesStates trianglesStates)
{
  generateEvents(events, trianglesStates, true);
}


void initSortedEvents(Events *events, TrianglesStates trianglesStates)
{
  generateEvents(events, trianglesStates, false);
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
