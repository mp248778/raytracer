/*
Copyright (c) 2012 Michal Pachucki

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <KDTree/KDTreeNode.h>
#include <Math/AABB.h>
#include <SAHBuildKDTree/TrianglesStates.h>
#include <SAHBuildKDTree/Events.h>
#include <stdio.h>
#include <stdlib.h>

#define MIN_SPLIT_NODE_COST 10.0f

static KDTreeNode createLeafNode(TrianglesStates trianglesStates, Events events)
{
  KDTreeNode leaf = calloc(1, sizeof(struct KDTreeNode));
  leaf->axis = 3;

  Triangles triangles;
  triangles.triangle = malloc(events.count[0] * sizeof(Triangle));
  triangles.count = 0;


  for(Event* currentEvent = events.firstEvent[1]; currentEvent; currentEvent = currentEvent->next) {
    if (currentEvent->type == EventType_end)
      continue;
    triangles.triangle[triangles.count] = trianglesStates.triangles[currentEvent->triangle];
    triangles.count++;
  }

  triangles.triangle = realloc(triangles.triangle, triangles.count * sizeof(Triangle));
  leaf->triangles = triangles;

  return leaf;
}

static KDTreeNode createInnerNode(KDTreeNode left, KDTreeNode right, float plane, int axis)
{
  KDTreeNode inner = calloc(1, sizeof(struct KDTreeNode));
  inner->axis = axis;
  inner->plane = plane;
  inner->left = left;
  inner->right = right;

  return inner;
}

KDTreeNode SAHBuildKDTree_(TrianglesStates trianglesStates, Events events, const AABB aabb)
{
  float splitCost;
  float splitPlane;
  int splitAxis;
  bool splitParallelLeft;
  findSplitPlane(events, &splitCost, &splitPlane, &splitAxis, &splitParallelLeft, aabb);

  if (splitCost < MIN_SPLIT_NODE_COST) {
    return createLeafNode(trianglesStates, events);
  }

  categorizeTriangles(trianglesStates, events, splitPlane, splitAxis, splitParallelLeft);

  Events newLeftEvents;
  Events newRightEvents;
  generateSortedEventsForSplitTriangles(events, trianglesStates, splitPlane, splitAxis, splitParallelLeft, &newLeftEvents, &newRightEvents);

  Events leftEvents;
  Events rightEvents;
  splitEvents(&leftEvents, &rightEvents, events, trianglesStates);
  mergeEvents(&leftEvents, newLeftEvents);
  mergeEvents(&rightEvents, newRightEvents);

  AABB leftAABB, rightAABB;
  splitAABB(&aabb, splitPlane, splitAxis, &leftAABB, &rightAABB);

  KDTreeNode leftNode = SAHBuildKDTree_(trianglesStates, leftEvents, leftAABB);
  deinitEvents(&newLeftEvents);
  KDTreeNode rightNode = SAHBuildKDTree_(trianglesStates, rightEvents, rightAABB);
  deinitEvents(&newRightEvents);

  return createInnerNode(leftNode, rightNode, splitPlane, splitAxis);
}

KDTreeNode* SAHBuildKDTree(const char* fileName)
{
  TrianglesStates trianglesStates;
  initTrianglesStates(&trianglesStates, fileName);

  Events events;
  initSortedEvents(&events, trianglesStates);

  AABB aabb;
  initAABB(&aabb, trianglesStates.triangles, trianglesStates.count);

  KDTreeNode root = SAHBuildKDTree_(trianglesStates, events, aabb);

  deinitTrianglesStates(&trianglesStates);
  deinitEvents(&events);

  return root;
}
