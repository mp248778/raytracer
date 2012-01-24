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

#define MIN_SPLIT_NODE_COST 10.0f

KDTreeNode* SAHBuildKDTree_(TrianglesStates trianglesStates, Events events, const AABB aabb)
{
  float splitCost;
  float splitPlane;
  int splitAxis;
  bool splitParallelLeft;
  findSplitPlane(events, &splitCost, &splitPlane, &splitAxis, &splitParallelLeft, aabb);

  if (splitCost < MIN_SPLIT_NODE_COST) {
    return createLeafNode(trianglesStates, events);
  }

  categorizeTriangles(trianglesStates, splitEvent);

  Events* newLeftEvents = NULL;
  Events* newRightEvents = NULL;
  generateSortedEventsForSplitTriangles(&newEvents, trianglesStates);

  Events leftEvents = NULL;
  Events rightEvents = NULL;
  TrianglesStates leftTrianglesStates
  splitEvents(&leftEvents, &rightEvents, trianglesStates);
  mergeEvents(&leftEvents, newLeftEvents);
  mergeEvents(&rightEvents, newRightEvents);

  AABB leftAABB, rightAABB;
  splitAABB(&aabb, splitEvent, &leftAABB, &rightAABB);

  KDTreeNode* leftNode = SAHBuildKDTree_(trianglesStates, leftEvents, leftAABB);
  KDTreeNode* rightNode = SAHBuildKDTree_(trianglesStates, rightEvents, rightAABB);
  deinitEvents(newEvents);
  return createInnerNode(leftNode, rightNode);
}

KDTreeNode* SAHBuildKDTree(const char* fileName)
{
  FILE *file = fopen(fileName, "r");
  TrianglesStates trianglesStates = NULL;
  initTrianglesStates(&trianglesStates, file);
  fclose(file);

  Events events = NULL;
  initSortedEvents(&events, trianglesStates);

  AABB aabb;
  initAABB(&aabb, trianglesStates);

  KDTreeNode* root = SAHBuildKDTree_(trianglesStates, events, aabb);

  deinitTrianglesStates(trianglesStates);
  deinitEvents(events);

  return root;
}
