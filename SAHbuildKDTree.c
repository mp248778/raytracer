#include <KDTreeNode.h>
#include <AABB.h>
#include <SAHBuildKDTree/TrianglesStates.h>
#include <SAHBuildKDTree/Events.h>

#include <stdio.h>

KDTreeNode* SAHBuildKDTree_(TrianglesStates trianglesStates, Events events, const AABB aabb)
{
  float splitConst;
  Event* splitEvent = findSplitEvent(events, &splitCost);

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
