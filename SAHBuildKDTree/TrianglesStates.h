#ifndef SAHBuildKDTree_TrianglesStates
#define SAHBuildKDTree_TrianglesStates

#include <Math/Triangle.h>
#include <stdint.h>

typedef enum {
  TriangleState_Side_Both,
  TriangleState_Side_Left,
  TriangleState_Side_Right,
} TriangleStateSide;

typedef struct {
  Triangle* triangles;
  uint64_t count;
  TriangleStateSide* side;
} TrianglesStates;

#endif
