#ifndef KDTree_KDTreeNode
#define KDTree_KDTreeNode

#include <Math/Triangle.h>

typedef struct KDTreeNode* KDTreeNode;

struct KDTreeNode {
  KDTreeNode left, righ;
  int axis;
  float plane;
  Triangles triangles;
};


#endif
