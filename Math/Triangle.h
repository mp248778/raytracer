#ifndef Math_Triangle
#define Math_Triangle

#include <Math/Vector.h>
#include <stdint.h>

typedef struct {
  Vector vertice[3];
} Triangle;

typedef struct {
  Triangle* triangle;
  uint64_t count;
} Triangles;

#endif
