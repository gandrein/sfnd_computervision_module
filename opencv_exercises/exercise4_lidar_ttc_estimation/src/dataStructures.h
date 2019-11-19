#ifndef dataStructures_h
#define dataStructures_h

#include <opencv2/core.hpp>
#include <vector>

struct LidarPoint {
  double x, y, z;  // x,y,z in [m]
  double r;        // r is point reflectivity
};

#endif
