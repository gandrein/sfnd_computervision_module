#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>

#include "dataStructures.h"
#include "structIO.hpp"

using namespace std;

double findMinDistance(std::vector<LidarPoint> &lidarPoints, double laneWidth) {
  double minDist = 1e9;
  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
    if (abs(it->y) <= laneWidth / 2.0 && it->x < minDist) {
      minDist = it->x;
    }
  }
  return minDist;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr, double &TTC) {
  double dT = 0.1;         // time between two measurements in seconds
  double laneWidth = 4.0;  // assumed width of the ego lane

  // find closest distance to Lidar points within ego lane
  double minXPrev = findMinDistance(lidarPointsPrev, laneWidth);
  double minXCurr = findMinDistance(lidarPointsCurr, laneWidth);

  // compute TTC from both measurements
  TTC = minXCurr * dT / (minXPrev - minXCurr);
}

int main() {
  std::vector<LidarPoint> currLidarPts, prevLidarPts;
  readLidarPts("../data/C22A5_currLidarPts.dat", currLidarPts);
  readLidarPts("../data/C22A5_prevLidarPts.dat", prevLidarPts);

  double ttc;
  computeTTCLidar(prevLidarPts, currLidarPts, ttc);
  cout << "ttc = " << ttc << "s" << endl;
}