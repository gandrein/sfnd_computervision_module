#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "structIO.hpp"

using namespace std;

void showLidarTopview() {
  std::vector<LidarPoint> lidarPoints;
  readLidarPts("../data/C51_LidarPts_0000.dat", lidarPoints);

  cv::Size worldSize(10.0, 20.0);  // display width and height of sensor field in m (for top-view)
  cv::Size imageSize(1000, 2000);  // corresponding top view image in pixel

  // create topview image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

  // plot Lidar points into image
  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
    float xw = it->x;  // world position in m with x facing forward from sensor
    float yw = it->y;  // world position in m with y facing left from sensor

    // scale and place/mirror with respect to bottom left corner; (image coordinates are with respect to top left corner
    // (0,0))
    int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
    int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

    /* STUDENT ASSIGNMENT
     * 2. Remove all Lidar points on the road surface while preserving
     * measurements on the obstacles in the scene.
     */
    if (it->z > -1.4) {
      /* STUDENT ASSIGNMENT
       * 1.Change the color of the Lidar points such that
       * X=0.0m corresponds to red while X=20.0m is shown as green.
       */
      float maxX = worldSize.height;
      // Scaling factor 1 at x = 0 and 0 at x = 20.0 (max horizon)
      float scaling = abs(maxX - it->x) / maxX;
      int greenComponent = min(255, static_cast<int>(255 * (1 - scaling)));
      int redComponent = min(255, static_cast<int>(255 * scaling));

      cv::Scalar color = cv::Scalar(0, greenComponent, redComponent);

      cv::circle(topviewImg, cv::Point(x, y), 5, color, -1);
    }
  }

  // plot distance markers
  float lineSpacing = 2.0;  // gap between distance markers
  int nMarkers = floor(worldSize.height / lineSpacing);
  for (size_t i = 0; i < nMarkers; ++i) {
    int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
    cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
  }

  // display image
  string windowName = "opencv-show";
  cv::namedWindow(windowName, cv::WINDOW_NORMAL);
  cv::resizeWindow(windowName, 100, 200);  // need this for Arch i3 otherwise it is not shown properly
  cv::imshow(windowName, topviewImg);

  while ((cv::waitKey() & 0xEFFFFF) != 27) {
    continue;
  }  // wait for esc input before continuing
}

int main() { showLidarTopview(); }
