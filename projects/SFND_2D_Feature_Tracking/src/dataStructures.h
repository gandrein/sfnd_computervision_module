#ifndef dataStructures_h
#define dataStructures_h

#include <opencv2/core.hpp>
#include <vector>

enum DetectorType { SHITOMASI = 0, HARRIS = 1, AKAZE = 2, BRISK = 3, FAST = 4, ORB = 5, SIFT = 6 };

struct DataFrame {  // represents the available sensor information at the same time instance

  cv::Mat cameraImg;  // camera image

  std::vector<cv::KeyPoint> keypoints;  // 2D keypoints within camera image
  cv::Mat descriptors;                  // keypoint descriptors
  std::vector<cv::DMatch> kptMatches;   // keypoint matches between previous and current frame
};

#endif /* dataStructures_h */
