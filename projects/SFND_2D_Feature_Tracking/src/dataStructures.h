#ifndef dataStructures_h
#define dataStructures_h

#include <opencv2/core.hpp>
#include <vector>

enum class DetectorMethod { SHITOMASI = 0, HARRIS, AKAZE, BRISK, FAST, ORB, SIFT };

enum class DescriptorMethod { BRISK = 0, AKAZE, BRIEF, FREAK, ORB, SIFT };

enum class DescriptorEncoding { BINARY = 0, HOG };

enum class MatcherMethod { BRUTE_FORCE = 0, FLANN };

enum class NeighborSelectorMethod { NN = 0, kNN };  // NearestNeighbor, kNearestNeighbor

struct DataFrame {  // represents the available sensor information at the same time instance

  cv::Mat cameraImg;  // camera image

  std::vector<cv::KeyPoint> keypoints;  // 2D keypoints within camera image
  cv::Mat descriptors;                  // keypoint descriptors
  std::vector<cv::DMatch> kptMatches;   // keypoint matches between previous and current frame
};

#endif /* dataStructures_h */
