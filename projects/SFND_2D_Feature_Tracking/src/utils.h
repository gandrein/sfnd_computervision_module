#ifndef UTILS_H
#define UTILS_H

#include <functional>
#include <numeric>
#include <opencv2/core.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "dataStructures.h"

struct DataSetConfig {
  std::string imgBasePath;
  std::string imgPrefix;
  std::string imgFileType;
  int imgStartIndex = 0;  // first file index to load (assumes Lidar and camera names have identical naming convention)
  int imgEndIndex = 9;    // last file index to load
  int imgFillWidth = 4;   // no. of digits which make up the file index (e.g. img-0001.png)
};

struct Distribution {
  double mean;
  double stdev;
};

void loadNextImage(std::string imgFullFilename, cv::Mat &imgGray);
std::string getDatasetImageName(DataSetConfig &dataInfo, size_t imgIndex);

void pushToBuffer(std::vector<DataFrame> &buffer, DataFrame &newFrame);

bool isInsideROI(cv::KeyPoint &kpt, cv::Rect &rectangle);

void filterKeypointsROI(cv::Rect &rectangle, std::vector<cv::KeyPoint> &keypoints);

void filterKeypointsNumber(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, size_t maxNumber);

Distribution evalDistribution(std::vector<cv::KeyPoint> &keypoints);

inline std::string DetectorMethodToString(const DetectorMethod &v) {
  switch (v) {
    case DetectorMethod::SHITOMASI:
      return "SHI-TOMASI";
    case DetectorMethod::HARRIS:
      return "HARRIS";
    case DetectorMethod::AKAZE:
      return "AKAZE";
    case DetectorMethod::BRISK:
      return "BRISK";
    case DetectorMethod::FAST:
      return "FAST";
    case DetectorMethod::ORB:
      return "ORB";
    case DetectorMethod::SIFT:
      return "SIFT";
    default:
      return "[Unknown DetectorMethod]";
  }
}

inline std::string DescriptorMethodToString(const DescriptorMethod &v) {
  switch (v) {
    case DescriptorMethod::BRISK:
      return "BRISK";
    case DescriptorMethod::AKAZE:
      return "AKAZE";
    case DescriptorMethod::BRIEF:
      return "BRIEF";
    case DescriptorMethod::FREAK:
      return "FREAK";
    case DescriptorMethod::ORB:
      return "ORB";
    case DescriptorMethod::SIFT:
      return "SIFT";
    default:
      return "[Unknown DescriptorMethod]";
  }
}

inline std::string DescriptorEncodingToString(const DescriptorEncoding &v) {
  switch (v) {
    case DescriptorEncoding::BINARY:
      return "BINARY";
    case DescriptorEncoding::HOG:
      return "HOG";
    default:
      return "[Unknown DescriptorEncoding]";
  }
}

inline std::string MatcherMethodToString(const MatcherMethod &v) {
  switch (v) {
    case MatcherMethod::BRUTE_FORCE:
      return "BRUTE_FORCE";
    case MatcherMethod::FLANN:
      return "FLANN";
    default:
      return "[Unknown MatcherMethod]";
  }
}

inline std::string NeighborSelectorMethodToString(const NeighborSelectorMethod &v) {
  switch (v) {
    case NeighborSelectorMethod::NN:
      return "NN";
    case NeighborSelectorMethod::kNN:
      return "kNN";
    default:
      return "[Unknown NeighborSelectorMethod]";
  }
}

inline std::string DetectorMethodToString(int value) {
  return DetectorMethodToString(static_cast<DetectorMethod>(value));
}

inline std::string DescriptorMethodToString(int value) {
  return DescriptorMethodToString(static_cast<DescriptorMethod>(value));
}

inline std::string DescriptorEncodingToString(int value) {
  return DescriptorEncodingToString(static_cast<DescriptorEncoding>(value));
}

inline std::string MatcherMethodToString(int value) { return MatcherMethodToString(static_cast<MatcherMethod>(value)); }

inline std::string NeighborSelectorMethodToString(int value) {
  return NeighborSelectorMethodToString(static_cast<NeighborSelectorMethod>(value));
}
void initSummaryFile(std::ofstream &ost, DetectorMethod detector, DescriptorMethod descriptor);
void initDistributionDataFile(std::ofstream &ost, DetectorMethod detector);
void appendToSummaryFile(std::ofstream &ost, DetectionResult &stats);
#endif