#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#include "dataStructures.h"
#include "utils.h"

void loadNextImage(std::string imgFullFilename, cv::Mat &imgGray) {
  cv::Mat img;
  img = cv::imread(imgFullFilename);
  cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
}

std::string getDatasetImageName(DataSetConfig &dataInfo, size_t imgIndex) {
  std::ostringstream imgNumber;
  imgNumber << std::setfill('0') << std::setw(dataInfo.imgFillWidth) << dataInfo.imgStartIndex + imgIndex;
  std::stringstream ss;
  ss << dataInfo.imgBasePath << dataInfo.imgPrefix << imgNumber.str() << dataInfo.imgFileType;
  return ss.str();
}

void pushToBuffer(std::vector<DataFrame> &buffer, DataFrame &newFrame) {
  int dataBufferSize = 2;  // no. of images which are held in memory (ring buffer) at the same time
  if (buffer.size() < dataBufferSize) {
    buffer.emplace_back(newFrame);
    std::cout << "Initializing buffer; Buffer size is: " << buffer.size() << std::endl;
  } else {
    std::rotate(buffer.begin(), buffer.begin() + 1, buffer.end());
    buffer.pop_back();
    buffer.emplace_back(newFrame);
    std::cout << "Updating buffer; Buffer size is: " << buffer.size() << std::endl;
  }

  std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;
}

bool isInsideROI(cv::KeyPoint &kpt, cv::Rect &rectangle) {
  cv::Point2f point = kpt.pt;
  // std::cout << "Point has x: " << keypoint.x << ", y:" << keypoint.y << std::endl;
  cv::Point2f topLeft(rectangle.x, rectangle.y);
  cv::Point2f topRight(rectangle.x + rectangle.width, rectangle.y);
  cv::Point2f bottomLeft(rectangle.x, rectangle.y + rectangle.height);
  cv::Point2f bottomRight(rectangle.x + rectangle.width, rectangle.y + rectangle.height);

  if (point.x >= topLeft.x && point.x <= topRight.x && point.y >= topLeft.y && point.y <= bottomLeft.y) {
    return true;
  }
  return false;
}

void filterKeypointsROI(cv::Rect &rectangle, std::vector<cv::KeyPoint> &keypoints) {
  auto newEnd = std::remove_if(keypoints.begin(), keypoints.end(),
                               [&rectangle](cv::KeyPoint kpt) { return !isInsideROI(kpt, rectangle); });
  keypoints.erase(newEnd, keypoints.end());
}

void filterKeypointsNumber(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, size_t maxNumber) {
  if (detector == DetectorMethod::SHITOMASI || detector == DetectorMethod::HARRIS) {
    // there is no response info, so keep the first 50 as they are sorted in descending quality order
    keypoints.erase(keypoints.begin() + maxNumber, keypoints.end());
  } else {
    //   DetectorMethod::FAST:
    //   DetectorMethod::BRISK:
    //   DetectorMethod::ORB:
    //   DetectorMethod::AKAZE:
    //   DetectorMethod::SIFT:
    cv::KeyPointsFilter::retainBest(keypoints, maxNumber);
  }
  std::cout << " NOTE: Keypoints have been limited!" << std::endl;
}

Distribution evalDistribution(std::vector<cv::KeyPoint> &keypoints) {
  Distribution dist;
  double sum = std::accumulate(keypoints.begin(), keypoints.end(), 0.0,
                               [](int sum, const cv::KeyPoint &kpt) { return sum + static_cast<double>(kpt.size); });
  double mean = sum / keypoints.size();

  std::vector<double> diff(keypoints.size());
  std::transform(keypoints.begin(), keypoints.end(), diff.begin(),
                 [mean](cv::KeyPoint &kpt) { return kpt.size - mean; });
  double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
  double stdev = std::sqrt(sq_sum / keypoints.size());
  dist.mean = mean;
  dist.stdev = stdev;
  return dist;
}

void initSummaryFile(std::ofstream &ost, DetectorMethod detector, DescriptorMethod descriptor) {
  std::string fname = "../output/results_" + DetectorMethodToString(detector) + "_" +
                      DescriptorMethodToString(descriptor) + "_summary.csv";
  ost = std::ofstream(fname, std::ios::binary);

  if (!ost.is_open()) {
    std::cerr << "Cannot save results, failed to open results file: " << fname << std::endl;
  } else {
    // write CSV header row
    ost << "Frame"
        << ","
        << "DetectorType"
        << ","
        << "DescriptorType"
        << ","
        << "KeyPointsPerFrame"
        << ","
        << "KeyPointsPerROI"
        << ","
        << "KeypointSizeMean"
        << ","
        << "KeypointSizeStdDev"
        << ","
        << "DetectorTime(ms)"
        << ","
        << "DescriptorTime(ms)"
        << ","
        << "MatchedPoints"
        << ","
        << "MatchingTime(ms)"
        << ","
        << "TotalTime(ms)" << std::endl;
  }
}

void initDistributionDataFile(std::ofstream &ost, DetectorMethod detector) {
  std::string fname = "../output/results_" + DetectorMethodToString(detector) + "_kpts_distribution_data.csv";
  ost = std::ofstream(fname, std::ios::binary);

  if (!ost.is_open()) {
    std::cerr << "Cannot save distribution data, failed to open results file: " << fname << std::endl;
  } else {
    // write CSV header row
    ost << "Frame"
        << ","
        << "DetectorType"
        << ","
        << "KeyPointsPerFrame"
        << ","
        << "KeyPointsPerROI"
        << ","
        << "KeypointSizeData" << std::endl;
  }
}

void appendToSummaryInfoFile(std::ofstream &ost, DetectionResult &stats) {
  if (!ost.is_open()) {
    std::cerr << "Cannot save results, failed to open summary results file" << std::endl;
  } else {
    ost << stats.imageIndex << "," << DetectorMethodToString(stats.detector) << ","
        << DescriptorMethodToString(stats.descriptor) << "," << stats.numKeypointsFrame << "," << stats.numKeypointsROI
        << "," << stats.keypointMeanSizeROI << "," << stats.keypointStddevSizeROI << ","
        << 1000.0 * stats.detectionComputeTimeSec << "," << 1000.0 * stats.descriptorComputeTimeSec << ","
        << stats.numMatches << "," << 1000.0 * stats.matchesComputeTimeSec << ", "
        << 1000.0 * (stats.detectionComputeTimeSec + stats.descriptorComputeTimeSec + stats.matchesComputeTimeSec)
        << std::endl;
  }
}

void appendToDistributionInfoFile(std::ofstream &ost, DetectionResult &stats, std::vector<cv::KeyPoint> &keypoints) {
  if (!ost.is_open()) {
    std::cerr << "Cannot save results, failed to open distribution results file" << std::endl;
  } else {
    // ost << "Frame"
    // << ","
    // << "DetectorType"
    // << ","
    // << "KeyPointsPerFrame"
    // << ","
    // << "KeyPointsPerROI"
    // << ","
    // << "KeypointSizeData" << std::endl;

    ost << stats.imageIndex << "," << DetectorMethodToString(stats.detector) << "," << stats.numKeypointsFrame << ","
        << stats.numKeypointsROI;
    for (auto it = keypoints.begin(); it != keypoints.end(); it++) {
      ost << "," << (*it).size;
    }
    ost << std::endl;
  }
}