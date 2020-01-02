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

void loadKittiCalibrationData(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT){
  // ANG: THESE values should be parsed/read from the KITTI camera calibration files
  RT.at<double>(0, 0) = 7.533745e-03;
  RT.at<double>(0, 1) = -9.999714e-01;
  RT.at<double>(0, 2) = -6.166020e-04;
  RT.at<double>(0, 3) = -4.069766e-03;
  RT.at<double>(1, 0) = 1.480249e-02;
  RT.at<double>(1, 1) = 7.280733e-04;
  RT.at<double>(1, 2) = -9.998902e-01;
  RT.at<double>(1, 3) = -7.631618e-02;
  RT.at<double>(2, 0) = 9.998621e-01;
  RT.at<double>(2, 1) = 7.523790e-03;
  RT.at<double>(2, 2) = 1.480755e-02;
  RT.at<double>(2, 3) = -2.717806e-01;
  RT.at<double>(3, 0) = 0.0;
  RT.at<double>(3, 1) = 0.0;
  RT.at<double>(3, 2) = 0.0;
  RT.at<double>(3, 3) = 1.0;

  R_rect_00.at<double>(0, 0) = 9.999239e-01;
  R_rect_00.at<double>(0, 1) = 9.837760e-03;
  R_rect_00.at<double>(0, 2) = -7.445048e-03;
  R_rect_00.at<double>(0, 3) = 0.0;
  R_rect_00.at<double>(1, 0) = -9.869795e-03;
  R_rect_00.at<double>(1, 1) = 9.999421e-01;
  R_rect_00.at<double>(1, 2) = -4.278459e-03;
  R_rect_00.at<double>(1, 3) = 0.0;
  R_rect_00.at<double>(2, 0) = 7.402527e-03;
  R_rect_00.at<double>(2, 1) = 4.351614e-03;
  R_rect_00.at<double>(2, 2) = 9.999631e-01;
  R_rect_00.at<double>(2, 3) = 0.0;
  R_rect_00.at<double>(3, 0) = 0;
  R_rect_00.at<double>(3, 1) = 0;
  R_rect_00.at<double>(3, 2) = 0;
  R_rect_00.at<double>(3, 3) = 1;

  P_rect_00.at<double>(0, 0) = 7.215377e+02;
  P_rect_00.at<double>(0, 1) = 0.000000e+00;
  P_rect_00.at<double>(0, 2) = 6.095593e+02;
  P_rect_00.at<double>(0, 3) = 0.000000e+00;
  P_rect_00.at<double>(1, 0) = 0.000000e+00;
  P_rect_00.at<double>(1, 1) = 7.215377e+02;
  P_rect_00.at<double>(1, 2) = 1.728540e+02;
  P_rect_00.at<double>(1, 3) = 0.000000e+00;
  P_rect_00.at<double>(2, 0) = 0.000000e+00;
  P_rect_00.at<double>(2, 1) = 0.000000e+00;
  P_rect_00.at<double>(2, 2) = 1.000000e+00;
  P_rect_00.at<double>(2, 3) = 0.000000e+00;
}

std::string getImageNumberAsString(DataSetConfig &dataInfo, size_t imgIndex) {
  std::ostringstream imgNumber;
  imgNumber << std::setfill('0') << std::setw(dataInfo.indexNameWidth) << dataInfo.startIndex + imgIndex;
  return imgNumber.str();
}

std::string getDatasetImageName(DataSetConfig &dataInfo, size_t imgIndex) {
  std::stringstream ss;
  ss << dataInfo.basePath << dataInfo.prefix << getImageNumberAsString(dataInfo, imgIndex) << dataInfo.fileType;
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