#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

struct NmsParams {
  // KeyPoints Non-maximum suppression parameters
  double maxOverlap = 0.0;  // max allowed overlap between two corners/features
  int minResponse = 100;    // minimum value for a corner in the 8bit scaled response matrix
};

bool checkOverlap(cv::KeyPoint& keyPoint1, cv::KeyPoint& keyPoint2, NmsParams& nmsParams) {
  bool overlap = false;
  double kptOveralpArea = cv::KeyPoint::overlap(keyPoint1, keyPoint2);
  return kptOveralpArea > nmsParams.maxOverlap;
}

void nonMaximumSuppression(cv::Mat& image, std::vector<cv::KeyPoint>& keyPoints, NmsParams& nmsParams,
                           int keyPointSize) {
  for (size_t i = 0; i < image.rows; i++) {
    for (size_t j = 0; j < image.cols; j++) {
      int response = static_cast<int>(image.at<float>(i, j));
      if (response > nmsParams.minResponse) {  // only store as keypoint, points above minimum threshold
        cv::KeyPoint keyPoint;
        keyPoint.pt = cv::Point2f(j, i);
        keyPoint.size = 2 * keyPointSize;
        keyPoint.response = response;

        // perform simple non-maximum suppression (NMS) in local neighbourhood around new key point
        bool overlap = false;
        for (auto it = keyPoints.begin(); it != keyPoints.end(); ++it) {
          overlap = checkOverlap(keyPoint, *it, nmsParams);
          // replace with current keypoint only when response is higher
          // otherwise mark as overlaping and don't consider this as a keypoint
          if (overlap && keyPoint.response > (*it).response) {
            *it = keyPoint;
            break;
          }
        }
        if (!overlap) {
          keyPoints.emplace_back(keyPoint);
        }
      }
    }
  }
}
void cornernessHarris() {
  // load image from file
  cv::Mat img;
  img = cv::imread("../images/img1.png");
  cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);  // convert to grayscale

  // Detector parameters
  int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
  int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
  double k = 0.04;       // Harris parameter (see equation for details)

  // Detect Harris corners and normalize output
  cv::Mat dst, dst_norm, dst_norm_scaled;
  dst = cv::Mat::zeros(img.size(), CV_32FC1);
  cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
  cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);

  // visualize results
  string windowName = "Harris Corner Detector Response Matrix";
  cv::namedWindow(windowName, 4);
  cv::imshow(windowName, dst_norm_scaled);
  cv::waitKey(0);

  // Look for prominent corners and instantiate keypoints
  std::vector<cv::KeyPoint> keyPoints;

  NmsParams nmsSettings;
  nmsSettings.maxOverlap = 0.0;
  nmsSettings.minResponse = 100;
  nonMaximumSuppression(dst_norm, keyPoints, nmsSettings, apertureSize);

  windowName = "Harris Corner Detection Results";
  cv::namedWindow(windowName, 5);
  cv::Mat visImage = dst_norm_scaled.clone();
  cv::drawKeypoints(dst_norm_scaled, keyPoints, visImage, cv::Scalar::all(-1),
                    cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  cv::imshow(windowName, visImage);

  while ((cv::waitKey() & 0xEFFFFF) != 27) {
    continue;
  }  // wait for keyboard input before continuing
}

int main() { cornernessHarris(); }