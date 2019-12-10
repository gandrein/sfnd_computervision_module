#include <numeric>
#include "matching2D.hpp"

using namespace std;

inline std::string DetectorTypeToString(DetectorType v) {
  switch (v) {
    case DetectorType::SHITOMASI:
      return "SHI-TOMASI";
    case DetectorType::HARRIS:
      return "HARRIS";
    case DetectorType::AKAZE:
      return "AKAZE";
    case DetectorType::BRISK:
      return "BRISK";
    case DetectorType::FAST:
      return "FAST";
    case DetectorType::ORB:
      return "ORB";
    case DetectorType::SIFT:
      return "SIFT";
    default:
      return "[Unknown DetectorType]";
  }
}

void detectKeypoints(DetectorType detector, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool visualize) {
  switch (detector) {
    case DetectorType::SHITOMASI:
      detKeypointsShiTomasi(keypoints, img);
      break;
    case DetectorType::HARRIS:
      detKeypointsHarris(keypoints, img);
      break;
    default:
      detKeypointsModern(detector, keypoints, img);
      break;
  }
  // visualize results
  if (visualize) {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = DetectorTypeToString(detector) + " Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    while ((cv::waitKey() & 0xEFFFFF) != 27) {
      continue;
    }  // wait for keyboard input before continuing
  }
}

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                      cv::Mat &descRef, std::vector<cv::DMatch> &matches, std::string descriptorType,
                      std::string matcherType, std::string selectorType) {
  // configure matcher
  bool crossCheck = false;
  cv::Ptr<cv::DescriptorMatcher> matcher;

  if (matcherType.compare("MAT_BF") == 0) {
    int normType = cv::NORM_HAMMING;
    matcher = cv::BFMatcher::create(normType, crossCheck);
  } else if (matcherType.compare("MAT_FLANN") == 0) {
    // ...
  }

  // perform matching task
  if (selectorType.compare("SEL_NN") == 0) {  // nearest neighbor (best match)

    matcher->match(descSource, descRef, matches);     // Finds the best match for each descriptor in desc1
  } else if (selectorType.compare("SEL_KNN") == 0) {  // k nearest neighbors (k=2)

    // ...
  }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType) {
  // select appropriate descriptor
  cv::Ptr<cv::DescriptorExtractor> extractor;
  if (descriptorType.compare("BRISK") == 0) {
    int threshold = 30;         // FAST/AGAST detection threshold score.
    int octaves = 3;            // detection octaves (use 0 to do single scale)
    float patternScale = 1.0f;  // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

    extractor = cv::BRISK::create(threshold, octaves, patternScale);
  } else {
    //...
  }

  // perform feature description
  double t = (double)cv::getTickCount();
  extractor->compute(img, keypoints, descriptors);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

void detectKeypointsClassic(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool useHarris) {
  // compute detector parameters based on image size
  int blockSize =
      4;  //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
  double maxOverlap = 0.0;  // max. permissible overlap between two features in %
  double minDistance = (1.0 - maxOverlap) * blockSize;
  int maxCorners = img.rows * img.cols / max(1.0, minDistance);  // max. num. of keypoints

  double qualityLevel = 0.01;  // minimal accepted quality of image corners
  double k = 0.04;

  // Apply corner detection
  double t = (double)cv::getTickCount();
  vector<cv::Point2f> corners;
  cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, useHarris, k);

  // add corners to result vector
  for (auto it = corners.begin(); it != corners.end(); ++it) {
    cv::KeyPoint newKeyPoint;
    newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
    newKeyPoint.size = blockSize;
    keypoints.push_back(newKeyPoint);
  }
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  if (useHarris) {
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
  } else {
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
  }
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  detectKeypointsClassic(keypoints, img, false);
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  detectKeypointsClassic(keypoints, img, true);
}

void detKeypointsModern(DetectorType detectorType, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  cv::Ptr<cv::FeatureDetector> detector = nullptr;
  auto tick = cv::getTickCount();
  switch (detectorType) {
    case DetectorType::FAST: {
      int threshold = 40;  // difference between intensity of the central pixel and pixels of a circle around this pixel
      bool setNMS = true;  // perform non-maxima suppression on keypoints
      cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
      detector = cv::FastFeatureDetector::create(threshold, setNMS, type);
      break;
    }
    case DetectorType::BRISK:
      detector = cv::BRISK::create();
      break;
    case DetectorType::ORB:
      detector = cv::ORB::create();
      break;
    case DetectorType::AKAZE:
      detector = cv::AKAZE::create();
      break;
    case DetectorType::SIFT:
      detector = cv::xfeatures2d::SIFT::create();
      break;
    default:
      std::cout << "Unknown detector method!" << std::endl;
      return;
  }

  detector->detect(img, keypoints);
  double t = static_cast<double>((cv::getTickCount() - tick)) / cv::getTickFrequency();
  std::string detectorName = DetectorTypeToString(detectorType);
  cout << detectorName << " with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
}