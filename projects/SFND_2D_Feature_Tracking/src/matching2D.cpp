#include <numeric>

#include "matching2D.hpp"
#include "utils.h"

using namespace std;

double detectKeypoints(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool visualize) {
  double timeDetector;
  std::cout << "#2 : DETECT KEYPOINTS" << std::endl;
  switch (detector) {
    case DetectorMethod::SHITOMASI:
      timeDetector = detKeypointsShiTomasi(keypoints, img);
      break;
    case DetectorMethod::HARRIS:
      timeDetector = detKeypointsHarris(keypoints, img);
      break;
    default:
      timeDetector = detKeypointsModern(detector, keypoints, img);
      break;
  }
  // visualize results
  if (visualize) {
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    string windowName = DetectorMethodToString(detector) + " Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    while ((cv::waitKey() & 0xEFFFFF) != 27) {
      continue;
    }  // wait for keyboard input before continuing
  }
  std::cout << ">>> : DETECT KEYPOINTS done" << std::endl;
  return timeDetector;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  return detectKeypointsClassic(keypoints, img, false);
}

double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  return detectKeypointsClassic(keypoints, img, true);
}

double detectKeypointsClassic(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool useHarris) {
  //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
  int blockSize = 4;
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
  return t;
}

double detKeypointsModern(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  cv::Ptr<cv::FeatureDetector> detectorPtr = nullptr;
  auto tick = cv::getTickCount();
  switch (detector) {
    case DetectorMethod::FAST: {
      int threshold = 40;  // difference between intensity of the central pixel and pixels of a circle around this pixel
      bool setNMS = true;  // perform non-maxima suppression on keypoints
      cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
      detectorPtr = cv::FastFeatureDetector::create(threshold, setNMS, type);
      break;
    }
    case DetectorMethod::BRISK:
      detectorPtr = cv::BRISK::create();
      break;
    case DetectorMethod::ORB: {
      int maxNumberFeatures = 500;
      detectorPtr = cv::ORB::create(maxNumberFeatures);
      break;
    }
    case DetectorMethod::AKAZE:
      detectorPtr = cv::AKAZE::create();
      break;
    case DetectorMethod::SIFT:
      detectorPtr = cv::xfeatures2d::SIFT::create();
      break;
    default:
      std::cout << "Unknown detector method!" << std::endl;
      return 0.0;
  }

  detectorPtr->detect(img, keypoints);
  double t = static_cast<double>((cv::getTickCount() - tick)) / cv::getTickFrequency();
  std::string detectorName = DetectorMethodToString(detector);
  std::cout << detectorName << " with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms"
            << std::endl;
  return t;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
double descKeypoints(DescriptorMethod descriptor, vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors) {
  // select appropriate descriptor
  cv::Ptr<cv::DescriptorExtractor> extractor;
  std::cout << "#3 : EXTRACT DESCRIPTORS" << std::endl;
  switch (descriptor) {
    case DescriptorMethod::BRISK: {
      int threshold = 30;         // FAST/AGAST detection threshold score.
      int octaves = 3;            // detection octaves (use 0 to do single scale)
      float patternScale = 1.0f;  // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

      extractor = cv::BRISK::create(threshold, octaves, patternScale);
      break;
    }
    case DescriptorMethod::AKAZE:
      extractor = cv::AKAZE::create();
      break;
    case DescriptorMethod::BRIEF: {
      int descriptorByteSize = 64;
      extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(descriptorByteSize);
      break;
    }
    case DescriptorMethod::FREAK:
      extractor = cv::xfeatures2d::FREAK::create();
      break;
    case DescriptorMethod::ORB:
      extractor = cv::ORB::create();
      break;
    case DescriptorMethod::SIFT:
      extractor = cv::xfeatures2d::FREAK::create();
      break;
    default:
      std::cout << "Unknown detector method!" << std::endl;
      return 0.0;
  }
  // perform feature description
  double t = (double)cv::getTickCount();
  extractor->compute(img, keypoints, descriptors);
  double timeDescriptor = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << DescriptorMethodToString(descriptor) << " descriptor extraction in " << 1000 * timeDescriptor / 1.0
            << " ms" << endl;
  std::cout << ">>> : EXTRACT DESCRIPTORS done" << std::endl;
  return timeDescriptor;
}

// Find best matches for keypoints in two camera images based on several matching methods
double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                        cv::Mat &descRef, std::vector<cv::DMatch> &matches, DescriptorMethod descriptorMethod,
                        DescriptorMetric descrMetric, MatcherMethod matcherMethod, NeighborSelectorMethod nnSelector,
                        bool crossCheck) {
  // configure matcher
  cv::Ptr<cv::DescriptorMatcher> matcher;
  int normType = selectNormTypeMatcher(descriptorMethod, descrMetric);
  /*
   * TASK MP.5 -> add FLANN matching
   */
  // Select matcher method to be used
  switch (matcherMethod) {
    case MatcherMethod::FLANN: {
      std::cout << "Using FLANN matching ..." << std::endl;
      if (descSource.type() != CV_32F) {
        // OpenCV bug workaround : convert binary descriptors to floating point due to
        // a bug in current OpenCV implementation
        std::cout << "Bypassing OpenCV FLANN implementation bug ..." << std::endl;
        descSource.convertTo(descSource, CV_32F);
      }
      if (descRef.type() != CV_32F) {
        descRef.convertTo(descRef, CV_32F);
      }
      matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
      break;
    }
    case MatcherMethod::BRUTE_FORCE:
      std::cout << "Using BRUTE_FORCE matching ..." << std::endl;
      matcher = cv::BFMatcher::create(normType, crossCheck);
      break;
    default:
      std::cout << "Unknown descriptor matcher method! Defaulting to BRUTE_FORCE" << std::endl;
      matcher = cv::BFMatcher::create(normType, crossCheck);
  }

  // Perform actual matching
  /*
   * TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering
   * with t=0.8
   */
  double timeMatching = 0.0;
  switch (nnSelector) {
    case NeighborSelectorMethod::NN: {
      std::cout << "Using NN match selection ..." << std::endl;
      double t = (double)cv::getTickCount();
      matcher->match(descSource, descRef, matches);  // Finds the best match for each descriptor in desc1
      timeMatching = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
      std::cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * timeMatching / 1.0 << " ms"
                << std::endl;
      break;
    }
    case NeighborSelectorMethod::kNN: {
      std::cout << "Using kNN match selection ..." << std::endl;
      // k nearest neighbors (k=2)
      int desiredNumMatches = 2;
      double minDescriptorDistRatio = 0.8;
      timeMatching = runKNN(descSource, descRef, matches, matcher, desiredNumMatches, minDescriptorDistRatio);

    } break;
    default:
      std::cout << "Unknown keypoint/descriptor matching method: allowed NN/kNN only!" << std::endl;
      return 0.0;
  }
  std::cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;
  return timeMatching;
}

int selectNormTypeMatcher(DescriptorMethod descriptorMethod, DescriptorMetric descrMetric) {
  switch (descrMetric) {
    case DescriptorMetric::BINARY:
      if (descriptorMethod == DescriptorMethod::SIFT) {
        std::cout << "Enforce L2 norm type for SIFT descriptor" << std::endl;
        return static_cast<int>(cv::NORM_L2);
      }
      return static_cast<int>(cv::NORM_HAMMING);
    case DescriptorMetric::HOG:
      return static_cast<int>(cv::NORM_L2);
    default:
      std::cout << "Unknown encoding for descriptor: supported binary(0)/hog(1)! Using binaray" << std::endl;
      return static_cast<int>(cv::NORM_HAMMING);
  }
}

double runKNN(cv::Mat &descSource, cv::Mat &descRef, std::vector<cv::DMatch> &matches,
              cv::Ptr<cv::DescriptorMatcher> &matcher, int desiredNumMatches, double minDescriptorDistRatio) {
  std::vector<std::vector<cv::DMatch>> knn_matches;
  double time = (double)cv::getTickCount();
  matcher->knnMatch(descSource, descRef, knn_matches, desiredNumMatches);
  time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
  std::cout << " (kNN) with n=" << knn_matches.size() << " matches in " << 1000 * time / 1.0 << " ms" << std::endl;
  for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it) {
    if ((*it)[0].distance < minDescriptorDistRatio * (*it)[1].distance) {
      matches.push_back((*it)[0]);
    }
  }
  std::cout << "# keypoints removed = " << knn_matches.size() - matches.size() << std::endl;
  return time;
}
