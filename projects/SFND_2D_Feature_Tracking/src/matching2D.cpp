#include <numeric>
#include "matching2D.hpp"

using namespace std;

inline std::string DetectorMethodToString(DetectorMethod &v) {
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

inline std::string DescriptorMethodToString(DescriptorMethod &v) {
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

void detectKeypoints(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool visualize) {
  switch (detector) {
    case DetectorMethod::SHITOMASI:
      detKeypointsShiTomasi(keypoints, img);
      break;
    case DetectorMethod::HARRIS:
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
    string windowName = DetectorMethodToString(detector) + " Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    while ((cv::waitKey() & 0xEFFFFF) != 27) {
      continue;
    }  // wait for keyboard input before continuing
  }
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  detectKeypointsClassic(keypoints, img, false);
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  detectKeypointsClassic(keypoints, img, true);
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

void detKeypointsModern(DetectorMethod DetectorMethod, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) {
  cv::Ptr<cv::FeatureDetector> detector = nullptr;
  auto tick = cv::getTickCount();
  switch (DetectorMethod) {
    case DetectorMethod::FAST: {
      int threshold = 40;  // difference between intensity of the central pixel and pixels of a circle around this pixel
      bool setNMS = true;  // perform non-maxima suppression on keypoints
      cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16;
      detector = cv::FastFeatureDetector::create(threshold, setNMS, type);
      break;
    }
    case DetectorMethod::BRISK:
      detector = cv::BRISK::create();
      break;
    case DetectorMethod::ORB:
      detector = cv::ORB::create();
      break;
    case DetectorMethod::AKAZE:
      detector = cv::AKAZE::create();
      break;
    case DetectorMethod::SIFT:
      detector = cv::xfeatures2d::SIFT::create();
      break;
    default:
      std::cout << "Unknown detector method!" << std::endl;
      return;
  }

  detector->detect(img, keypoints);
  double t = static_cast<double>((cv::getTickCount() - tick)) / cv::getTickFrequency();
  std::string detectorName = DetectorMethodToString(DetectorMethod);
  cout << detectorName << " with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(DescriptorMethod descriptor, vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors) {
  // select appropriate descriptor
  cv::Ptr<cv::DescriptorExtractor> extractor;
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
      return;
  }
  // perform feature description
  double t = (double)cv::getTickCount();
  extractor->compute(img, keypoints, descriptors);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << DescriptorMethodToString(descriptor) << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
  std::cout << "#3 : EXTRACT DESCRIPTORS done" << endl;
}

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                      cv::Mat &descRef, std::vector<cv::DMatch> &matches, DescriptorMethod descriptorMethod,
                      DescriptorEncoding descrEncoding, MatcherMethod matcherMethod, NeighborSelectorMethod nnSelector,
                      bool crossCheck) {
  // configure matcher
  cv::Ptr<cv::DescriptorMatcher> matcher;
  int normType = selectNormTypeMatcher(descriptorMethod, descrEncoding);
  /*
   * TASK MP.5 -> add FLANN matching
   */
  // Select matcher method to be used
  switch (matcherMethod) {
    case MatcherMethod::BRUTE_FORCE:
      matcher = cv::BFMatcher::create(normType, crossCheck);
      break;
    case MatcherMethod::FLANN: {
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
    default:
      std::cout << "Unknown descriptor matcher method!" << std::endl;
      return;
  }

  // Perform actual matching
  /*
   * TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering
   * with t=0.8
   */
  switch (nnSelector) {
    case NeighborSelectorMethod::NN: {
      double t = (double)cv::getTickCount();
      matcher->match(descSource, descRef, matches);  // Finds the best match for each descriptor in desc1
      t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
      std::cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << std::endl;
      break;
    }
    case NeighborSelectorMethod::kNN: {
      // k nearest neighbors (k=2)
      int desiredNumMatches = 2;
      double minDescriptorDistRatio = 0.8;
      runKNN(descSource, descRef, matches, matcher, desiredNumMatches, minDescriptorDistRatio);

    } break;
    default:
      std::cout << "Unknown keypoint/descriptor matching method: allowed NN/kNN only!" << std::endl;
      return;
  }
  std::cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl;
}

int selectNormTypeMatcher(DescriptorMethod descriptorMethod, DescriptorEncoding descrEncoding) {
  switch (descrEncoding) {
    case DescriptorEncoding::BINARY:
      if (descriptorMethod == DescriptorMethod::SIFT) {
        std::cout << "Enforce L2 norm type for SIFT descriptor" << std::endl;
        return static_cast<int>(cv::NORM_L2);
      }
      return static_cast<int>(cv::NORM_HAMMING);
    case DescriptorEncoding::HOG:
      return static_cast<int>(cv::NORM_L2);
    default:
      std::cout << "Unknown encoding for descriptor: supported binary(0)/hog(1)! Using binaray" << std::endl;
      return static_cast<int>(cv::NORM_HAMMING);
  }
}

void runKNN(cv::Mat &descSource, cv::Mat &descRef, std::vector<cv::DMatch> &matches,
            cv::Ptr<cv::DescriptorMatcher> &matcher, int desiredNumMatches, double minDescriptorDistRatio) {
  std::vector<std::vector<cv::DMatch>> knn_matches;
  double t = (double)cv::getTickCount();
  matcher->knnMatch(descSource, descRef, knn_matches, desiredNumMatches);
  t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
  std::cout << " (kNN) with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << std::endl;
  for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it) {
    if ((*it)[0].distance < minDescriptorDistRatio * (*it)[1].distance) {
      matches.push_back((*it)[0]);
    }
  }
  std::cout << "# keypoints removed = " << knn_matches.size() - matches.size() << std::endl;
}
