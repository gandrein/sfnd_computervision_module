#ifndef matching2D_hpp
#define matching2D_hpp

#include <stdio.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"

double detectKeypoints(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                       bool visualize = false);
double detectKeypointsClassic(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool useHarris);
double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img);
double detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img);
double detKeypointsModern(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img);

double descKeypoints(DescriptorMethod descriptor, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                     cv::Mat &descriptors);

double matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                        cv::Mat &descRef, std::vector<cv::DMatch> &matches, DescriptorMethod descriptorMethod,
                        DescriptorMetric descrMetric, MatcherMethod matcherMethod, NeighborSelectorMethod nnSelector,
                        bool crossCheck);
int selectNormTypeMatcher(DescriptorMethod descriptorMethod, DescriptorMetric descrMetric);
double runKNN(cv::Mat &descSource, cv::Mat &descRef, std::vector<cv::DMatch> &matches,
              cv::Ptr<cv::DescriptorMatcher> &matcher, int desiredNumMatches, double minDescriptorDistRatio);

std::string DetectorMethodToString(int value);
std::string DescriptorMethodToString(int value);

#endif /* matching2D_hpp */
