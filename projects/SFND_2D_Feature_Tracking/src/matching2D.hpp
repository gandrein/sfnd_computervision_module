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

void detectKeypoints(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                     bool visualize = false);
void detectKeypointsClassic(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool useHarris);
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img);
void detKeypointsShiTomasi(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img);
void detKeypointsModern(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img);

void descKeypoints(DescriptorMethod descriptor, std::vector<cv::KeyPoint> &keypoints, cv::Mat &img,
                   cv::Mat &descriptors);

void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource,
                      cv::Mat &descRef, std::vector<cv::DMatch> &matches, DescriptorMethod descriptorMethod,
                      DescriptorEncoding descrEncoding, MatcherMethod matcherMethod, NeighborSelectorMethod nnSelector,
                      bool crossCheck);
int selectNormTypeMatcher(DescriptorMethod descriptorMethod, DescriptorEncoding descrEncoding);
void runKNN(cv::Mat &descSource, cv::Mat &descRef, std::vector<cv::DMatch> &matches,
            cv::Ptr<cv::DescriptorMatcher> &matcher, int desiredNumMatches, double minDescriptorDistRatio);

#endif /* matching2D_hpp */
