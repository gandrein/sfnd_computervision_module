#ifndef UTILS_H
#define UTILS_H

#include <functional>
#include <numeric>
#include <opencv2/core.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "dataStructures.h"

void loadKittiCalibrationData(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT);

std::string getImageNumberAsString(DataSetConfig &dataInfo, size_t imgIndex);

std::string getDatasetImageName(DataSetConfig &dataInfo, size_t imgIndex);

void pushToBuffer(std::vector<DataFrame> &buffer, DataFrame &newFrame);

bool isInsideROI(cv::KeyPoint &kpt, cv::Rect &rectangle);

double computeMedian(std::vector<double> &vals);

double computeMean(std::vector<double> &vals);

void filterKeypointsNumber(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, size_t maxNumber);

NormalDistribution evalNormalDistributionParams(std::vector<double> &vals);

void showYoloDetectionOnImage(DataFrame &frameData, YoloConfig yoloConfig, std::string labelPostFix = "");

void visualizeMatchedYoloBoundingBoxes(DataFrame &prev_frame, DataFrame &curr_frame);

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

inline std::string DescriptorMetricToString(const DescriptorMetric &v) {
	switch (v) {
		case DescriptorMetric::BINARY:
			return "BINARY";
		case DescriptorMetric::HOG:
			return "HOG";
		default:
			return "[Unknown DescriptorMetric]";
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

inline std::string DescriptorMetricToString(int value) {
	return DescriptorMetricToString(static_cast<DescriptorMetric>(value));
}

inline std::string MatcherMethodToString(int value) { return MatcherMethodToString(static_cast<MatcherMethod>(value)); }

inline std::string NeighborSelectorMethodToString(int value) {
	return NeighborSelectorMethodToString(static_cast<NeighborSelectorMethod>(value));
}

void showMultimapContent(std::multimap<int, int> &myMap);

void drawMatches(std::vector<cv::DMatch> &matches, DataFrame &currentFrame, DataFrame &previousFram,
				 std::string windowTitle);

#endif