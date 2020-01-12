
#ifndef CAMERA_FUSION_H_
#define CAMERA_FUSION_H_

#include <stdio.h>
#include <opencv2/core.hpp>
#include <vector>

#include "dataStructures.h"

void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints,
						 float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT);

void clusterKptMatchesWithROI(KptMatchesClusterConf clusterConf, std::vector<cv::DMatch> &kptMatches,
							  DataFrame &prevFrame, DataFrame &currFrame, BoundingBox &prevBox, BoundingBox &currBox,
							  bool visualize);

std::vector<cv::DMatch> getValidEnclosedMatches(std::vector<cv::DMatch> &kptMatches,
												std::vector<cv::KeyPoint> &kptsPrev,
												std::vector<cv::KeyPoint> &kptsCurr, BoundingBox &prevBox,
												BoundingBox &currBox);

std::vector<double> evalDistanceOfKptMatches(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
											 std::vector<cv::DMatch> &kptMatches);

void matchBoundingBoxes(DataFrame &currFrame, DataFrame &prevFrame);

BoundingBox *findBoundingBoxByID(std::vector<BoundingBox> &boundingBoxes, int boxId);

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait = true);

#endif /* CAMERA_FUSION_H_ */
