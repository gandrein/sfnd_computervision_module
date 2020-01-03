
#ifndef LIDAR_DATA_H_
#define LIDAR_DATA_H_

#include <stdio.h>
#include <fstream>
#include <string>

#include "dataStructures.h"
double computeMedianLidarX(std::vector<LidarPoint> &vals);
double computeMeanLidarX(std::vector<LidarPoint> &vals);
void cropLidarPoints(std::vector<LidarPoint> &lidarPoints, LidarROI &roi);
void loadLidarFromFile(std::vector<LidarPoint> &lidarPoints, std::string filename);

void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, bool bWait = true);
void showLidarImgOverlay(cv::Mat &img, std::vector<LidarPoint> &lidarPoints, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx,
                         cv::Mat &RT, cv::Mat *extVisImg = nullptr);
#endif /* LIDAR_DATA_H_ */
