
#ifndef TTC_H_
#define TTC_H_

#include <stdio.h>
#include "dataStructures.h"
#include "utils.h"

void evalTTC(LidarTtcMethod lidarTtcMethod, KptMatchesClusterConf kptClusterConfig, DataFrame &currFrame,
             DataFrame &prevFrame, cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT, double sensorFrameRate,
             bool visualize);

double computeTTCLidar(LidarTtcMethod ttcMethod, std::vector<LidarPoint> &lidarPointsPrev,
                       std::vector<LidarPoint> &lidarPointsCurr, double frameRate);

double computeTTCLidarMedianBased(std::vector<double> &xLidarPrev, std::vector<double> &xLidarCurr,
                                  double lidarFrameRate);

double computeTTCLidarMeanBased(std::vector<double> &xLidarPrev, std::vector<double> &xLidarCurr,
                                double lidarFrameRate);

double computeTTCLidarClusterBased(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr,
                                   double frameRate);

double computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                        std::vector<cv::DMatch> kptMatches, double frameRate, cv::Mat *visImg = nullptr);

#endif /* TTC_H_ */
