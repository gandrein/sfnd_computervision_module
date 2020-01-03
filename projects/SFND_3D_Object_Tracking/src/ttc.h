
#ifndef TTC_H_
#define TTC_H_

#include <stdio.h>
#include "dataStructures.h"
#include "utils.h"

void evalTTC(DataFrame &currFrame, DataFrame &prevFrame, cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT,
             double sensorFrameRate,bool visualize);

void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg = nullptr);
void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr,
                     double frameRate, double &TTC);

#endif /* TTC_H_ */
