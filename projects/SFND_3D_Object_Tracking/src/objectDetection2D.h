
#ifndef OBJECT_DETECTION_H_
#define OBJECT_DETECTION_H_

#include <stdio.h>
#include <opencv2/core.hpp>

#include "dataStructures.h"

void detectObjects(cv::Mat& img, std::vector<BoundingBox>& bBoxes, float confThreshold, float nmsThreshold,
                   std::string basePath, std::string classesFile, std::string modelConfiguration,
                   std::string modelWeights, bool bVis);

#endif /* OBJECT_DETECTION_H_ */
