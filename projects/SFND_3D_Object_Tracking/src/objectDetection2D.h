
#ifndef OBJECT_DETECTION_H_
#define OBJECT_DETECTION_H_

#include <stdio.h>
#include <opencv2/core.hpp>

#include "dataStructures.h"
#include "utils.h"

void detectObjects(DataFrame& frameData, YoloConfig yoloConfig, bool visualize);

#endif /* OBJECT_DETECTION_H_ */
