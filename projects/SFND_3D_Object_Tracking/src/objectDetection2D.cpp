
#include <fstream>
#include <iostream>
#include <sstream>

#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "objectDetection2D.h"

using namespace std;

// detects objects in an image using the YOLO library and a set of pre-trained objects from the COCO database;
// a set of 80 classes is listed in "coco.names" and pre-trained weights are stored in "yolov3.weights"
void detectObjects(DataFrame& frameData, YoloConfig yoloConfig, bool visualize) {
  // load neural network
  cv::dnn::Net net = cv::dnn::readNetFromDarknet(yoloConfig.modelWeightsCfg, yoloConfig.modelWeightsFile);
  net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

  // generate 4D blob from input image
  cv::Mat blob;
  vector<cv::Mat> netOutput;
  double scalefactor = 1 / 255.0;
  cv::Size size = cv::Size(416, 416);
  cv::Scalar mean = cv::Scalar(0, 0, 0);
  bool swapRB = false;
  bool crop = false;
  cv::dnn::blobFromImage(frameData.cameraImg, blob, scalefactor, size, mean, swapRB, crop);

  // Get names of output layers
  vector<cv::String> names;
  vector<int> outLayers =
      net.getUnconnectedOutLayers();  // get  indices of  output layers, i.e.  layers with unconnected outputs
  vector<cv::String> layersNames = net.getLayerNames();  // get  names of all layers in the network

  names.resize(outLayers.size());
  for (size_t i = 0; i < outLayers.size(); ++i)  // Get the names of the output layers in names
    names[i] = layersNames[outLayers[i] - 1];

  // invoke forward propagation through network
  net.setInput(blob);
  net.forward(netOutput, names);

  // Scan through all bounding boxes and keep only the ones with high confidence
  vector<int> classIds;
  vector<float> confidences;
  vector<cv::Rect> boxes;
  for (size_t i = 0; i < netOutput.size(); ++i) {
    float* data = (float*)netOutput[i].data;
    for (int j = 0; j < netOutput[i].rows; ++j, data += netOutput[i].cols) {
      cv::Mat scores = netOutput[i].row(j).colRange(5, netOutput[i].cols);
      cv::Point classId;
      double confidence;

      // Get the value and location of the maximum score
      cv::minMaxLoc(scores, 0, &confidence, 0, &classId);
      if (confidence > yoloConfig.confidenceThreshold) {
        cv::Rect box;
        int cx, cy;
        cx = (int)(data[0] * frameData.cameraImg.cols);
        cy = (int)(data[1] * frameData.cameraImg.rows);
        box.width = (int)(data[2] * frameData.cameraImg.cols);
        box.height = (int)(data[3] * frameData.cameraImg.rows);
        box.x = cx - box.width / 2;   // left
        box.y = cy - box.height / 2;  // top

        boxes.push_back(box);
        classIds.push_back(classId.x);
        confidences.push_back((float)confidence);
      }
    }
  }

  // perform non-maxima suppression
  vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, yoloConfig.confidenceThreshold, yoloConfig.nmsThreshold, indices);

  // fill in Bounding Box structure with data from YOLO
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    BoundingBox bBox;
    bBox.roi = boxes[*it];
    bBox.classID = classIds[*it];
    bBox.confidence = confidences[*it];
    bBox.boxID = (int)frameData.boundingBoxes.size();  // zero-based unique identifier for this bounding box

    frameData.boundingBoxes.push_back(bBox);
  }

  // show results
  if (visualize) {
    showYoloDetectionOnImage(frameData, yoloConfig);
  }
  std::cout << "#2 : DETECT & CLASSIFY OBJECTS done" << std::endl;
}
