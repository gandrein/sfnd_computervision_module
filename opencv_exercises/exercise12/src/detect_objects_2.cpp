#include <fstream>
#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "dataStructures.h"

void detectObjects2() {
  // load class names from file
  std::string yoloBasePath = "../data/yolo/";
  std::string yoloClassesFile = yoloBasePath + "coco.names";
  std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
  std::string yoloModelWeights = yoloBasePath + "yolov3.weights";

  std::vector<std::string> classes;
  std::ifstream ifs(yoloClassesFile.c_str());
  std::string line;
  while (getline(ifs, line)) classes.push_back(line);

  std::cout << "Number of classifcation classes: " << classes.size() << std::endl;

  // load neural network
  cv::dnn::Net net = cv::dnn::readNetFromDarknet(yoloModelConfiguration, yoloModelWeights);
  net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
  net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

  // load image from file
  // cv::Mat img = cv::imread("../images/s_thrun.jpg");
  cv::Mat img = cv::imread("../images/0000000000.png");

  // generate 4D blob from input image
  cv::Mat blob;
  double scalefactor = 1 / 255.0;
  cv::Size size = cv::Size(416, 416);
  cv::Scalar mean = cv::Scalar(0, 0, 0);
  bool swapRB = false;
  bool crop = false;
  cv::dnn::blobFromImage(img, blob, scalefactor, size, mean, swapRB, crop);

  // Get names of output layers
  std::vector<cv::String> names;
  // get indices of output layers, i.e. layers with unconnected outputs
  std::vector<int> outLayers = net.getUnconnectedOutLayers();
  names.resize(outLayers.size());
  // get names of all layers in the network
  std::vector<cv::String> layersNames = net.getLayerNames();

  for (size_t i = 0; i < outLayers.size(); ++i)  // Get the names of the output layers in names
  {
    names[i] = layersNames[outLayers[i] - 1];
  }

  // invoke forward propagation through network
  std::vector<cv::Mat> netOutput;
  net.setInput(blob);
  net.forward(netOutput, names);

  // Scan through all bounding boxes and keep only the ones with high confidence
  float confThreshold = 0.20;
  std::vector<int> classIds;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  for (size_t i = 0; i < netOutput.size(); ++i) {
    float* data = (float*)netOutput[i].data;
    for (int j = 0; j < netOutput[i].rows; ++j, data += netOutput[i].cols) {
      cv::Mat scores = netOutput[i].row(j).colRange(5, netOutput[i].cols);
      cv::Point classId;
      double confidence;

      // Get the value and location of the maximum score
      cv::minMaxLoc(scores, 0, &confidence, 0, &classId);
      if (confidence > confThreshold) {
        cv::Rect box;
        int cx, cy;
        cx = (int)(data[0] * img.cols);
        cy = (int)(data[1] * img.rows);
        box.width = (int)(data[2] * img.cols);
        box.height = (int)(data[3] * img.rows);
        box.x = cx - box.width / 2;   // left
        box.y = cy - box.height / 2;  // top

        boxes.push_back(box);
        classIds.push_back(classId.x);
        confidences.push_back((float)confidence);
      }
    }
  }

  // perform non-maxima suppression
  /* ANG notes:
   * confThreshold - will be used for actually considering or discarding a bounding box.
   *    e.g., if set to 1, all boxes are discared, if set to 0, all YOLO resulting boxes are included
   * nmsThreshold - is the threshold on the bounding-box overalp considered for Non-Maximum Suppression
   *    e.g., if set to 1, then boxes are discared only if they fully overlap, if set to 0, then any
   *    any tiny overalp will be considered and the the super-posed boxes will be discarded even for the
   *    tinyest overlap
   */
  float nmsThreshold = 0.4;  // Non-maximum suppression threshold
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
  std::vector<BoundingBox> bBoxes;
  for (auto it = indices.begin(); it != indices.end(); ++it) {
    BoundingBox bBox;
    bBox.roi = boxes[*it];
    bBox.classID = classIds[*it];
    bBox.confidence = confidences[*it];
    bBox.boxID = (int)bBoxes.size();  // zero-based unique identifier for this bounding box

    bBoxes.push_back(bBox);
  }

  // show results
  cv::Mat visImg = img.clone();
  for (auto it = bBoxes.begin(); it != bBoxes.end(); ++it) {
    // Draw rectangle displaying the bounding box
    int top, left, width, height;
    top = (*it).roi.y;
    left = (*it).roi.x;
    width = (*it).roi.width;
    height = (*it).roi.height;
    cv::rectangle(visImg, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0), 2);

    std::string label = cv::format("%.2f", (*it).confidence);
    label = classes[((*it).classID)] + ":" + label;

    // Display label at the top of the bounding box
    int baseLine;
    cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    rectangle(visImg, cv::Point(left, top - round(1.5 * labelSize.height)),
              cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::putText(visImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 0), 1);
  }

  std::string windowName = "Object classification";
  cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
  cv::imshow(windowName, visImg);
  while ((cv::waitKey() & 0xEFFFFF) != 27) {
    continue;
  }  // wait for esc input before continuing
}

int main() { detectObjects2(); }