/* INCLUDES FOR THIS PROJECT */
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <sstream>
#include <vector>

#include "dataStructures.h"
#include "matching2D.hpp"

#include "tclap/CmdLine.h"

using namespace std;

struct DataSetConfig {
  std::string imgBasePath;
  std::string imgPrefix;
  std::string imgFileType;
  int imgStartIndex = 0;  // first file index to load (assumes Lidar and camera names have identical naming convention)
  int imgEndIndex = 9;    // last file index to load
  int imgFillWidth = 4;   // no. of digits which make up the file index (e.g. img-0001.png)
};

void loadNextImage(std::string imgFullFilename, cv::Mat &imgGray) {
  cv::Mat img;
  img = cv::imread(imgFullFilename);
  cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
}

std::string getDatasetImageName(DataSetConfig &dataInfo, size_t imgIndex) {
  ostringstream imgNumber;
  imgNumber << setfill('0') << setw(dataInfo.imgFillWidth) << dataInfo.imgStartIndex + imgIndex;
  stringstream ss;
  ss << dataInfo.imgBasePath << dataInfo.imgPrefix << imgNumber.str() << dataInfo.imgFileType;
  return ss.str();
}

void pushToBuffer(std::vector<DataFrame> &buffer, DataFrame &newFrame) {
  int dataBufferSize = 2;  // no. of images which are held in memory (ring buffer) at the same time
  if (buffer.size() < dataBufferSize) {
    buffer.emplace_back(newFrame);
    std::cout << "Initializing buffer; Buffer size is: " << buffer.size() << std::endl;
  } else {
    std::rotate(buffer.begin(), buffer.begin() + 1, buffer.end());
    buffer.pop_back();
    buffer.emplace_back(newFrame);
    std::cout << "Updating buffer; Buffer size is: " << buffer.size() << std::endl;
  }

  cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;
}

bool isInsideROI(cv::KeyPoint &kpt, cv::Rect &rectangle) {
  cv::Point2f point = kpt.pt;
  // std::cout << "Point has x: " << keypoint.x << ", y:" << keypoint.y << std::endl;
  cv::Point2f topLeft(rectangle.x, rectangle.y);
  cv::Point2f topRight(rectangle.x + rectangle.width, rectangle.y);
  cv::Point2f bottomLeft(rectangle.x, rectangle.y + rectangle.height);
  cv::Point2f bottomRight(rectangle.x + rectangle.width, rectangle.y + rectangle.height);

  if (point.x >= topLeft.x && point.x <= topRight.x && point.y >= topLeft.y && point.y <= bottomLeft.y) {
    return true;
  }
  return false;
}

void filterKeypointsROI(cv::Rect &rectangle, std::vector<cv::KeyPoint> &keypoints) {
  auto newEnd = std::remove_if(keypoints.begin(), keypoints.end(),
                               [&rectangle](cv::KeyPoint kpt) { return !isInsideROI(kpt, rectangle); });
  keypoints.erase(newEnd, keypoints.end());
}

void filterKeypointsNumber(DetectorType detector, std::vector<cv::KeyPoint> &keypoints, size_t maxNumber) {
  switch (detector) {
    case DetectorType::SHITOMASI:
    case DetectorType::HARRIS:
      // there is no response info, so keep the first 50 as they are sorted in descending quality order
      keypoints.erase(keypoints.begin() + maxNumber, keypoints.end());
      break;
    case DetectorType::FAST:
    case DetectorType::BRISK:
    case DetectorType::ORB:
    case DetectorType::AKAZE:
    case DetectorType::SIFT:
      cv::KeyPointsFilter::retainBest(keypoints, maxNumber);
      break;
    default:
      std::cout << "Unknown detector method!" << std::endl;
      break;
  }
  cout << " NOTE: Keypoints have been limited!" << endl;
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {
  std::string dataPath;
  // Defaults
  bool visualizeResult = false;  // visualize results
  bool applyROI = false;
  bool limitMaxKeypoints = false;
  int detectorType = static_cast<int>(DetectorType::SHITOMASI);  // default SHITOMASI

  // Command line arguments are used for debugging
  try {
    TCLAP::CmdLine cmdlineArg("2D features tracking");

    TCLAP::ValueArg<std::string> dir("d", "dir", "Path to directory containing sequential image files", false,
                                     "../images/", "string");
    cmdlineArg.add(dir);

    TCLAP::ValueArg<uint> dType("", "dtype", "Keypoint detector type", 0, detectorType, "int");
    cmdlineArg.add(dType);

    TCLAP::ValueArg<bool> useROI("", "roi", "Apply an ROI on preceeding vehicle", false, applyROI, "bool");
    cmdlineArg.add(useROI);

    TCLAP::ValueArg<bool> maxNumKeypoints("", "max-keypts", "Limit the number of keypoints (for debugging)", false,
                                          limitMaxKeypoints, "bool");
    cmdlineArg.add(maxNumKeypoints);

    TCLAP::ValueArg<bool> visualize("", "visualize", "Show results in OpenCV window", false, visualizeResult, "bool");
    cmdlineArg.add(visualize);

    cmdlineArg.parse(argc, argv);

    dataPath = dir.getValue();
    visualizeResult = visualize.getValue();
    detectorType = dType.getValue();
    applyROI = useROI.getValue();
    limitMaxKeypoints = maxNumKeypoints.getValue();

  } catch (TCLAP::ArgException &e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }
  // camera dataset config
  DataSetConfig dataInfo;
  dataInfo.imgBasePath = dataPath;
  dataInfo.imgPrefix = "KITTI/2011_09_26/image_00/data/000000";  // left camera, color
  dataInfo.imgFileType = ".png";
  // first file index to load (assumes Lidar and camera have identical naming convention)
  dataInfo.imgStartIndex = 0;
  dataInfo.imgEndIndex = 9;   // last file index to load
  dataInfo.imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

  std::vector<DataFrame> dataBuffer;  // list of data frames which are held in memory at the same time

  /* MAIN LOOP OVER ALL IMAGES */
  for (size_t imgIndex = 0; imgIndex <= dataInfo.imgEndIndex - dataInfo.imgStartIndex; imgIndex++) {
    // assemble filenames for current index
    std::string imgFullFilename = getDatasetImageName(dataInfo, imgIndex);

    // load image from file and convert to grayscale
    cv::Mat imgGray;
    loadNextImage(imgFullFilename, imgGray);

    /*
     * TASK MP.1 -> implement ring buffer
     */
    // push image into data frame buffer
    DataFrame frame;
    frame.cameraImg = imgGray;
    pushToBuffer(dataBuffer, frame);

    /*
     * TASK MP.2 -> DETECT IMAGE KEYPOINTS
     *  -> add the following keypoint detectors in file matching2D.cpp:
     *  HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
     */
    // extract 2D keypoints from current image
    std::vector<cv::KeyPoint> keypoints;  // create empty feature list for current image
    DetectorType detector = static_cast<DetectorType>(detectorType);
    detectKeypoints(detector, keypoints, imgGray, visualizeResult);

    /*
     * TASK MP.3 -> Apply box ROI (and reduce number of keypoints for debugging)
     */
    if (applyROI) {
      //  -> only keep keypoints on the preceding vehicle
      cv::Rect vehicleRect(535, 180, 180, 150);
      filterKeypointsROI(vehicleRect, keypoints);

      // Debug section ....
      /*
      for (auto it = keypoints.begin(); it != keypoints.end(); it++) {
        cv::Point2f pt = (*it).pt;
        if (isInsideROI(pt, vehicleRect)) {
          imgGray.at<unsigned char>(pt.y, pt.x) = 255;
        }
      }
      cv::namedWindow("testROI", 1);  // create window
      cv::imshow("testROI", imgGray);
      while ((cv::waitKey() & 0xEFFFFF) != 27) {
        continue;
      }  // wait for keyboard input before continuing
      */
    }

    // optional : limit number of keypoints (helpful for debugging and learning)
    if (limitMaxKeypoints) {
      int maxKeypoints = 50;
      filterKeypointsNumber(detector, keypoints, maxKeypoints);
    }

    // push keypoints and descriptor for current frame to end of data buffer
    (dataBuffer.end() - 1)->keypoints = keypoints;
    cout << "#2 : DETECT KEYPOINTS done" << endl;

    /*
     * TASK MP.4 -> EXTRACT KEYPOINT DESCRIPTORS
     * -> add the following descriptors in file matching2D.cpp and enable string-based selection based on
     * -> BRIEF, ORB, FREAK, AKAZE, SIFT
     */

    cv::Mat descriptors;
    string descriptorType = "BRISK";  // BRIEF, ORB, FREAK, AKAZE, SIFT
    descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
    //// EOF STUDENT ASSIGNMENT

    // push descriptors for current frame to end of data buffer
    (dataBuffer.end() - 1)->descriptors = descriptors;

    cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

    if (dataBuffer.size() > 1)  // wait until at least two images have been processed
    {
      /* MATCH KEYPOINT DESCRIPTORS */

      vector<cv::DMatch> matches;
      string matcherType = "MAT_BF";         // MAT_BF, MAT_FLANN
      string descriptorType = "DES_BINARY";  // DES_BINARY, DES_HOG
      string selectorType = "SEL_NN";        // SEL_NN, SEL_KNN

      //// STUDENT ASSIGNMENT
      //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
      //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file
      /// matching2D.cpp

      matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                       (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors, matches,
                       descriptorType, matcherType, selectorType);

      //// EOF STUDENT ASSIGNMENT

      // store matches in current data frame
      (dataBuffer.end() - 1)->kptMatches = matches;

      cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

      // visualize matches between current and previous image
      visualizeResult = true;
      if (visualizeResult) {
        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints, matches, matchImg,
                        cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(),
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

        string windowName = "Matching keypoints between two camera images";
        cv::namedWindow(windowName, 7);
        cv::imshow(windowName, matchImg);
        cout << "Press key to continue to next image" << endl;
        // cv::waitKey(0);  // wait for key to be pressed
        while ((cv::waitKey() & 0xEFFFFF) != 27) {
          continue;
        }  // wait for keyboard input before continuing
      }
      visualizeResult = false;
    }

  }  // eof loop over all images

  return 0;
}