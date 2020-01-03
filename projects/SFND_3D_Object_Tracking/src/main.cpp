#include <algorithm>
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
#include <string>
#include <vector>

#include "cameraFusion.h"
#include "dataStructures.h"
#include "lidarData.h"
#include "matchingFeatures2D.h"
#include "objectDetection2D.h"
#include "tclap/CmdLine.h"
#include "utils.h"

int main(int argc, const char *argv[]) {
  std::string dataPath = "../";
  // Defaults
  bool visualizeYolo = false;
  bool visualizeFusedData = false;
  bool visualizeKeypoints = false;
  bool visualizeKeypointMatch = false;
  bool crossCheckBruteForce = false;
  int limitMaxKeypoints = 0;
  int detectorSelected = static_cast<int>(DetectorMethod::FAST);
  int descriptorSelected = static_cast<int>(DescriptorMethod::BRISK);
  int DescriptorMetricSel = static_cast<int>(DescriptorMetric::BINARY);
  int matcherSelected = static_cast<int>(MatcherMethod::BRUTE_FORCE);
  int nnMatcherSelected = static_cast<int>(NeighborSelectorMethod::kNN);

  // Command line arguments are used for debugging
  // Example
  /*
  ./3D_object_tracking --show-yolo 1 --show-front-object-fused 1  --show-keypoints 1 --show-keypoint-match 1
  --limit-keypts 10
  */
  try {
    TCLAP::CmdLine cmdlineArg("2D features tracking");

    TCLAP::ValueArg<std::string> dir("d", "dir", "Path to directory containing image and lidar files", false, "../",
                                     "string");
    cmdlineArg.add(dir);

    TCLAP::ValueArg<int> detType("", "detector", "Keypoint detector type", 0, detectorSelected, "int");
    cmdlineArg.add(detType);

    TCLAP::ValueArg<int> descType("", "descriptor", "Keypoint descriptor type", 0, descriptorSelected, "int");
    cmdlineArg.add(descType);

    TCLAP::ValueArg<int> matcherType("", "matcher", "Descriptor matcher method", 0, matcherSelected, "int");
    cmdlineArg.add(matcherType);

    TCLAP::ValueArg<int> nnType("", "matcher-selector", "Keypoint selector method for matchers", 0, nnMatcherSelected,
                                "int");
    cmdlineArg.add(nnType);

    TCLAP::ValueArg<int> descrMetric("", "descriptor-metric", "Metric used in matching computation", 0,
                                     DescriptorMetricSel, "int");
    cmdlineArg.add(descrMetric);

    TCLAP::ValueArg<bool> useCrossCheck(
        "", "cross-check",
        "Cross-Check matching between source and destination images. Used only for BRUTE_FORCE matcher.", false,
        crossCheckBruteForce, "bool");
    cmdlineArg.add(useCrossCheck);

    TCLAP::ValueArg<int> maxNumKeypoints("", "limit-keypts",
                                         "Limit the number of keypoints on object (for debugging and visualization)",
                                         false, limitMaxKeypoints, "int");
    cmdlineArg.add(maxNumKeypoints);

    TCLAP::ValueArg<bool> visYOLO("", "show-yolo", "Show results of Yolo detection for each frame", false,
                                  visualizeYolo, "bool");
    cmdlineArg.add(visYOLO);
    TCLAP::ValueArg<bool> visFusion("", "show-front-object-fused",
                                    "Show results of fusing image detection with lidar for front ego-lane object",
                                    false, visualizeFusedData, "bool");
    cmdlineArg.add(visFusion);
    TCLAP::ValueArg<bool> visKeypoints("", "show-keypoints", "Show results of keypoint detection", false,
                                       visualizeKeypoints, "bool");
    cmdlineArg.add(visKeypoints);
    TCLAP::ValueArg<bool> visKeypointMatch("", "show-keypoint-match", "Show keypoint matches between frames", false,
                                           visualizeKeypointMatch, "bool");
    cmdlineArg.add(visKeypointMatch);

    cmdlineArg.parse(argc, argv);

    dataPath = dir.getValue();
    visualizeYolo = visYOLO.getValue();
    visualizeFusedData = visFusion.getValue();
    visualizeKeypoints = visKeypoints.getValue();
    visualizeKeypointMatch = visKeypointMatch.getValue();

    limitMaxKeypoints = maxNumKeypoints.getValue();

    detectorSelected = detType.getValue();
    descriptorSelected = descType.getValue();
    DescriptorMetricSel = descrMetric.getValue();
    matcherSelected = matcherType.getValue();
    nnMatcherSelected = nnType.getValue();
    crossCheckBruteForce = useCrossCheck.getValue();

    // Check AKAZE descriptor/detector combination
    if (descriptorSelected == static_cast<int>(DescriptorMethod::AKAZE) &&
        detectorSelected != static_cast<int>(DetectorMethod::AKAZE)) {
      std::cerr << "AKAZE descriptor type is allowed only with AKAZE/KAZE keypoints. Exiting ..." << std::endl;
      exit(EXIT_FAILURE);
    }

  } catch (TCLAP::ArgException &e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  DetectorMethod detectorMethod = static_cast<DetectorMethod>(detectorSelected);
  DescriptorMethod descriptorMethod = static_cast<DescriptorMethod>(descriptorSelected);
  MatcherMethod matcherMethod = static_cast<MatcherMethod>(matcherSelected);
  DescriptorMetric descriptorMetric = static_cast<DescriptorMetric>(DescriptorMetricSel);
  NeighborSelectorMethod nnSelector = static_cast<NeighborSelectorMethod>(nnMatcherSelected);

  // camera dataset config
  DataSetConfig imgDataInfo;
  imgDataInfo.basePath = dataPath + "images/";
  imgDataInfo.prefix = "KITTI/2011_09_26/image_02/data/000000";  // left camera, color
  imgDataInfo.fileType = ".png";
  // first file index to load (assumes Lidar and camera have identical naming convention)
  imgDataInfo.startIndex = 0;
  imgDataInfo.endIndex = 18;  // last file index to load
  imgDataInfo.indexStepSize = 1;
  imgDataInfo.indexNameWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

  // yolo config
  YoloConfig yoloConfig;
  yoloConfig.filesPath = dataPath + "data/yolo/";
  yoloConfig.nnClassFile = yoloConfig.filesPath + "coco.names";
  yoloConfig.modelWeightsCfg = yoloConfig.filesPath + "yolov3.cfg";
  yoloConfig.modelWeightsFile = yoloConfig.filesPath + "yolov3.weights";

  DataSetConfig lidarDataInfo;
  lidarDataInfo.basePath = dataPath + "images/";
  lidarDataInfo.prefix = "KITTI/2011_09_26/velodyne_points/data/000000";
  lidarDataInfo.fileType = ".bin";

  // calibration data for camera and lidar
  cv::Mat P_rect_00(3, 4, cv::DataType<double>::type);  // 3x4 projection matrix after rectification
  cv::Mat R_rect_00(4, 4, cv::DataType<double>::type);  // 3x3 rectifying rotation to make image planes co-planar
  cv::Mat RT(4, 4, cv::DataType<double>::type);         // rotation matrix and translation vector
  loadKittiCalibrationData(P_rect_00, R_rect_00, RT);

  // Other misc settings
  double sensorFrameRate = 10.0 / imgDataInfo.indexStepSize;  // frames per second for Lidar and camera
  int dataBufferSize = 2;             // no. of images which are held in memory (ring buffer) at the same time
  std::vector<DataFrame> dataBuffer;  // list of data frames which are held in memory at the same time
  bool visualize = false;             // visualize results

  /* MAIN LOOP OVER ALL IMAGES */
  for (size_t imgIndex = 0; imgIndex <= imgDataInfo.endIndex - imgDataInfo.startIndex;
       imgIndex += imgDataInfo.indexStepSize) {
    // Assemble filenames for current index
    std::string imgFullFilename = getDatasetImageName(imgDataInfo, imgIndex);
    // Load image from file
    cv::Mat img = cv::imread(imgFullFilename);

    // Push image into data frame buffer
    DataFrame frame;
    frame.cameraImg = img;
    pushToBuffer(dataBuffer, frame);
    auto currentFrameIter = dataBuffer.end() - 1;

    // Detect and classify objectst with YOLO
    yoloConfig.confidenceThreshold = 0.2;
    yoloConfig.nmsThreshold = 0.4;
    detectObjects(currentFrameIter->cameraImg, currentFrameIter->boundingBoxes, yoloConfig, visualizeYolo);

    // Load 3D Lidar points from file
    std::string lidarFullFilename = lidarDataInfo.basePath + lidarDataInfo.prefix +
                                    getImageNumberAsString(imgDataInfo, imgIndex) + lidarDataInfo.fileType;
    std::vector<LidarPoint> lidarPoints;
    loadLidarFromFile(lidarPoints, lidarFullFilename);

    // Crop lidar points - remove Lidar points based on distance properties
    LidarROI lidarEgoLane;  // focus on ego lane
    lidarEgoLane.minZ = -1.5;
    lidarEgoLane.maxZ = -0.9;
    lidarEgoLane.minX = 2.0;
    lidarEgoLane.maxX = 20.0;
    lidarEgoLane.maxY = 2.0;
    lidarEgoLane.minReflect = 0.1;
    cropLidarPoints(lidarPoints, lidarEgoLane);
    currentFrameIter->lidarPoints = lidarPoints;

    /* Associate Lidar points with camera-based ROI
     *  -> shrink factor - shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of
     * an ROI
     */
    float shrinkFactor = 0.10;
    clusterLidarWithROI(currentFrameIter->boundingBoxes, currentFrameIter->lidarPoints, shrinkFactor, P_rect_00,
                        R_rect_00, RT);

    // Visualize 3D objects
    if (visualizeFusedData) {
      show3DObjects(currentFrameIter->boundingBoxes, cv::Size(4.0, 20.0), cv::Size(2000, 2000), visualizeFusedData);
    }

    // Perform features detection and run feature descriptor algorithms
    runFeatureDetection(*currentFrameIter, detectorMethod, descriptorMethod, limitMaxKeypoints, visualizeKeypoints);

    // Perform Keypoint matching
    if (dataBuffer.size() > 1)  // wait until at least two images have been processed
    {
      auto previousFrameIter = dataBuffer.end() - 2;
      performFeatureMatching(*currentFrameIter, *previousFrameIter, descriptorMethod, descriptorMetric, matcherMethod,
                             nnSelector, crossCheckBruteForce, visualizeKeypointMatch);

      continue;

      /* Track 3D object bounding boxes
       *  associate bounding boxes between current and previous frame using keypoint matches
       */
      matchBoundingBoxes(*currentFrameIter, *previousFrameIter);

      /* COMPUTE TTC ON OBJECT IN FRONT */
      // loop over all BB match pairs
      for (auto it1 = currentFrameIter->bbMatches.begin(); it1 != currentFrameIter->bbMatches.end(); ++it1) {
        // find bounding boxes associates with current match
        BoundingBox *prevBB, *currBB;
        for (auto it2 = currentFrameIter->boundingBoxes.begin(); it2 != currentFrameIter->boundingBoxes.end(); ++it2) {
          if (it1->second == it2->boxID)  // check wether current match partner corresponds to this BB
          {
            currBB = &(*it2);
          }
        }

        for (auto it2 = previousFrameIter->boundingBoxes.begin(); it2 != previousFrameIter->boundingBoxes.end();
             ++it2) {
          if (it1->first == it2->boxID)  // check wether current match partner corresponds to this BB
          {
            prevBB = &(*it2);
          }
        }

        // compute TTC for current match
        if (currBB->lidarPoints.size() > 0 &&
            prevBB->lidarPoints.size() > 0)  // only compute TTC if we have Lidar points
        {
          //// STUDENT ASSIGNMENT
          //// TASK FP.2 -> compute time-to-collision based on Lidar data (implement -> computeTTCLidar)
          double ttcLidar;
          computeTTCLidar(prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate, ttcLidar);
          //// EOF STUDENT ASSIGNMENT

          //// STUDENT ASSIGNMENT
          //// TASK FP.3 -> assign enclosed keypoint matches to bounding box (implement -> clusterKptMatchesWithROI)
          //// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
          double ttcCamera;
          clusterKptMatchesWithROI(*currBB, previousFrameIter->keypoints, currentFrameIter->keypoints,
                                   currentFrameIter->kptMatches);
          computeTTCCamera(previousFrameIter->keypoints, currentFrameIter->keypoints, currBB->kptMatches,
                           sensorFrameRate, ttcCamera);
          //// EOF STUDENT ASSIGNMENT

          visualize = true;
          if (visualize) {
            cv::Mat visImg = currentFrameIter->cameraImg.clone();
            showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
            cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y),
                          cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height),
                          cv::Scalar(0, 255, 0), 2);

            char str[200];
            sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
            putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255));

            std::string windowName = "Final Results : TTC";
            cv::namedWindow(windowName, 4);
            cv::imshow(windowName, visImg);
            std::cout << "Press key to continue to next frame" << std::endl;
            cv::waitKey(0);
          }
          visualize = false;

        }  // eof TTC computation
      }    // eof loop over all BB matches
    }
  }  // eof loop over all images

  return 0;
}
