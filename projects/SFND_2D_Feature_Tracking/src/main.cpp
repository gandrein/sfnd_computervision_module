/* INCLUDES FOR THIS PROJECT */
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

#include "dataStructures.h"
#include "matching2D.hpp"
#include "tclap/CmdLine.h"
#include "utils.h"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {
  std::string dataPath;
  // Defaults
  bool visualizeResult = false;  // visualize results
  bool applyROI = false;
  bool crossCheckBruteForce = false;
  int maxKeypoints = 0;
  int detectorSelected = static_cast<int>(DetectorMethod::SHITOMASI);        // default
  int descriptorSelected = static_cast<int>(DescriptorMethod::BRISK);        // default
  int descriptorEncodingSel = static_cast<int>(DescriptorEncoding::BINARY);  // default
  int matcherSelected = static_cast<int>(MatcherMethod::BRUTE_FORCE);        // default
  int nnMatcherSelected = static_cast<int>(NeighborSelectorMethod::NN);      // default

  // Command line arguments are used for debugging
  try {
    TCLAP::CmdLine cmdlineArg("2D features tracking");

    TCLAP::ValueArg<std::string> dir("d", "dir", "Path to directory containing sequential image files", false,
                                     "../images/", "string");
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

    TCLAP::ValueArg<int> descrEncoding("", "descriptor-encode", "Encoding of descriptor", 0, descriptorEncodingSel,
                                       "int");
    cmdlineArg.add(descrEncoding);

    TCLAP::ValueArg<bool> useROI("", "roi", "Apply an ROI on preceeding vehicle", false, applyROI, "bool");
    cmdlineArg.add(useROI);

    TCLAP::ValueArg<bool> useCrossCheck(
        "", "cross-check",
        "Cross-Check matching between source and destination images. Used only for BRUTE_FORCE matcher.", false,
        crossCheckBruteForce, "bool");
    cmdlineArg.add(useCrossCheck);

    TCLAP::ValueArg<int> maxNumKeypoints("", "max-keypts",
                                         "Limit the number of keypoints (for debugging) to provided value", false,
                                         maxKeypoints, "int");
    cmdlineArg.add(maxNumKeypoints);

    TCLAP::ValueArg<bool> visualize("", "visualize", "Show results in OpenCV window", false, visualizeResult, "bool");
    cmdlineArg.add(visualize);

    cmdlineArg.parse(argc, argv);

    dataPath = dir.getValue();
    visualizeResult = visualize.getValue();

    applyROI = useROI.getValue();
    maxKeypoints = maxNumKeypoints.getValue();

    detectorSelected = detType.getValue();
    descriptorSelected = descType.getValue();
    descriptorEncodingSel = descrEncoding.getValue();
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
  // camera dataset config
  DataSetConfig dataInfo;
  dataInfo.imgBasePath = dataPath;
  dataInfo.imgPrefix = "KITTI/2011_09_26/image_00/data/000000";  // left camera, color
  dataInfo.imgFileType = ".png";
  // first file index to load (assumes Lidar and camera have identical naming convention)
  dataInfo.imgStartIndex = 0;
  dataInfo.imgEndIndex = 9;   // last file index to load
  dataInfo.imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

  DetectorMethod detectorMethod = static_cast<DetectorMethod>(detectorSelected);
  DescriptorMethod descriptorMethod = static_cast<DescriptorMethod>(descriptorSelected);
  // output files
  std::ofstream summaryCsvFile;
  initSummaryFile(summaryCsvFile, detectorMethod, descriptorMethod);
  std::ofstream distributionCsvFile;
  initDistributionDataFile(distributionCsvFile, detectorMethod);

  /* MAIN LOOP OVER ALL IMAGES */
  std::vector<DataFrame> dataBuffer;  // list of data frames which are held in memory at the same time

  for (size_t imgIndex = 0; imgIndex <= dataInfo.imgEndIndex - dataInfo.imgStartIndex; imgIndex++) {
    // assemble filenames for current index
    std::string imgFullFilename = getDatasetImageName(dataInfo, imgIndex);

    DetectionResult detectionInfoStats;

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
    double timeKptDetection = detectKeypoints(detectorMethod, keypoints, imgGray, visualizeResult);
    detectionInfoStats.imageIndex = imgIndex;
    detectionInfoStats.detector = detectorMethod;
    detectionInfoStats.numKeypointsFrame = keypoints.size();
    detectionInfoStats.detectionComputeTimeSec = timeKptDetection;

    /*
     * TASK MP.3 -> Apply box ROI (and reduce number of keypoints for debugging)
     */
    if (applyROI) {
      //  -> only keep keypoints on the preceding vehicle
      cv::Rect vehicleRect(535, 180, 180, 150);
      filterKeypointsROI(vehicleRect, keypoints);
      detectionInfoStats.numKeypointsROI = keypoints.size();
      detectionInfoStats.roiApplyed = true;

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
    if (maxKeypoints != 0) {
      filterKeypointsNumber(detectorMethod, keypoints, maxKeypoints);
    }
    // push keypoints and descriptor for current frame to end of data buffer
    (dataBuffer.end() - 1)->keypoints = keypoints;

    // print distribution info
    Distribution distInfo = evalDistribution(keypoints);
    std::cout << "Method: " << DetectorMethodToString(detectorMethod) << ", time: " << timeKptDetection
              << ", numKpts: " << detectionInfoStats.numKeypointsFrame << ", numKptsROI "
              << detectionInfoStats.numKeypointsROI << ", meanKptSize" << distInfo.mean << ", stddevKptSize "
              << distInfo.stdev << std::endl;

    /*
     * TASK MP.4 -> EXTRACT KEYPOINT DESCRIPTORS
     * -> add the following descriptors in file matching2D.cpp and enable string-based selection based on
     * -> BRIEF, ORB, FREAK, AKAZE, SIFT
     */

    cv::Mat descriptors;
    double timeDescriptor = descKeypoints(descriptorMethod, (dataBuffer.end() - 1)->keypoints,
                                          (dataBuffer.end() - 1)->cameraImg, descriptors);
    detectionInfoStats.descriptor = descriptorMethod;
    detectionInfoStats.descriptorComputeTimeSec = timeDescriptor;
    // push descriptors for current frame to end of data buffer
    (dataBuffer.end() - 1)->descriptors = descriptors;

    /* MATCH KEYPOINT DESCRIPTORS */
    if (dataBuffer.size() > 1) {
      // wait until at least two images have been processed
      std::vector<cv::DMatch> matches;
      MatcherMethod matcherMethod = static_cast<MatcherMethod>(matcherSelected);
      DescriptorEncoding descriptorEncoding = static_cast<DescriptorEncoding>(descriptorEncodingSel);
      NeighborSelectorMethod nnSelector = static_cast<NeighborSelectorMethod>(nnMatcherSelected);

      double timeMatcher =
          matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                           (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors, matches,
                           descriptorMethod, descriptorEncoding, matcherMethod, nnSelector, crossCheckBruteForce);

      detectionInfoStats.numMatches = matches.size();
      detectionInfoStats.matchesComputeTimeSec = timeMatcher;

      // store matches in current data frame
      (dataBuffer.end() - 1)->kptMatches = matches;

      // visualize matches between current and previous image
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
    }
    appendToSummaryFile(summaryCsvFile, detectionInfoStats);
  }  // eof loop over all images

  summaryCsvFile.close();
  distributionCsvFile.close();
  return 0;
}
