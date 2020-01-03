
#include <algorithm>
#include <iostream>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cameraFusion.h"
#include "dataStructures.h"
#include "utils.h"

using namespace std;

BoundingBox *findBoundingBoxByID(std::vector<BoundingBox> &boundingBoxes, int boxId) {
  for (auto it = boundingBoxes.begin(); it != boundingBoxes.end(); ++it) {
    if (boxId == it->boxID)  // check wether current match partner corresponds to this BB
    {
      return &(*it);
    }
  }
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints,
                         float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT) {
  // loop over all Lidar points and associate them to a 2D bounding box
  cv::Mat X(4, 1, cv::DataType<double>::type);
  cv::Mat Y(3, 1, cv::DataType<double>::type);

  for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1) {
    // assemble vector for matrix-vector-multiplication
    X.at<double>(0, 0) = it1->x;
    X.at<double>(1, 0) = it1->y;
    X.at<double>(2, 0) = it1->z;
    X.at<double>(3, 0) = 1;

    // project Lidar point into camera
    Y = P_rect_xx * R_rect_xx * RT * X;
    cv::Point pt;
    pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);  // pixel coordinates
    pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

    vector<vector<BoundingBox>::iterator>
        enclosingBoxes;  // pointers to all bounding boxes which enclose the current Lidar point
    for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2) {
      // shrink current bounding box slightly to avoid having too many outlier points around the edges
      cv::Rect smallerBox;
      smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
      smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
      smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
      smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

      // check wether point is within current bounding box
      if (smallerBox.contains(pt)) {
        enclosingBoxes.push_back(it2);
      }

    }  // eof loop over all bounding boxes

    // check wether point has been enclosed by one or by multiple boxes
    if (enclosingBoxes.size() == 1) {
      // add Lidar point to bounding box
      enclosingBoxes[0]->lidarPoints.push_back(*it1);
    }

  }  // eof loop over all Lidar points
  std::cout << "#4 : CLUSTER LIDAR POINT CLOUD done" << std::endl;
}

void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait) {
  // create topview image
  cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

  for (auto it1 = boundingBoxes.begin(); it1 != boundingBoxes.end(); ++it1) {
    // create randomized color for current 3D object
    cv::RNG rng(it1->boxID);
    cv::Scalar currColor = cv::Scalar(rng.uniform(0, 150), rng.uniform(0, 150), rng.uniform(0, 150));

    // plot Lidar points into top view image
    int top = 1e8, left = 1e8, bottom = 0.0, right = 0.0;
    float xwmin = 1e8, ywmin = 1e8, ywmax = -1e8;
    for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2) {
      // world coordinates
      float xw = (*it2).x;  // world position in m with x facing forward from sensor
      float yw = (*it2).y;  // world position in m with y facing left from sensor
      xwmin = xwmin < xw ? xwmin : xw;
      ywmin = ywmin < yw ? ywmin : yw;
      ywmax = ywmax > yw ? ywmax : yw;

      // top-view coordinates
      int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
      int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

      // find enclosing rectangle
      top = top < y ? top : y;
      left = left < x ? left : x;
      bottom = bottom > y ? bottom : y;
      right = right > x ? right : x;

      // draw individual point
      cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
    }

    // draw enclosing rectangle
    cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 0), 2);

    // augment object with some key data
    char str1[200], str2[200];
    sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
    putText(topviewImg, str1, cv::Point2f(left - 250, bottom + 50), cv::FONT_ITALIC, 2, currColor);
    sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax - ywmin);
    putText(topviewImg, str2, cv::Point2f(left - 250, bottom + 125), cv::FONT_ITALIC, 2, currColor);
  }

  // plot distance markers
  float lineSpacing = 2.0;  // gap between distance markers
  int nMarkers = floor(worldSize.height / lineSpacing);
  for (size_t i = 0; i < nMarkers; ++i) {
    int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
    cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
  }

  // display image
  string windowName = "3D Objects";
  cv::namedWindow(windowName, cv::WINDOW_NORMAL);
  cv::imshow(windowName, topviewImg);
  cv::resizeWindow(windowName, 200, 200);  // need this for Arch i3 otherwise it is not shown properly

  while ((cv::waitKey() & 0xEFFFFF) != 27) {
    continue;
  }  // wait for keyboard input before continuing
}

void matchBoundingBoxes(DataFrame &currFrame, DataFrame &prevFrame) {
  /* NOTE
   * A DMatch contains a query-index element and a train-index element, where
   *  - the keypoints in the initial (previous) frame are indexed by queryIdx
   *  - the keypoints in the current frame are indexed by trainIdx
   */
  std::map<int, int> bbBestMatches{};
  // Multimap to store associated bounding boxes between current and previous frame based on keypoint matches
  std::multimap<int, int> bboxesMatchIds{};
  for (auto match : currFrame.kptMatches) {
    cv::KeyPoint prevKeypoint = prevFrame.keypoints[match.queryIdx];
    cv::KeyPoint curKeypoint = currFrame.keypoints[match.trainIdx];

    for (auto currBox : currFrame.boundingBoxes) {
      if (currBox.roi.contains(curKeypoint.pt)) {
        for (auto prevBox : prevFrame.boundingBoxes) {
          if (prevBox.roi.contains(prevKeypoint.pt)) {
            bboxesMatchIds.insert({currBox.boxID, prevBox.boxID});
            // std::cout << "Found association between currFrame box: " << currBox.boxID
            //           << " and prevFrame box: " << prevBox.boxID << std::endl;
          }
        }
      }
    }
  }

  // Print multimap content
  // showMultimapContent(bboxesMatchIds);

  bool debugPrintBoxAssociations = false;
  std::vector<std::vector<int>> boxMatchCounts;  // for debugging
  size_t prevFrameNumBoxes = prevFrame.boundingBoxes.size();

  // Iterate over current frame boxes and count for each box the most associated box from the previous frame
  for (auto currBox : currFrame.boundingBoxes) {
    // Get all previous frame boxIDs which are associated to this current frame boxID
    auto prevFrameMatchedBoxes = bboxesMatchIds.equal_range(currBox.boxID);

    // Count the number of times each box of previous frame is matched against this box of the current frame
    std::vector<int> prevFrameBoxMatchesIndex(prevFrameNumBoxes, 0);
    for (std::multimap<int, int>::iterator it = prevFrameMatchedBoxes.first; it != prevFrameMatchedBoxes.second; ++it) {
      prevFrameBoxMatchesIndex[it->second] += 1;
    }
    boxMatchCounts.push_back(prevFrameBoxMatchesIndex);

    // Get the index of the box from the previous frame that is associated the most with this current boxID
    int boxIndex = std::distance(prevFrameBoxMatchesIndex.begin(),
                                 std::max_element(prevFrameBoxMatchesIndex.begin(), prevFrameBoxMatchesIndex.end()));
    if (prevFrameBoxMatchesIndex[boxIndex] != 0) {
      bbBestMatches.insert({prevFrame.boundingBoxes[boxIndex].boxID, currBox.boxID});
      std::cout << " >>> currentBoxID -> previousBoxID: " << currBox.boxID << " => "
                << prevFrame.boundingBoxes[boxIndex].boxID << std::endl;
    } else {
      std::cout << " >>> currentBoxID -> previousBoxID: " << currBox.boxID << " => NONE" << std::endl;
    }

    if (debugPrintBoxAssociations) {
      std::cout << currBox.boxID << " =>";
      for (std::multimap<int, int>::iterator it = prevFrameMatchedBoxes.first; it != prevFrameMatchedBoxes.second;
           ++it) {
        std::cout << ' ' << it->second;
      }
      std::cout << '\n';
      for (size_t i = 0; i < prevFrameBoxMatchesIndex.size(); i++) {
        std::cout << prevFrameBoxMatchesIndex[i] << ", ";
      }
      std::cout << '\n';
    }
  }

  // Print overall mapped bounding box counts
  if (debugPrintBoxAssociations) {
    for (int i = 0; i < boxMatchCounts.size(); i++) {
      for (int j = 0; j < prevFrameNumBoxes; j++) {
        std::cout << boxMatchCounts[i][j] << ", ";
      }
      std::cout << '\n';
    }
  }

  // store matches in current data frame
  currFrame.bbMatches = bbBestMatches;
  std::cout << "#8 : TRACK 3D OBJECT BOUNDING BOXES done" << std::endl;
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev,
                              std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches) {
  // ...
}

