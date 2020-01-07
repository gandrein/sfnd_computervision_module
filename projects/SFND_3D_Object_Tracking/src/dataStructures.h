
#ifndef DATA_STRUCTURES_H_
#define DATA_STRUCTURES_H_

#include <map>
#include <opencv2/core.hpp>
#include <vector>

enum class DetectorMethod { SHITOMASI = 0, HARRIS, AKAZE, BRISK, FAST, ORB, SIFT };

enum class DescriptorMethod { BRISK = 0, AKAZE, BRIEF, FREAK, ORB, SIFT };

enum class DescriptorMetric { BINARY = 0, HOG };

enum class MatcherMethod { BRUTE_FORCE = 0, FLANN };

enum class NeighborSelectorMethod { NN = 0, kNN };  // NearestNeighbor, kNearestNeighbor

enum class LidarTtcMethod { MEDIAN = 0, MEAN, CLUSTER_EUCLID };

enum class KptMatchesClusterDistanceMethod {THRESHOLD=0, STDEV};

struct NormalDistribution {
  float mean;
  float stddev;
};

struct KptMatchesClusterConf {
  KptMatchesClusterDistanceMethod method;
  union {
    double threshold;
    double numStddev;
  };
};

struct DataSetConfig {
  std::string basePath;
  std::string prefix;
  std::string fileType;
  int startIndex = 0;      // first file index to load (assumes Lidar and camera names have identical naming convention)
  int endIndex = 9;        // last file index to load
  int indexNameWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)
  int indexStepSize = 1;
};

struct YoloConfig {
  std::string filesPath;
  std::string nnClassFile;
  std::string modelWeightsCfg;
  std::string modelWeightsFile;
  float confidenceThreshold;
  float nmsThreshold;
};

struct LidarPoint {   // single lidar point in space
  double x, y, z, r;  // x,y,z in [m], r is point reflectivity
};

struct LidarROI {
  float minZ;
  float maxZ;
  float minX;
  float maxX;
  float minY;
  float maxY;
  float minReflect;
};

struct BoundingBox {  // bounding box around a classified object (contains both 2D and 3D data)

  int boxID;    // unique identifier for this bounding box
  int trackID;  // unique identifier for the track to which this bounding box belongs

  cv::Rect roi;       // 2D region-of-interest in image coordinates
  int classID;        // ID based on class file provided to YOLO framework
  double confidence;  // classification trust

  std::vector<LidarPoint> lidarPoints;  // Lidar 3D points which project into 2D image roi
  std::vector<cv::KeyPoint> keypoints;  // keypoints enclosed by 2D roi
  std::vector<cv::DMatch> kptMatches;   // keypoint matches enclosed by 2D roi
};

struct DataFrame {  // represents the available sensor information at the same time instance

  cv::Mat cameraImg;  // camera image

  std::vector<cv::KeyPoint> keypoints;  // 2D keypoints within camera image
  cv::Mat descriptors;                  // keypoint descriptors
  std::vector<cv::DMatch> kptMatches;   // keypoint matches between previous and current frame
  std::vector<LidarPoint> lidarPoints;

  std::vector<BoundingBox> boundingBoxes;  // ROI around detected objects in 2D image coordinates
  std::map<int, int> bbMatches;            // bounding box matches between previous and current frame
};

#endif /* DATA_STRUCTURES_H_ */
