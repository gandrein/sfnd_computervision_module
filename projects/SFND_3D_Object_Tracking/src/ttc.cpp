#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cameraFusion.h"
#include "lidarData.h"
#include "ttc.h"

void evalTTC(LidarTtcMethod lidarTtcMethod, KptMatchesClusterConf kptClusterConfig, DataFrame &currFrame,
			 DataFrame &prevFrame, cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT, double sensorFrameRate,
			 bool visualize) {
	// loop over all bounding-boxes matched pairs
	for (auto it1 = currFrame.bbMatches.begin(); it1 != currFrame.bbMatches.end(); ++it1) {
		// find bounding boxes associates with current match
		BoundingBox *currBB = findBoundingBoxByID(currFrame.boundingBoxes, it1->second);
		BoundingBox *prevBB = findBoundingBoxByID(prevFrame.boundingBoxes, it1->first);

		// compute TTC for current match
		double ttcLidar = 0;
		// only compute TTC if we have Lidar points otherwise it defaults to 0
		if (currBB->lidarPoints.size() > 0 && prevBB->lidarPoints.size() > 0) {
			// Assignment Task-2 -> compute time-to-collision based on Lidar data
			ttcLidar = computeTTCLidar(lidarTtcMethod, prevBB->lidarPoints, currBB->lidarPoints, sensorFrameRate);
			// Assignment Task-3 -> assign enclosed keypoint matches to bounding box
			clusterKptMatchesWithROI(kptClusterConfig, *currBB, currFrame.kptMatches, prevFrame, currFrame, visualize);

			// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
			double ttcCamera =
				computeTTCCamera(prevFrame.keypoints, currFrame.keypoints, currBB->kptMatches, sensorFrameRate);
			if (visualize) {
				cv::Mat visImg = currFrame.cameraImg.clone();
				showLidarImgOverlay(visImg, currBB->lidarPoints, P_rect_00, R_rect_00, RT, &visImg);
				cv::rectangle(visImg, cv::Point(currBB->roi.x, currBB->roi.y),
							  cv::Point(currBB->roi.x + currBB->roi.width, currBB->roi.y + currBB->roi.height),
							  cv::Scalar(0, 255, 0), 2);

				char str[200];
				sprintf(str, "TTC Lidar : %f s, TTC Camera : %f s", ttcLidar, ttcCamera);
				putText(visImg, str, cv::Point2f(80, 50), cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0, 0, 255));

				std::string windowName = "opencv: Final Results-TTC";
				cv::namedWindow(windowName, 4);
				cv::imshow(windowName, visImg);
				std::cout << "Press ESC key to continue to next frame" << std::endl;
				while ((cv::waitKey() & 0xEFFFFF) != 27) {
					continue;
				}  // wait for keyboard input before continuing
			}
		}
	}
}

double computeTTCLidar(LidarTtcMethod ttcMethod, std::vector<LidarPoint> &lidarPointsPrev,
					   std::vector<LidarPoint> &lidarPointsCurr, double lidarFrameRate) {
	switch (ttcMethod) {
		case LidarTtcMethod::MEAN:
			return computeTTCLidarMeanBased(lidarPointsPrev, lidarPointsCurr, lidarFrameRate);
			break;
		case LidarTtcMethod::CLUSTER_EUCLID:
			return computeTTCLidarClusterBased(lidarPointsPrev, lidarPointsCurr, lidarFrameRate);
			break;
		case LidarTtcMethod::MEDIAN:
		default:
			return computeTTCLidarMedianBased(lidarPointsPrev, lidarPointsCurr, lidarFrameRate);
	}
}

double computeTTCLidarMedianBased(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr,
								  double lidarFrameRate) {
	double distance0 = computeMedianLidarX(lidarPointsPrev);
	double distance1 = computeMedianLidarX(lidarPointsCurr);

	// Commpute TTC using the constant-velocity model
	// TTC = d1 * delta_t / (d0 - d1)
	// where d0 and d1 are the distance to the detected front object in
	//      the previous and the current frame respectively
	// and  delta_t is 1/lidarFrameRate in our case.
	double ttc = distance1 / (lidarFrameRate * (distance0 - distance1));
	return ttc;
}

double computeTTCLidarMeanBased(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr,
								double lidarFrameRate) {
	double distance0 = computeMeanLidarX(lidarPointsPrev);
	double distance1 = computeMeanLidarX(lidarPointsCurr);

	// Commpute TTC using the constant-velocity model
	// TTC = d1 * delta_t / (d0 - d1)
	// where d0 and d1 are the distance to the detected front object in
	//      the previous and the current frame respectively
	// and  delta_t is 1/lidarFrameRate in our case.
	double ttc = distance1 / (lidarFrameRate * (distance0 - distance1));
	return ttc;
}

double computeTTCLidarClusterBased(std::vector<LidarPoint> &lidarPointsPrev, std::vector<LidarPoint> &lidarPointsCurr,
								   double frameRate) {
	std::cout << "Cluster Based TTC computation not implemented!" << std::endl;
	return 0.0;
}

// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
double computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
						std::vector<cv::DMatch> kptMatches, double frameRate, cv::Mat *visImg) {
	return 0.0;
}
