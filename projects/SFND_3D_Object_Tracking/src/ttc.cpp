#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cameraFusion.h"
#include "lidarData.h"
#include "ttc.h"

void evalTTC(LidarTtcMethod lidarTtcMethod, KptMatchesClusterConf kptClusterConfig, DataFrame &currFrame,
			 DataFrame &prevFrame, cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT, double sensorFrameRate,
			 bool showKeypointSelected, bool showTTCOnImage) {
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
			clusterKptMatchesWithROI(kptClusterConfig, currFrame.kptMatches, prevFrame, currFrame, *prevBB, *currBB,
									 showKeypointSelected);

			// TASK FP.4 -> compute time-to-collision based on camera (implement -> computeTTCCamera)
			double ttcCamera =
				computeTTCCamera(prevFrame.keypoints, currFrame.keypoints, currBB->kptMatches, sensorFrameRate);
			if (showTTCOnImage) {
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
	std::vector<double> xCompPrev = extractXcomponent(lidarPointsPrev);
	std::vector<double> xCompCurr = extractXcomponent(lidarPointsCurr);

	switch (ttcMethod) {
		case LidarTtcMethod::MEAN:
			return computeTTCLidarMeanBased(xCompPrev, xCompCurr, lidarFrameRate);
			break;
		case LidarTtcMethod::CLUSTER_EUCLID:
			return computeTTCLidarClusterBased(lidarPointsPrev, lidarPointsCurr, lidarFrameRate);
			break;
		case LidarTtcMethod::MEDIAN:
		default:
			return computeTTCLidarMedianBased(xCompPrev, xCompCurr, lidarFrameRate);
	}
}

double computeTTCLidarMedianBased(std::vector<double> &xLidarPrev, std::vector<double> &xLidarCurr,
								  double lidarFrameRate) {
	double distance0 = computeMedian(xLidarPrev);
	double distance1 = computeMedian(xLidarCurr);
	std::cout << "  >>> Lidar TTC: estimated distance to preceeding vehicle: " << distance1 << std::endl;
	std::cout << "  >>> Lidar TTC: estimated distance change (d0 - d1): " << (distance0 - distance1) << std::endl;

	// Commpute TTC using the constant-velocity model
	// TTC = d1 * delta_t / (d0 - d1)
	// where d0 and d1 are the distance to the detected front object in
	//      the previous and the current frame respectively
	// and  delta_t is 1/lidarFrameRate in our case.
	double ttc = distance1 / (lidarFrameRate * (distance0 - distance1));
	return ttc;
}

double computeTTCLidarMeanBased(std::vector<double> &xLidarPrev, std::vector<double> &xLidarCurr,
								double lidarFrameRate) {
	double distance0 = computeMean(xLidarPrev);
	double distance1 = computeMean(xLidarCurr);
	std::cout << "  >>> Lidar TTC: estimated distance to preceeding vehicle: " << distance1 << std::endl;
	std::cout << "  >>> Lidar TTC: estimated distance change (d0 - d1): " << (distance0 - distance1) << std::endl;

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
	/* As explained in Lesson 3 - Engineering a Collision Detection System
	 * given the currnet frame, we compute all distances between all keypoint combinations, let these be h_curr^i
	 * the same is done for the previous frame for the matched keypoints, let these be h_prev_i
	 * Then, given the geometrical properties of the camera, the ratio h_prev^i/h_curr^i is directly proportional
	 * to the inverse ratio of distances (i.e. d_curr^i/d_prev^i) between the matched keypoints
	 */
	// compute distance ratios between all matched keypoints
	std::vector<double> distRatios;  // stores the distance ratios for all keypoints between curr. and prev. frame
	double minDist = 100.0;			 // min. required distance
	for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1) {
		// get current keypoint and its matched partner in the prev. frame
		cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
		cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

		for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2) {
			// get next keypoint and its matched partner in the prev. frame
			cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
			cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

			// compute distances between keypoints in current frame and do the same for the previous frame
			double distHcurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
			double distHprev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

			// compute the distance ratios h_curr_i/h_prev_i and avoid division by zero
			if (distHprev > std::numeric_limits<double>::epsilon() && distHcurr >= minDist) {
				double distRatio = distHcurr / distHprev;
				distRatios.push_back(distRatio);
			}
		}
	}

	// only continue if list of distance ratios is not empty
	double ttc = 0.0;
	if (distRatios.size() == 0) {
		ttc = NAN;
		return ttc;
	}

	// Commpute TTC using the constant-velocity model
	// TTC = d1 * delta_t / (d0 - d1)
	// where d0 and d1 are the distance to the detected front object in
	//      the previous and the current frame respectively
	// and  delta_t is 1/lidarFrameRate in our case.
	// use median of the data for distance computation

	// Compute TTC using median of the data
	// dT = 1 / frameRate;
	double selectedRatio = computeMedian(distRatios);

	std::cout << "  >>> Camera TTC: estimated distance ratio d_prev/d_curr: " << selectedRatio << std::endl;

	ttc = 1 / (frameRate * (selectedRatio - 1));
	return ttc;
}
