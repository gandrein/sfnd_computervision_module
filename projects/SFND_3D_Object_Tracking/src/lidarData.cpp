
#include <algorithm>
#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "lidarData.h"

std::vector<double> extractXcomponent(std::vector<LidarPoint> &vals) {
	std::vector<double> xComp;
	xComp.reserve(vals.size());

	for (const auto& pt: vals) {
		xComp.push_back(pt.x);
	}
	return xComp;
}

// remove Lidar points based on min. and max distance in X, Y and Z
void cropLidarPoints(std::vector<LidarPoint> &lidarPoints, LidarROI &roi) {
	std::vector<LidarPoint> newLidarPts;
	for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
		if ((*it).x >= roi.minX && (*it).x <= roi.maxX && (*it).z >= roi.minZ && (*it).z <= roi.maxZ &&
			(*it).z <= 0.0 && abs((*it).y) <= roi.maxY &&
			(*it).r >= roi.minReflect)  // Check if Lidar point is outside of boundaries
		{
			newLidarPts.push_back(*it);
		}
	}

	lidarPoints = newLidarPts;
	std::cout << "#3 : CROP LIDAR POINTS done" << std::endl;
}

// Load Lidar points from a given location and store them in a vector
void loadLidarFromFile(std::vector<LidarPoint> &lidarPoints, std::string filename) {
	// allocate 4 MB buffer (only ~130*4*4 KB are needed)
	unsigned long num = 1000000;
	float *data = (float *)malloc(num * sizeof(float));

	// pointers
	float *px = data + 0;
	float *py = data + 1;
	float *pz = data + 2;
	float *pr = data + 3;

	// load point cloud
	std::FILE *stream;
	stream = fopen(filename.c_str(), "rb");
	num = fread(data, sizeof(float), num, stream) / 4;

	for (int32_t i = 0; i < num; i++) {
		LidarPoint lpt;
		lpt.x = *px;
		lpt.y = *py;
		lpt.z = *pz;
		lpt.r = *pr;
		lidarPoints.push_back(lpt);
		px += 4;
		py += 4;
		pz += 4;
		pr += 4;
	}
	fclose(stream);
	free(data);
}

void showLidarTopview(std::vector<LidarPoint> &lidarPoints, cv::Size worldSize, cv::Size imageSize, bool bWait) {
	// create topview image
	cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

	// plot Lidar points into image
	for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
		float xw = (*it).x;  // world position in m with x facing forward from sensor
		float yw = (*it).y;  // world position in m with y facing left from sensor

		int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
		int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

		cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
	}

	// plot distance markers
	float lineSpacing = 1.0;  // gap between distance markers
	int nMarkers = floor(worldSize.height / lineSpacing);
	for (size_t i = 0; i < nMarkers; ++i) {
		int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
		cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
	}

	// display image
	std::string windowName = "opencv: Top-View Perspective of LiDAR data";
	cv::namedWindow(windowName, 2);
	cv::imshow(windowName, topviewImg);
	if (bWait) {
		cv::waitKey(0);  // wait for key to be pressed
	}
}

void showLidarImgOverlay(cv::Mat &img, std::vector<LidarPoint> &lidarPoints, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx,
						 cv::Mat &RT, cv::Mat *extVisImg) {
	// init image for visualization
	cv::Mat visImg;
	if (extVisImg == nullptr) {
		visImg = img.clone();
	} else {
		visImg = *extVisImg;
	}

	cv::Mat overlay = visImg.clone();

	// find max. x-value
	double maxVal = 0.0;
	for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
		maxVal = maxVal < it->x ? it->x : maxVal;
	}

	cv::Mat X(4, 1, cv::DataType<double>::type);
	cv::Mat Y(3, 1, cv::DataType<double>::type);
	for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
		X.at<double>(0, 0) = it->x;
		X.at<double>(1, 0) = it->y;
		X.at<double>(2, 0) = it->z;
		X.at<double>(3, 0) = 1;

		Y = P_rect_xx * R_rect_xx * RT * X;
		cv::Point pt;
		pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2);
		pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

		float val = it->x;
		int red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
		int green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
		cv::circle(overlay, pt, 5, cv::Scalar(0, green, red), -1);
	}

	float opacity = 0.6;
	cv::addWeighted(overlay, opacity, visImg, 1 - opacity, 0, visImg);

	// return augmented image or wait if no image has been provided
	if (extVisImg == nullptr) {
		std::string windowName = "opencv: LiDAR data on image overlay";
		cv::namedWindow(windowName, 3);
		cv::imshow(windowName, visImg);
		cv::waitKey(0);  // wait for key to be pressed
	} else {
		extVisImg = &visImg;
	}
}