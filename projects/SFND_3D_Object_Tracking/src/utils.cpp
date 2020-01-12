#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#include "dataStructures.h"
#include "utils.h"

void loadKittiCalibrationData(cv::Mat &P_rect_00, cv::Mat &R_rect_00, cv::Mat &RT) {
	// ANG: THESE values should be parsed/read from the KITTI camera calibration
	// files
	RT.at<double>(0, 0) = 7.533745e-03;
	RT.at<double>(0, 1) = -9.999714e-01;
	RT.at<double>(0, 2) = -6.166020e-04;
	RT.at<double>(0, 3) = -4.069766e-03;
	RT.at<double>(1, 0) = 1.480249e-02;
	RT.at<double>(1, 1) = 7.280733e-04;
	RT.at<double>(1, 2) = -9.998902e-01;
	RT.at<double>(1, 3) = -7.631618e-02;
	RT.at<double>(2, 0) = 9.998621e-01;
	RT.at<double>(2, 1) = 7.523790e-03;
	RT.at<double>(2, 2) = 1.480755e-02;
	RT.at<double>(2, 3) = -2.717806e-01;
	RT.at<double>(3, 0) = 0.0;
	RT.at<double>(3, 1) = 0.0;
	RT.at<double>(3, 2) = 0.0;
	RT.at<double>(3, 3) = 1.0;

	R_rect_00.at<double>(0, 0) = 9.999239e-01;
	R_rect_00.at<double>(0, 1) = 9.837760e-03;
	R_rect_00.at<double>(0, 2) = -7.445048e-03;
	R_rect_00.at<double>(0, 3) = 0.0;
	R_rect_00.at<double>(1, 0) = -9.869795e-03;
	R_rect_00.at<double>(1, 1) = 9.999421e-01;
	R_rect_00.at<double>(1, 2) = -4.278459e-03;
	R_rect_00.at<double>(1, 3) = 0.0;
	R_rect_00.at<double>(2, 0) = 7.402527e-03;
	R_rect_00.at<double>(2, 1) = 4.351614e-03;
	R_rect_00.at<double>(2, 2) = 9.999631e-01;
	R_rect_00.at<double>(2, 3) = 0.0;
	R_rect_00.at<double>(3, 0) = 0;
	R_rect_00.at<double>(3, 1) = 0;
	R_rect_00.at<double>(3, 2) = 0;
	R_rect_00.at<double>(3, 3) = 1;

	P_rect_00.at<double>(0, 0) = 7.215377e+02;
	P_rect_00.at<double>(0, 1) = 0.000000e+00;
	P_rect_00.at<double>(0, 2) = 6.095593e+02;
	P_rect_00.at<double>(0, 3) = 0.000000e+00;
	P_rect_00.at<double>(1, 0) = 0.000000e+00;
	P_rect_00.at<double>(1, 1) = 7.215377e+02;
	P_rect_00.at<double>(1, 2) = 1.728540e+02;
	P_rect_00.at<double>(1, 3) = 0.000000e+00;
	P_rect_00.at<double>(2, 0) = 0.000000e+00;
	P_rect_00.at<double>(2, 1) = 0.000000e+00;
	P_rect_00.at<double>(2, 2) = 1.000000e+00;
	P_rect_00.at<double>(2, 3) = 0.000000e+00;
}

std::string getImageNumberAsString(DataSetConfig &dataInfo, size_t imgIndex) {
	std::ostringstream imgNumber;
	imgNumber << std::setfill('0') << std::setw(dataInfo.indexNameWidth) << dataInfo.startIndex + imgIndex;
	return imgNumber.str();
}

std::string getDatasetImageName(DataSetConfig &dataInfo, size_t imgIndex) {
	std::stringstream ss;
	ss << dataInfo.basePath << dataInfo.prefix << getImageNumberAsString(dataInfo, imgIndex) << dataInfo.fileType;
	return ss.str();
}

void pushToBuffer(std::vector<DataFrame> &buffer, DataFrame &newFrame) {
	int dataBufferSize = 2;  // no. of images which are held in memory (ring
							 // buffer) at the same time
	if (buffer.size() < dataBufferSize) {
		buffer.emplace_back(newFrame);
		std::cout << "Initializing buffer; Buffer size is: " << buffer.size() << std::endl;
	} else {
		std::rotate(buffer.begin(), buffer.begin() + 1, buffer.end());
		buffer.pop_back();
		buffer.emplace_back(newFrame);
		std::cout << "Updating buffer; Buffer size is: " << buffer.size() << std::endl;
	}

	std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl;
}

bool isInsideROI(cv::KeyPoint &kpt, cv::Rect &rectangle) {
	cv::Point2f point = kpt.pt;
	// std::cout << "Point has x: " << keypoint.x << ", y:" << keypoint.y <<
	// std::endl;
	cv::Point2f topLeft(rectangle.x, rectangle.y);
	cv::Point2f topRight(rectangle.x + rectangle.width, rectangle.y);
	cv::Point2f bottomLeft(rectangle.x, rectangle.y + rectangle.height);
	cv::Point2f bottomRight(rectangle.x + rectangle.width, rectangle.y + rectangle.height);

	if (point.x >= topLeft.x && point.x <= topRight.x && point.y >= topLeft.y && point.y <= bottomLeft.y) {
		return true;
	}
	return false;
}

void filterKeypointsNumber(DetectorMethod detector, std::vector<cv::KeyPoint> &keypoints, size_t maxNumber) {
	if (detector == DetectorMethod::SHITOMASI || detector == DetectorMethod::HARRIS) {
		// there is no response info, so keep the first 50 as they are sorted in
		// descending quality order
		keypoints.erase(keypoints.begin() + maxNumber, keypoints.end());
	} else {
		//   DetectorMethod::FAST:
		//   DetectorMethod::BRISK:
		//   DetectorMethod::ORB:
		//   DetectorMethod::AKAZE:
		//   DetectorMethod::SIFT:
		cv::KeyPointsFilter::retainBest(keypoints, maxNumber);
	}
	std::cout << "    >>> NOTE: Keypoints have been limited to " << maxNumber << "!" << std::endl;
}

void showYoloDetectionOnImage(DataFrame &frameData, YoloConfig yoloConfig, std::string labelPostFix) {
	// load class names from file
	std::vector<std::string> classes;
	std::ifstream ifs(yoloConfig.nnClassFile.c_str());
	std::string line;
	while (getline(ifs, line)) classes.push_back(line);

	cv::Mat visImg = frameData.cameraImg.clone();
	for (auto it = frameData.boundingBoxes.begin(); it != frameData.boundingBoxes.end(); ++it) {
		// Draw rectangle displaying the bounding box
		int top, left, width, height;
		top = (*it).roi.y;
		left = (*it).roi.x;
		width = (*it).roi.width;
		height = (*it).roi.height;
		cv::rectangle(visImg, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0), 2);

		std::string label = cv::format("%.2f", (*it).confidence);
		label = std::to_string(it->boxID) + ":" + classes[((*it).classID)] + ":" + label;

		// Display label at the top of the bounding box
		int baseLine;
		cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseLine);
		top = std::max(top, labelSize.height);
		rectangle(visImg, cv::Point(left, top - round(1.5 * labelSize.height)),
				  cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(255, 255, 255),
				  cv::FILLED);
		cv::putText(visImg, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 0), 1);
	}

	std::string windowName = "opencv: Object classification | " + labelPostFix;
	cv::namedWindow(windowName, 1);
	cv::imshow(windowName, visImg);
	while ((cv::waitKey() & 0xEFFFFF) != 27) {
	  continue;
	}  // wait for keyboard input before continuing
}

void showMultimapContent(std::multimap<int, int> &myMap) {
	for (std::multimap<int, int>::iterator itr = myMap.begin(); itr != myMap.end(); ++itr) {
		std::cout << '\t' << itr->first << '\t' << itr->second << '\n';
	}
	std::cout << '\n';
}

NormalDistribution evalNormalDistributionParams(std::vector<double> &vals) {
	NormalDistribution ndist;
	double sum = std::accumulate(vals.begin(), vals.end(), 0.0, [](int sum, const double val) { return sum + val; });
	double mean = sum / vals.size();

	std::vector<double> diff(vals.size());
	std::transform(vals.begin(), vals.end(), diff.begin(), [mean](double &val) { return val - mean; });
	double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
	double stddev = std::sqrt(sq_sum / vals.size());
	ndist.mean = mean;
	ndist.stddev = stddev;
	return ndist;
}

void drawMatches(std::vector<cv::DMatch> &matches, DataFrame &currentFrame, DataFrame &previousFrame,
				 std::string windowTitle) {
	cv::Mat matchImg = currentFrame.cameraImg.clone();
	cv::drawMatches(previousFrame.cameraImg, previousFrame.keypoints, currentFrame.cameraImg, currentFrame.keypoints,
					matches, matchImg, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(),
					cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	std::string windowName = "opencv: " + windowTitle;
	;
	cv::namedWindow(windowName, 7);
	cv::imshow(windowName, matchImg);
	std::cout << "Press key to continue to next image" << std::endl;
	while ((cv::waitKey() & 0xEFFFFF) != 27) {
		continue;
	}  // wait for keyboard input before continuing
}

double computeMedian(std::vector<double> &vals) {
	size_t size = vals.size();

	if (size == 0) {
		return 0;  // Undefined.
	}
	std::sort(vals.begin(), vals.end(), [](double a, double b) { return a < b; });
	if (size % 2 == 0) {
		return (vals[size / 2 - 1] + vals[size / 2]) / 2;
	} else {
		return vals[size / 2];
	}
}

double computeMean(std::vector<double> &vals) {
	double sum = std::accumulate(vals.begin(), vals.end(), 0.0, [](int sum, const double &p) { return sum + p; });
	return sum / vals.size();
}

void visualizeMatchedYoloBoundingBoxes(DataFrame &prev_frame, DataFrame &curr_frame) {
	cv::Mat prv_img = prev_frame.cameraImg.clone();
	cv::Mat cur_img = curr_frame.cameraImg.clone();

	for (std::map<int, int>::iterator itr = curr_frame.bbMatches.begin(); itr != curr_frame.bbMatches.end(); ++itr) {
		// convert bbox match id to string
		int currBoxId = itr->second;
		int prevBoxId = itr->first;
		
		std::string labelCurr = std::to_string(currBoxId) + "->" + std::to_string(prevBoxId);
		std::string labelPrev = std::to_string(prevBoxId);

		// draw previous frame bboxes in green
		BoundingBox prevBox = prev_frame.boundingBoxes[prevBoxId];
		BoundingBox currBox = curr_frame.boundingBoxes[currBoxId];
		cv::rectangle(prv_img, cv::Point(prevBox.roi.x, prevBox.roi.y),
					  cv::Point(prevBox.roi.x + prevBox.roi.width, prevBox.roi.y + prevBox.roi.height), cv::Scalar(0, 255, 0), 2);
		// draw id label on bbox
		cv::putText(prv_img, labelPrev, cv::Point(prevBox.roi.x, prevBox.roi.y), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 255, 0), 2);

		// draw current frame bboxes in red
		cv::rectangle(cur_img, cv::Point(currBox.roi.x, currBox.roi.y),
					  cv::Point(currBox.roi.x + currBox.roi.width, currBox.roi.y + currBox.roi.height), cv::Scalar(0, 0, 255), 2);
		// draw id label on bbox
		cv::putText(cur_img, labelCurr, cv::Point(currBox.roi.x, currBox.roi.y), cv::FONT_ITALIC, 0.75, cv::Scalar(0, 0, 255), 2);
	}

	// vertically concatenate previous and current frames
	cv::Mat concat_img;
	cv::vconcat(cur_img, prv_img,concat_img);
	// draw both frames with corresponding bboxes,ids drawn on both
	std::string windowName = "opencv: Matching Bboxes";
	cv::namedWindow(windowName, 2);
	cv::imshow(windowName, concat_img);
	while ((cv::waitKey() & 0xEFFFFF) != 27) {
		continue;
	}  // wait for keyboard input before continuing
}
