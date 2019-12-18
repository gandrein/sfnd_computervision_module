# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

This project focuses on tracking features in camera images using feature detection methods from Computer Vision. To achieve the OpenCV framework. The library implements a multitude of classical methods and modern methods which are compared in this project.

This mid-term project consists of four main parts:
* loading a dataset of images into a ring buffer of size 2;
* integrating a list of provided feature (keypoint) detection methods:
* integrating a list of provided feature (keypoint) descriptor extraction methods: ; followed by integrating 2 matching methods (Brute Force and FLANN) for matching keypoints between successive images / frames;
* comparing different combinations of keypoint detectors with keypoint descriptors with respect to computation time, number of keypoints matched and accuracy of matching

The assignment and the code is based on the [SFND_2D_Feature_Tracking](https://github.com/udacity/SFND_2D_Feature_Tracking) github repository.

## Overview

### Keypoint Detection

From the class of _classical methods_, which compute gradients of images, HARRIS and SHI-TOMASI methods have been integrated from OpenCV.

From the class of _modern methods_, the following detection methods have been integrated:
* AKAZE
* BRISK
* FAST
* ORB
* SIFT

### Keypoint Descriptors

For describing the neighborhood of keypoints, the following descriptor methods have been integrated from OpenCV:
* BRISK
* AKAZE
* BRIEF
* FREAK
* ORB
* SIFT

Note that, as per [OpenCV documentation](https://docs.opencv.org/master/d8/d30/classcv_1_1AKAZE.html) AKAZE descriptors can only be used with KAZE or AKAZE keypoints.

### Keypoint Matching

Available keypoint matching


## Results

### Keypoint Detection

The statistics for each keypoint detection method for each of the 10 images are listed in the tables bellow. Each table provides information on the following
 * the total number of keypoints for the entire image
 * the number of keypoints inside the ROI; these are the number of points that fell inside a bounding rectangle enclosing the preceding vehicle (ROI pre-defined in this assignment)
 * time in milliseconds required for computing the total number of keypoints

 _Total number of keypoints_

|#Image|SHITOMASI |	HARRIS | AKAZE	| BRISK	| FAST	| ORB	     | SIFT |
|:------:|:------:|:-----:|:------:|:------:|:------:|:------:|:------:|
| 0		 | 1370	  | 492   | 1351	| 2757 	| 1359	| 7822 (500) | 1438
| 1		 | 1301	  | 502   | 1327	| 2777 	| 1337	| 7684 (500) | 1371
| 2		 | 1361	  | 516   | 1311	| 2741 	| 1324	| 7612 (500) | 1380
| 3		 | 1358	  | 524   | 1351	| 2735 	| 1331	| 7783 (500) | 1335
| 4		 | 1333	  | 523   | 1360	| 2757 	| 1291	| 7797 (500) | 1305
| 5		 | 1284	  | 511   | 1347	| 2695 	| 1309	| 7724 (500) | 1370
| 6		 | 1322	  | 505   | 1363	| 2715 	| 1307	| 7612 (500) | 1396
| 7		 | 1366	  | 510   | 1331	| 2628 	| 1242	| 7534 (500) | 1382
| 8		 | 1389	  | 529   | 1357	| 2639 	| 1270	| 7622 (500) | 1463
| 9		 | 1339	  | 520   | 1331	| 2672 	| 1248	| 7635 (500) | 1422


_Number of keypoints in ROI_

|#Image|SHITOMASI |	HARRIS | AKAZE	| BRISK	| FAST	| ORB	    | SIFT |
|:------:|:------:|:-----:|:------:|:------:|:------:|:--------:|:------:|
| 0	     | 127	  | 50    | 162		| 254	| 119	| 860 (91)  | 137
| 1	     | 120	  | 54    | 157		| 274	| 115	| 845 (102) | 131
| 2	     | 123	  | 53    | 159		| 276	| 123	| 838 (106) | 121
| 3	     | 120	  | 55    | 154		| 275	| 121	| 860 (113) | 135
| 4	     | 120	  | 56    | 162		| 293	| 118	| 827 (109) | 134
| 5	     | 115	  | 58    | 163		| 275	| 125	| 849 (124) | 139
| 6	     | 114	  | 57    | 173		| 289	| 111	| 835 (129) | 136
| 7	     | 125	  | 61    | 175		| 268	| 114	| 832 (127) | 147
| 8	     | 112	  | 60    | 175		| 260	| 117	| 798 (124) | 156
| 9	     | 113	  | 57    | 175		| 250	| 106	| 792 (125) | 135

_Keypoints computation time (miliseconds)_

|#Image  |SHITOMASI |  HARRIS | AKAZE	| BRISK	    | FAST	    | ORB	| SIFT |
|:------:|:------:|:---------:|:------:|:------:|:------:|:------:|:------:|
| 0		 | 90.9347  | 83.5431 | 180.126	| 330.927	| 6.13433	| 82.9398 (85.6828)	| 202.972
| 1		 | 9.9286   | 10.2357 | 59.1923	| 215.134	| 0.715118	| 27.0924 (6.288)	| 76.689
| 2		 | 8.02743  | 7.96588 | 61.4542	| 213.953	| 0.678512	| 13.3493 (5.52124)	| 73.8601
| 3		 | 7.88374  | 7.70825 | 80.705	| 234.5	    | 0.677328	| 26.7999 (5.14213)	| 98.9269
| 4		 | 7.68976  | 7.37359 | 71.6181	| 219.737	| 0.670054	| 13.4633 (5.0168)	| 92.6481
| 5		 | 26.1601  | 24.9949 | 78.1041	| 214.887	| 0.644493	| 13.5916 (10.1905)	| 121.939
| 6		 | 14.7836  | 25.1343 | 73.2334	| 212.814	| 0.611675	| 29.9228 (20.726)	| 101.614
| 7		 | 13.5707  | 24.4064 | 69.3104	| 263.241	| 1.45156	| 28.5461 (10.3178)	| 105.964
| 8		 | 14.8852  | 14.1907 | 82.6023	| 236.763	| 1.37491	| 13.1088 (9.9636)	| 101.923
| 9		 | 30.2376  | 17.9685 | 80.2895	| 255.53	| 1.45609	| 27.2193 (22.3574)	| 102.674

!<-- Mean times     22.4 | 22.35   | 83.64   | 239.75 |    1.44      |27.603 (18.121) | 107.92 -->

