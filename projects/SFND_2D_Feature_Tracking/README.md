# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

This project focuses on tracking features in camera images using feature detection methods from Computer Vision. To determine features in images, the OpenCV framework is used. The library implements a multitude of classical methods and modern methods which are compared in this project.

This mid-term project consists of four main parts:
* loading a dataset of images into a ring buffer of size 2;
* integrating a list of provided feature (keypoint) detection methods:
* integrating a list of provided feature (keypoint) descriptor extraction methods: ; followed by integrating 2 matching methods (Brute Force and FLANN) for matching keypoints between successive images / frames;
* comparing different combinations of keypoint detectors and keypoint descriptors with respect to computation time, number of keypoints matched and accuracy of matching

The assignment and the code is based on the [SFND_2D_Feature_Tracking](https://github.com/udacity/SFND_2D_Feature_Tracking) github repository.

## Dependencies and build system

For this project OpenCV (tag 4.1.2) and the additional modules in OpenCV_contrib (tag 4.1.2) are required.

OpenCV was build from the source metioned above using the following build flags
```
cmake -D CMAKE_BUILD_TYPE=RELEASE \
     -D INSTALL_C_EXAMPLES=OFF \
     -D WITH_GSTREAMER=ON \
     -D WITH_FFMPEG=ON \
     -D OPENCV_ENABLE_NONFREE=ON \
     -D BUILD_EXAMPLES=ON \
     -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules  ..
```
where the `opencv_contrib` repository was cloned in the `../../opencv_contrib` folder relative to the `build` folder.

Docker containers were used to build and run this project's application.

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

For evaluating the goodness of matching, the following two metrics can be used
* Hamming distance or
* L2 norm

Hamming distance is used and recommended for binary descriptors while for SIFT the L2 norm is enforced.

For matching keypoints between two successive frames, the metric above is used with the following two methods:
* Brute Force matching
* FLANN (Fast Library for Approximate Nearest Neighbors) matching

In order to select the best candidate match resulting from the algorithm above, two methods are implemented:
 * either the best candidate (nearest-neighbor (NN) ) is selected
 * k-Nearest-Neighbor(kNN) matching is used for selecting two best matches and the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

All the above options are defined as `enum types` in `dataStructures.h` file and each option is selectable via command line arguments.

## Results

### Keypoint Detection

Each keypoint detection method was ran for each of the 10 images and the results obtained are listed in the tables bellow. Each table provides information on the following
 * the total number of keypoints for the entire image
 * the number of keypoints inside the ROI; these are the number of points that fell inside a bounding rectangle enclosing the preceding vehicle (ROI pre-defined in this assignment)
 * time in milliseconds required for computing the total number of keypoints
 * mean and standard-deviation for the keypoint neighborhood size

 _**Total number of keypoints**_

|#Image|SHI-TOMASI |HARRIS | AKAZE	| BRISK	| FAST	| ORB (default) | SIFT |
|:------:|:------:|:-----:|:------:|:------:|:-----:|:-------------:|:----:|
| 0		 | 1370	  | 492   | 1351	| 2757 	| 1359	| 7822 (500)    | 1438
| 1		 | 1301	  | 502   | 1327	| 2777 	| 1337	| 7684 (500)    | 1371
| 2		 | 1361	  | 516   | 1311	| 2741 	| 1324	| 7612 (500)    | 1380
| 3		 | 1358	  | 524   | 1351	| 2735 	| 1331	| 7783 (500)    | 1335
| 4		 | 1333	  | 523   | 1360	| 2757 	| 1291	| 7797 (500)    | 1305
| 5		 | 1284	  | 511   | 1347	| 2695 	| 1309	| 7724 (500)    | 1370
| 6		 | 1322	  | 505   | 1363	| 2715 	| 1307	| 7612 (500)    | 1396
| 7		 | 1366	  | 510   | 1331	| 2628 	| 1242	| 7534 (500)    | 1382
| 8		 | 1389	  | 529   | 1357	| 2639 	| 1270	| 7622 (500)    | 1463
| 9		 | 1339	  | 520   | 1331	| 2672 	| 1248	| 7635 (500)    | 1422


_**Number of keypoints in ROI**_

|#Image|SHI-TOMASI |HARRIS | AKAZE	| BRISK	| FAST	| ORB (default)	| SIFT |
|:------:|:------:|:-----:|:------:|:------:|:------:|:------------:|:------:|
| 0	     | 127	  | 50    | 162		| 254	| 119	| 860 (91)      | 137
| 1	     | 120	  | 54    | 157		| 274	| 115	| 845 (102)     | 131
| 2	     | 123	  | 53    | 159		| 276	| 123	| 838 (106)     | 121
| 3	     | 120	  | 55    | 154		| 275	| 121	| 860 (113)     | 135
| 4	     | 120	  | 56    | 162		| 293	| 118	| 827 (109)     | 134
| 5	     | 115	  | 58    | 163		| 275	| 125	| 849 (124)     | 139
| 6	     | 114	  | 57    | 173		| 289	| 111	| 835 (129)     | 136
| 7	     | 125	  | 61    | 175		| 268	| 114	| 832 (127)     | 147
| 8	     | 112	  | 60    | 175		| 260	| 117	| 798 (124)     | 156
| 9	     | 113	  | 57    | 175		| 250	| 106	| 792 (125)     | 135

_**Keypoints computation time (miliseconds)**_

|#Image  |SHI-TOMASI | HARRIS | AKAZE	| BRISK	    | FAST	    | ORB	(default)   | SIFT |
|:------:|:------:|:---------:|:------:|:----------:|:---------:|:-----------------:|:------:|
| 0		 | 41.0599  | 15.435  | 103.924	| 263.443	| 0.76094   | 19.8344 (9.75)	| 135.30
| 1		 | 9.76449  | 9.35217 | 43.0131	| 217.964	| 0.623702	| 8.97564 (5.57)	| 77.32
| 2	     | 8.75839  | 7.85033 | 42.5633 | 219.035	| 0.620976	| 9.89893 (5.17) 	| 78.12
| 3		 | 8.77485  | 7.3549  | 42.3676 | 220.761   | 0.607594	| 8.62204 (4.94)	| 76.56
| 4		 | 8.45931  | 7.45507 | 44.3246	| 221.714	| 0.624519	| 8.90695 (5.15		| 76.15
| 5		 | 9.04518  | 7.50166 | 47.5531	| 219.561	| 0.622143	| 9.68342 (5.17		| 76.67
| 6		 | 8.30861  | 8.05752 | 47.4576	| 218.852	| 0.616674	| 8.14016 (5.81		| 77.36
| 7		 | 8.40587  | 10.3918 | 44.6039	| 218.931	| 0.668127  | 9.99981 (4.97)	| 77.38
| 8		 | 8.38145  | 10.3225 | 43.3202	| 215.885	| 0.644785  | 9.62902 (4.95		| 77.78
| 9		 | 11.1336  | 7.37907 | 44.7813	| 218.801   | 0.782431  | 9.14905 (5.20)	| 76.12

_**Keypoints average computation time (miliseconds)**_

|SHI-TOMASI | HARRIS  | AKAZE   | BRISK	|   FAST    | ORB	    | SIFT |
|:---------:|:-------:|:-------:|:-----:|:---------:|:---------:|:----:|
| 12.21    | 9.11    | 50.4    | 223.5	|   0.65    | 10.28		| 82.88

#### Neighborhood Size Distribution

Note that the following methods use a fixed pre-defined neighborhood size:
* SHI-TOMASI - 4 (specified by block size in `detectKeypointsClassic` function)
* HARRIS - 4 (specified by block size in `detectKeypointsClassic` function)
* FAST - 7 (default value)

The table below lists the mean and standard deviation of the neighborhood size computed for the other remaining detector methods. The mean and standard deviations are computed only for the points that fall in the ROI of the preceding vehicle.

_**Keypoints neighboorhood size distribution**_

|#Image  | AKAZE (mean / std)	| BRISK	(mean / std) | ORB (mean / std) | SIFT (mean / std) |
|:------:|:--------------------:|:------------------:|:----------------:|:-----------------:|
| 0		 | 7.2478	/ 3.99091   | 20.99 / 14.44 	 | 50.50 / 22.02 	| 4.59 / 5.97
| 1		 | 6.9670	/ 3.56317   | 21.50 / 14.66		 | 51.02 / 22.78	| 4.62 / 6.21
| 2		 | 6.8794	/ 3.542     | 21.38 / 13.91		 | 50.27 / 22.20	| 4.48 / 6.10
| 3		 | 6.9534   / 3.43268   | 19.98 / 12.65 	 | 50.55 / 22.41 	| 4.24 / 5.29
| 4		 | 7.2520	/ 3.47988   | 22.37 / 14.93		 | 50.74 / 22.65	| 4.21 / 5.53
| 5		 | 7.2014	/ 3.42175   | 22.58 / 15.89		 | 50.22 / 22.27	| 4.11 / 5.60
| 6		 | 7.2475	/ 3.46432   | 21.38 / 14.67		 | 50.63 / 22.52	| 4.88 / 6.55
| 7		 | 7.3361	/ 3.54378   | 21.84 / 15.11 	 | 50.41 / 22.05 	| 4.03 / 5.19
| 8		 | 7.3019	/ 3.53708   | 22.26 / 15.27		 | 50.63 / 22.40	| 5.01 / 6.74
| 9		 | 7.3590	/ 3.65161   | 21.76 / 14.72 	 | 50.04 / 22.13 	| 5.14 / 6.74


### Observations

All the _modern detectors_ have been used _as is_ without changing the default values, with the exception of ORB as highlighted below.

* by default ORB runs with a pre-defined limit on the maximum number of keypoints of `500`. When changing this limit to a large value `30000` and keeping all other parameters to default values, ORB resulted in ~7600 kepyoints per image.
* on average FAST (as the name implies) is the fasted algorithm for detecting features, with an average detection time of 0.65 `msec` over the 10 frames. The second best in terms of speed over total number of keypoints would be either ORB (limited to 500 features) or HARRIS.
* BRISK and SIFT take the longest time to detect features
* the first frame always takes the bulk of the computation time, while the subsequent iterations are much faster.
* SIFT, SHI-TOMASI and HARRIS have very small and precise/dense keypoint neighborhood sizes; AKAZE has a keypoint size comparable to the default size of FAST, whereas BRISK and ORB have the largest keypoint distribution patterns

**Possible improvements**

In the current implementation the keypoint detection algorithm is fed the entire image/frame. However, since the interest is only on the preceding vehicle, considerable speed improvement could be gained by feeding the algorithms only with the cropped image of the preceding vehicle.

### Keypoint Matching

`BRUTE_FORCE` matching was used together with the distance ratio test of 0.8 (ratio of best vs. second-best match), for each descriptor - detector combination. The matching was performed only for those keypoints that were within the ROI.

The results for all the combinations were saved in CSV files in the [results/task_mp8_mp9](./results/task_mp8_mp9) folder. The CSV files follow the naming pattern `results_<DETECTOR-METHOD>_<DESCRIPTOR-METHOD>_summary.csv`

The table below gives the average matches per the 10 images used, for all descriptor-detector combinations (except AKAZE, for the reasons mentioned above as mentioned above).
The values in parenthesis represents the average number of keypoints in the ROI.

_**Average ROI keypoints matched for all 10 images**_

| Descriptor / Detector  |SHI-TOMASI    | HARRIS       | AKAZE	        | BRISK	      	| FAST	        | ORB (default)  | SIFT       |
|:----------------------:|:------------:|:------------:|:--------------:|:-------------:|:-------------:|:--------------:|:----------:|
| BRISK		 	    	 | 86	(119)   |			   |				|				|				|				 |					
| AKAZE		 	    	 | N/A          | N/A     	   |  N/A 			| N/A 			| N/A 			|	N/A 		 |
| BRIEF	     	    	 | 105 	(119)   |			   |				|				|				|				 |					
| FREAK		 	    	 | 86  	(119)   |			   |				|				|				|				 |					
| ORB		 	    	 | 102	(119)   |			   |				|				|				|				 |					
| SIFT		 	    	 | 64	(119)   |			   |				|				|				|				 |					

The table below gives the average descriptor computation time and the average matching computation time per the 10 images used, for all descriptor-detector combinations, in the form `mean descriptor computation time ( mean matching evaluation time)`.


_**Average Keypoint Descriptor/Match evaluation time for all 10 images**_

| Descriptor / Detector  |SHI-TOMASI    | HARRIS     | AKAZE	      | BRISK	    | FAST	        | ORB (default) | SIFT       |
|:----------------------:|:-------------:|:---------:|:--------------:|:-----------:|:-------------:|:-------------:|:----------:|
| BRISK		 	    	 | 1.62  (0.33) | 1.62  (0.33) | 1.62  (0.33) | 1.62  (0.33) | 1.62  (0.33) | 1.62  (0.33)  | 1.62  (0.33)
| AKAZE		 	    	 | N/A 			| N/A     	   | N/A    	  | N/A 		 | N/A 		    | N/A 			| 	N/A
| BRIEF	     	    	 | 1.27  (0.70) | 1.27  (0.70) | 1.27  (0.70) | 1.27  (0.70) | 1.27  (0.70) | 1.27  (0.70)  | 1.27  (0.70)
| FREAK		 	    	 | 31.62 (0.20) | 31.62 (0.20) | 31.62 (0.20) | 31.62 (0.20) | 31.62 (0.20) | 31.62 (0.20)  | 31.62 (0.20)
| ORB		 	    	 | 1.21  (0.82)	| 1.21  (0.82) | 1.21  (0.82) | 1.21  (0.82) | 1.21  (0.82)	| 1.21  (0.82)  | 1.21  (0.82)
| SIFT		 	    	 | 32.50 (0.56)	| 32.50 (0.56) | 32.50 (0.56) | 32.50 (0.56) | 32.50 (0.56)	| 32.50 (0.56)  | 32.50 (0.56)|



### Recommended option

