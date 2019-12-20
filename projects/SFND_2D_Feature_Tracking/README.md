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
* SIFT, SHI-TOMASI and HARRIS have very small and precise/dense keypoint neighborhood sizes; AKAZE has a keypoint size comparable to the default size of FAST, whereas BRISK and ORB have the largest keypoint distributon patterns

**Possible improvements**

In the current implementation the keypoint detection algorithm is fed the entire image/frame. However, since the interest is only on the preceding vehicle, considerable speed improvement could be gained by feeding the algorithms only with the cropped image of the preceding vehicle.

### Keypoint Matching

`BRUTE_FORCE` matching was used together with the distance ratio test of 0.8 (ratio of best vs. second-best match), for each descriptor - detector combination. Only the keypoints within the ROI were used. The 6 tables below summarize the results.

_**BRISK descriptor with all keypoint detector types**_

|Image   | Detector | KeyPointsPerROI | MatchedPoints | Detector T(ms) | Descriptor T(ms) | Matching T(ms) | TotalTime(ms)|
|:------:|:------------:|:---------------:|:-------------:|:----------------:|:------------------:|:----------------:|:------:|
|0 | SHI_TOMASI | 127 | 0 | 32.4548 | 5.5921 | 0 |  38.0469
|1 | SHI_TOMASI | 120 | 97 | 10.8714 | 1.10104 | 1.33039 |  13.3029
|2 | SHI_TOMASI | 123 | 88 | 9.00774 | 1.12305 | 0.230361 |  10.3612
|3 | SHI_TOMASI | 120 | 80 | 8.70968 | 1.10022 | 0.184013 |  9.99391
|4 | SHI_TOMASI | 120 | 90 | 8.70761 | 1.34158 | 0.189455 |  10.2386
|5 | SHI_TOMASI | 115 | 82 | 8.35853 | 1.06517 | 0.216664 |  9.64037
|6 | SHI_TOMASI | 114 | 79 | 9.0096 | 1.33289 | 0.208765 |  10.5513
|7 | SHI_TOMASI | 125 | 86 | 11.6899 | 1.40771 | 0.240518 |  13.3381
|8 | SHI_TOMASI | 112 | 86 | 10.2495 | 1.04948 | 0.21347 |  11.5125
|9 | SHI_TOMASI | 113 | 83 | 8.6552 | 1.06235 | 0.168131 |  9.88569
|0 | HARRIS | 50 | 0 | 17.7051 | 1.02726 | 0 |  18.7323
|1 | HARRIS | 54 | 43 | 10.2861 | 0.682176 | 0.168275 |  11.1366
|2 | HARRIS | 53 | 39 | 9.37378 | 0.645471 | 0.219681 |  10.2389
|3 | HARRIS | 55 | 45 | 8.72205 | 0.940363 | 0.115736 |  9.77815
|4 | HARRIS | 56 | 44 | 10.6307 | 0.951378 | 0.099395 |  11.6814
|5 | HARRIS | 58 | 37 | 10.4758 | 0.699725 | 0.135615 |  11.3112
|6 | HARRIS | 57 | 41 | 8.32218 | 0.675107 | 0.157128 |  9.15441
|7 | HARRIS | 61 | 44 | 8.46825 | 0.701414 | 0.136047 |  9.30571
|8 | HARRIS | 60 | 52 | 8.16878 | 0.678356 | 0.143523 |  8.99066
|9 | HARRIS | 57 | 48 | 8.60065 | 0.950609 | 0.104828 |  9.65608
|0 | AKAZE | 162 | 0 | 83.4297 | 1.43313 | 0 |  84.8628
|1 | AKAZE | 157 | 134 | 42.4038 | 1.34052 | 0.347893 |  44.0922
|2 | AKAZE | 159 | 124 | 45.6392 | 1.34803 | 0.243248 |  47.2305
|3 | AKAZE | 154 | 129 | 43.2854 | 1.30937 | 0.263352 |  44.8582
|4 | AKAZE | 162 | 128 | 46.3164 | 1.31368 | 0.249985 |  47.8801
|5 | AKAZE | 163 | 130 | 46.9063 | 1.48174 | 0.220014 |  48.6081
|6 | AKAZE | 173 | 132 | 51.1184 | 1.59663 | 0.316633 |  53.0316
|7 | AKAZE | 175 | 142 | 45.3646 | 1.48386 | 0.273241 |  47.1217
|8 | AKAZE | 175 | 144 | 43.0612 | 1.46641 | 0.302884 |  44.8305
|9 | AKAZE | 175 | 141 | 43.134 | 1.45221 | 0.260469 |  44.8466
|0 | BRISK | 254 | 0 | 276.657 | 2.87792 | 0 |  279.535
|1 | BRISK | 274 | 168 | 215.132 | 2.22185 | 0.435252 |  217.79
|2 | BRISK | 276 | 169 | 214.641 | 2.23942 | 0.467097 |  217.347
|3 | BRISK | 275 | 157 | 243.191 | 2.20592 | 0.440173 |  245.837
|4 | BRISK | 293 | 170 | 226.266 | 2.41099 | 0.46823 |  229.145
|5 | BRISK | 275 | 171 | 223.049 | 2.31379 | 0.515236 |  225.878
|6 | BRISK | 289 | 186 | 220.79 | 2.31971 | 0.493802 |  223.604
|7 | BRISK | 268 | 174 | 211.75 | 2.19461 | 0.432206 |  214.377
|8 | BRISK | 260 | 168 | 220.358 | 2.12753 | 0.387382 |  222.873
|9 | BRISK | 250 | 182 | 218.396 | 2.06803 | 0.438722 |  220.903
|0 | FAST | 119 | 0 | 1.18454 | 1.92052 | 0 |  3.10507
|1 | FAST | 115 | 80 | 0.721815 | 1.43115 | 0.242711 |  2.39568
|2 | FAST | 123 | 86 | 0.69416 | 0.989977 | 0.180903 |  1.86504
|3 | FAST | 121 | 83 | 0.721726 | 1.10922 | 0.194268 |  2.02521
|4 | FAST | 118 | 86 | 0.69667 | 1.22775 | 0.172271 |  2.09669
|5 | FAST | 125 | 69 | 0.673807 | 1.36946 | 0.219571 |  2.26284
|6 | FAST | 111 | 89 | 0.732233 | 1.12883 | 0.182701 |  2.04377
|7 | FAST | 114 | 81 | 0.687914 | 1.03923 | 0.198564 |  1.92571
|8 | FAST | 117 | 79 | 0.739207 | 1.06837 | 0.246466 |  2.05404
|9 | FAST | 106 | 83 | 0.714605 | 1.19324 | 0.198673 |  2.10651
|0 | ORB | 91 | 0 | 12.9547 | 1.23308 | 0 |  14.1877
|1 | ORB | 102 | 73 | 6.35497 | 1.00949 | 0.268507 |  7.63297
|2 | ORB | 106 | 74 | 5.8508 | 1.01486 | 0.143647 |  7.0093
|3 | ORB | 113 | 79 | 5.63835 | 1.06726 | 0.188545 |  6.89415
|4 | ORB | 109 | 85 | 5.71162 | 1.0605 | 0.170644 |  6.94277
|5 | ORB | 124 | 79 | 5.93028 | 1.37466 | 0.598769 |  7.90371
|6 | ORB | 129 | 90 | 5.86267 | 1.18963 | 0.168642 |  7.22094
|7 | ORB | 127 | 88 | 5.60655 | 1.25378 | 0.196342 |  7.05668
|8 | ORB | 124 | 86 | 5.74111 | 1.23257 | 0.170901 |  7.14458
|9 | ORB | 125 | 90 | 5.7308 | 1.17291 | 0.194206 |  7.09792
|0 | SIFT | 137 | 0 | 155.544 | 1.26948 | 0 |  156.813
|1 | SIFT | 131 | 63 | 76.483 | 1.20971 | 0.305965 |  77.9987
|2 | SIFT | 121 | 64 | 79.1023 | 1.09319 | 0.198234 |  80.3937
|3 | SIFT | 135 | 60 | 78.8377 | 1.1836 | 0.172762 |  80.1941
|4 | SIFT | 134 | 65 | 62.0404 | 1.18543 | 0.292201 |  63.518
|5 | SIFT | 139 | 59 | 77.3974 | 1.21144 | 0.311517 |  78.9203
|6 | SIFT | 136 | 65 | 63.4525 | 1.19633 | 0.241365 |  64.8902
|7 | SIFT | 147 | 64 | 79.0667 | 1.28352 | 0.326732 |  80.677
|8 | SIFT | 156 | 67 | 81.5591 | 1.32321 | 0.195708 |  83.078
|9 | SIFT | 135 | 79 | 78.2649 | 1.17772 | 0.274986 |  79.7176



enum class DetectorMethod { SHITOMASI = 0, HARRIS, AKAZE, BRISK, FAST, ORB, SIFT };

enum class DescriptorMethod { BRISK = 0, AKAZE, BRIEF, FREAK, ORB, SIFT };

_**BRISK detector matched ROI keypoints**_

|#Image-Sequnce | SHI-TOMASI      | HARRIS      | AKAZE	          | BRISK	        | FAST	        | ORB	(default)   | SIFT |
|:-------------:|:---------------:|:-----------:|:---------------:|:---------------:|:-------------:|:---------------:|:------:|
| 0 / 1  		|  97  (127/120) |  43  (50/54) |  134  (162/157) |  168  (254/274) |  80 (119/115) |  73  (91 /102) |  97  (127/120)
| 1 / 2  		|  88  (120/123) |  39  (54/53) |  124  (157/159) |  169  (274/276) |  86 (115/123) |  74  (102/106) |  88  (120/123)
| 2 / 3  		|  80  (123/120) |  45  (53/55) |  129  (159/154) |  157  (276/275) |  83 (123/121) |  79  (106/113) |  80  (123/120)
| 3 / 4  		|  90  (120/120) |  44  (55/56) |  128  (154/162) |  170  (275/293) |  86 (121/118) |  85  (113/109) |  90  (120/120)
| 4 / 5  		|  82  (120/115) |  37  (56/58) |  130  (162/163) |  171  (293/275) |  69 (118/125) |  79  (109/124) |  82  (120/115)
| 5 / 6  		|  79  (115/114) |  41  (58/57) |  132  (163/173) |  186  (275/289) |  89 (125/111) |  90  (124/129) |  79  (115/114)
| 6 / 7  		|  86  (114/125) |  44  (57/61) |  142  (173/175) |  174  (289/268) |  81 (111/114) |  88  (129/127) |  86  (114/125)
| 7 / 8  		|  86  (125/112) |  52  (61/60) |  144  (175/175) |  168  (268/260) |  79 (114/117) |  86  (127/124) |  86  (125/112)
| 8 / 9  		|  83  (112/113) |  48  (60/57) |  141  (175/175) |  182  (260/250) |  83 (117/106) |  90  (124/125) |  83  (112/113)


### Recommended option

