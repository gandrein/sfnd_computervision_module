# SFND 3D Object Tracking

<img src="docs/readme_images/ttc_result_example.png" width="820" height="248" />

This project focuses on computing Time-To-Collision (TTC) estimates for a tracked object preceding our vehicle. The goal is to use lidar and camera data to detect and track preceding vehicles. This is achieved by using feature detection methods to detect, describe and track keypoints between successive images, the YOLO deep-learning framework to detect vehicles and associate them with the features. Finally lidar points in 3D space are used to build a fused tracking system.

This final project consists of four main parts:
1. Matching YOLO detected objects between successive frames using image keypoint correspondences in order to track 3D objects over time.
2. Making use of provided code that fuses image data to lidar points to compute a lidar only TTC estimate for colliding with the vehicle in front, in the ego-lane. The (simplest) velocity model is used for TTC computation.
3. Similarly, the computation of a camera only TTC estimate for colliding with the vehicle in front.
4. Evaluating performance, identify the most suitable detector/descriptor combination for TTC estimation and also identifying problems that can lead to faulty measurements by the camera or lidar sensors.

The assignment and the code is based on the [SFND_3D_Feature_Tracking](https://github.com/udacity/SFND_3D_Feature_Tracking) github repository.

In principle this project, implements the missing components of the schematic below. The other components being implemented during the `SFND_3D_Feature_Tracking` project or course exercises.

<img src="docs/readme_images/course_code_structure.png" width="779" height="414" />

## Dependencies and build system

OS dependencies
```
  cmake >= 3.10
  make >= 4.1
  git-lfs
  opencv >= 4.1.2
  gcc >= 7.40
```

For this project OpenCV (tag 4.1.2) and the additional modules in OpenCV_contrib (tag 4.1.2) are required.

OpenCV was build from the source mentioned above using the following build flags
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

Also the ORB descriptor does not work with SIFT feature detector. Only reference to this behavior is this OpenCV [forum answer](https://answers.opencv.org/question/5542/sift-feature-descriptor-doesnt-work-with-orb-keypoinys/)

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

### TTC Model

In this project, the goal is to compute the Time-To-Collision (TTC) with the preceding vehicle in the ego lane.

TTC is computed using the distance to the preceding vehicle recorded in the previous time instance (either lidar or camera frame) and the current instance, as depicted in the image below. Where **index=0** represents current time instance (current frame) and **index=1** represents previous time instance (previous frame). Where, `v_0` represents the current measured velocity of the preceding vehicle. **NOTE** due to a course lecture note error this is denoted as `v_0` but should rather be `v_1`.

<img src="docs/readme_images/ttc_overview.png" width="600" height="250"  />

As introduced in _Lesson 3 - Engineering a Collision Detection System_, the simplest two methods to evaluate the TTC are using a constant velocity model or a constant acceleration model, governed by the equations depicted below. For this project, the `constant velocity model` is used.

<img src="docs/readme_images/ttc_models.png" width="400" height="200"  />

### TTC Computation Lidar

A Lidar sensor provides direct distance measurements of the objects in the environment. Hence the distance `d_0` and `d_1` from the schematic below should be directly available when the sensor data is stored for each individual time instance.

<img src="docs/readme_images/ttc_lidar_view.png" width="600" height="200"  />

Using the constant velocity model one can calculate the TTC using lidar measurements by using the `equation (3)` derived below.

<img src="docs/readme_images/ttc_lidar_formula.png" width="400" height="200" />

However, estimating the distance of the preceding vehicle using lidar data can be problematic due to nosy measurements. As such, one cannot estimate the distance by just taking the closest lidar point detected. Various filter and/or models have to be implemented in order to obtain a robust measurement.

In this project, a few simple filters are applied to the lidar data before estimating the distance to the preceding vehicle:
* ground plane points are filtered based on `z` coordinate value
* adjacent lane vehicle points are filtered based on assumed ego lane width
* preceding vehicle points are filtered by matching them against a YOLO detected bounding box
* extra points are removed by applying a `shrinkage factor` to keep only lidar points that fall within a smaller version of the original YOLO bounding box

Finally, the distance to the preceding vehicle is estimated taking the median of the lidar points sorted based on `x-distance`. The idea being to filter out outliers. However, as shall be seen this is a simplistic approach which does not yield robust results.

The evaluation of the TTC  is done in the `computeTTCLidar*` functions in the `src/ttc.cpp` file and computation of the median is performed in `computeMedian.cpp` which can be found in `src/utils.cpp`. Filtering of the lidar points is performed in the functions `cropLidarPoints` located in `lidarData.cpp` and `clusterLidarWithROI` located in `cameraFusion.cpp`.

### TTC Computation Camera

A Camera sensor does not provide direct measurement of the distance to objects. However, in the case of TTC computation, using sequential frames, distance ratios can be inferred from the camera properties. The image and formulas below illustrates how this can be achieved for the `pinhole model` and the constant velocity model.

<img src="docs/readme_images/camera_pinhole_distance_computation.png" width="720" height="240"  />

Using the constant velocity model one can calculate the TTC using camera measurements by using the `equation (4)` below.

<img src="docs/readme_images/ttc_camera_formula.png" width="600" height="340" />

However, to be able to compute the TTC with the method above, one needs a way to compute the ratio `h_1/h_0`, of the height of an object between successive frames.

To this end, in this project keypoints/feature detection is used. Where the keypoints detected and matches between successive frames are used to compute a set of distances ("heights") in the current and previous frame. By taking the ratio of this distances, one obtains a set of distance ratio values `d_0/d_1`, where `d_0` is the distance to the preceding vehicle in the previous frame and `d_1` is the distance to the preceding vehicle in the current frame. The basic principle is depicted in the image below.

<img src="docs/readme_images/keypoint_distances_overview.jpg" width="600" height="400" />

Similarly as in the Lidar case, as an attempt to achieve a robust computation of the TTC, the keypoint matches are first filtered. Only those keypoints are kept that fall inside the YOLO bounding box and are _2 standard deviations_ away from the mean keypoint distance between current and previous frame.

Finally, the distance to the preceding vehicle is estimated taking the median of the "height" ratios, after being sorted in ascending order. The idea being to filter out outliers. However, as shall be seen this is a simplistic approach which does not yield robust results.

The evaluation of the TTC  is done in the `computeTTCCamera` function in the `src/ttc.cpp` file and computation of the median is performed in `computeMedian.cpp` which can be found in `src/utils.cpp`. The filtering and keypoint association is implemented in `clusterKptMatchesWithROI` function inside the `src/cameraFusion.cpp` file.

## Results - TTC Lidar

The tables below list the results for TTC computation for the 19 frames provided from the KITTI data set using the lidar mesurements only. The aforementioned constant velocity model and distance computation implementation are used. In the table `_c` subscripts stands for `current` and `_p` subscript stands for `previous`.

_**LIDAR TTC results**_

|FRAME # | TTC [s]  | min(x_p) [m]| min(x_c) [m]| median(x_p) [m]| median(x_c) [m]|
|:-------:|:--------:|:----------:|:-----------:|:-----------:|:-----------:|
| 0       | N/A      |  N/A       | 7.97        |   N/A       |  8.072      |
| 1       | 12.51    |   7.97     | 7.91        |   8.072     |  8.008      |
| 2       | 11.51    |   7.91     | 7.85        |   8.008     |  7.939      |
| 3       | 15.78    |   7.85     | 7.79        |   7.939     |  7.889      |
| 4       | 16.68    |   7.79     | 7.68        |   7.889     |  7.842      |
| 5       | 15.28    |   7.68     | 7.64        |   7.842     |  7.79       |
| 6       | 12.67    |   7.64     | 7.58        |   7.79      |  7.73       |
| 7       | 11.98    |   7.58     | 7.65        |   7.73      |  7.66       |
| 8       | 13.35    |   7.65     | 7.51        |   7.66      |  7.61       |
| 8       | 13.02    |   7.51     | 7.47        |   7.61      |  7.55       |
| 10      | 11.34    |   7.47     | 7.39        |   7.55      |  7.49       |
| 11      | 12.59    |   7.39     | 7.20        |   7.49      |  7.43       |
| 12      | 8.90     |   7.20     | 7.27        |   7.43      |  7.34       |
| 13      | 9.63     |   7.27     | 7.19        |   7.34      |  7.27       |
| 14      | 9.59     |   7.19     | 7.13        |   7.27      |  7.19       |
| 15      | 8.26     |   7.13     | 7.04        |   7.19      |  7.11       |
| 16      | 10.20    |   7.04     | 6.83        |   7.11      |  7.04       |
| 17      | 9.95     |   6.83     | 6.90        |   7.04      |  6.97       |
| 18      | 8.40     |   6.90     | 6.81        |   6.97      |  6.89       |

### Observations

The plot below shows the minimum X distance (`min(X_c)`) for all frame versus the medium X distance (`median(x_c)`). As the plot shows, the median distance is a better estimate of the distance as it is less subjected to noise in the measurment data.

<img src="docs/readme_images/results/lidarPrecedingVehicleDistance.png" width="600" height="400" />

The plot below shows the TTC from the table above per frame. Although the median distance is a better approximate of the distance, taking the difference between previous and current distance measurement, amplifies any inconsistency and the resulting TTC is not monotonically decreasing.

<img src="docs/readme_images/results/lidarTTCresult.png" width="600" height="400" />

The following instance are cases where the lidar measurement seems to be inconsistent. In the 3Dview object representation below, the x-position of the median point used for TTC computation is shown as a circle of cyan color.

| FRAME #|  TTC  [s] |         |
|:-------:|:--------:|:--------:|
| 2  | 11.51 | <img src="docs/readme_images/results/3rd_frame_3dobjectview.png" width="800" height="250" /> |
| 3  | 15.78 | <img src="docs/readme_images/results/4th_frame_3dobjectview.png" width="800" height="250" /> |
| 4  | 16.68 | <img src="docs/readme_images/results/5th_frame_3dobjectview.png" width="800" height="250" /> |
| 14 | 9.59 | <img src="docs/readme_images/results/15th_frame_3dobjectview.png" width="800" height="250" /> |
| 16 | 10.2 | <img src="docs/readme_images/results/17th_frame_3dobjectview.png" width="800" height="250" /> |

From the analysis of the 3Dview's:
* in FRAME #3 frame, the points have a bigger spread with more outliers on the edges as compared for example with the previous distribution from FRAME #2. This causes the median to be estimated further from the tailgate and more inside the point cloud which leads to a sudden increase in the TTC value. The blue line crossing the ROI box is the 8 [m] estimate which shows that the shift in the vehicle position is not as pronounced as the TTC estimate.
* in FRAME #4, the spread is further increased due to the single outlier in front of the tailgate (probably unfiltered ground plane point). This creates a further increase in the TTC.
* in FRAME #16, the data is slightly more spread than preceding instances (e.g. FRAME #14) with a visible outlier well in front of the tailgate, causing again a shift of the detected median point further from the tailgate.


## Results - TTC Camera

The tables below list the results for TTC computation for the 19 frames provided from the KITTI data set using the camera measurements only. For this example the FAST detector with the BRISK descriptor was used. An analysis of different descriptor/detector combinations is provided in the next section.

The aforementioned constant velocity model and distance computation implementation are used. In the table `dist_ratio` denotes the value of `d0/d1` (previous-frame / current-frame distance as detailed in the lecture notes.


_**Camera TTC results**_

| FRAME # | TTC [s]  | median(dist_ratio) [m] |
|:-------:|:-----:|:------------:|
| 0       | N/A   | N/A   |
| 1       | 12.40 |1.00807|
| 2       | 12.52 |1.00798|
| 3       | 14.29 |1.007  |
| 4       | 12.85 |1.00778|
| 5       | inf   |1      |
| 6       | 14.25 |1.00702|
| 7       | 14.33 |1.00698|
| 8       | 12.87 |1.00821|
| 8       | 12.71 |1.00787|
| 10      | 14.82 |1.00675|
| 11      | 13.36 |1.00748|
| 12      | 13.70 |1.0073 |
| 13      | 12.28 |1.00808|
| 14      | 11.48 |1.00871|
| 15      | 12.40 |1.00806|
| 16      | 12.73 |1.00785|
| 17      | 11.51 |1.00869|
| 18      | 12.46 |1.00803|

The results from camera TTC are less intuitive. As long as the ratio is larger than 1, it means that the estimated current distance is less then the previous estimated distance (`d0/d1 > 1` => `d0 > d1`). However, given the overall TTC values, the TTC computation based on this descriptor/detector combination doesn't seem to be very consistent nor in line with the Lidar results.

### Keypoint Detector/Descriptor Performance

### Recommendations

E.g.

| TOP  | Detector                | Descriptor |
|:----:|:-----------------------:|:----------:|
| 1    | FAST                |  BRIEF
| 2    | FAST                  |  ORB
| 3    | SHI-TOMASI or HARRIS    |  BRIEF
