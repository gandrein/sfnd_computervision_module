# SFND 3D Object Tracking

<img src="docs/readme_images/keypoints.png" width="820" height="248" />

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

## Results

The tables below list the results for TTC computation for the 19 frames provided from the KITTI data set, for lidar TTC and camera TTC respectively, using the aforementioned constant velocity model and distance computation implementations.

_**LIDAR TTC results**_

|#Image   | TTC   | min(X_p)| min(X_c)| med(X_p)| med(X_c)|
|:-------:|:------:|:------:|:-------:|:-------:|:-------:|
| 0       | 127    | 50     | 162     |     254 |     119 |


_**Camera TTC results**_

|#Image   | TTC   | min(X_p)| min(X_c)| med(X_p)| med(X_c)|
|:-------:|:------:|:------:|:-------:|:-------:|:-------:|
| 0       | 127    | 50     | 162     |     254 |     119 |

**TODO** make a table for lidar:
 * each row is a frame
 * for each row show: ttc lidar, min x distance current frame, min x distance previous frame, same for median values


**TODO** make a table for camera:
 * each row is a frame
 * for each row show: ttc camera, ? distance ratios ??, same for median values


### Observations

**TODO** show a few cases of lidar median being off

**TODO** show a few cases where the keypoints in the images are off


### Keypoint Detector/Descriptor Performance


### Recommendations

E.g.

| TOP  | Detector                | Descriptor |
|:----:|:-----------------------:|:----------:|
| 1    | FAST                |  BRIEF
| 2    | FAST                  |  ORB
| 3    | SHI-TOMASI or HARRIS    |  BRIEF
