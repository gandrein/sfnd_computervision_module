# SFND 3D Object Tracking

<img src="images/keypoints.png" width="820" height="248" />

This project focuses on computing Time-To-Collision (TTC) estimates for a tracked object preceding our vehicle. The goal is to use lidar and camera data to detect and track preceding vehicles. This is achieved by using feature detection methods to detect, describe and track keypoints between successive images, the YOLO deep-learning framework to detect vehicles and associate them with the features. Finally lidar points in 3D space are used to build a fused tracking system.

This final project consists of four main parts:
1. Matching YOLO detected objects between successive frames using image keypoint correspondences in order to track 3D objects over time.
2. Making use of provided code that fuses image data to lidar points to compute a lidar only TTC estimate for colliding with the vehicle in front, in the ego-lane. The (simplest) velocity model is used for TTC computation.
3. Similarly, the computation of a camera only TTC estimate for colliding with the vehicle in front.
4. Evaluating performance, identify the most suitable detector/descriptor combination for TTC estimation and also identifying problems that can lead to faulty measurements by the camera or lidar sensors.

The assignment and the code is based on the [SFND_3D_Feature_Tracking](https://github.com/udacity/SFND_3D_Feature_Tracking) github repository.

In principle this project, implements the missing components of the schematic below. The other components being implemented during the `SFND_3D_Feature_Tracking` project or course exercises.

<img src="images/course_code_structure.png" width="779" height="414" />

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

### TTC Computation Lidar

### TTC Computation Camera


## Results

### Observations

### Recommendations

E.g.

| TOP  | Detector                | Descriptor |
|:----:|:-----------------------:|:----------:|
| 1    | FAST                |  BRIEF
| 2    | FAST                  |  ORB
| 3    | SHI-TOMASI or HARRIS    |  BRIEF
