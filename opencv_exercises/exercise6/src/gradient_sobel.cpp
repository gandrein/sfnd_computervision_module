#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void gradientSobel(std::string direction) {
  // TODO: Based on the image gradients in both x and y, compute an image
  // which contains the gradient magnitude according to the equation at the
  // beginning of this section for every pixel position. Also, apply different
  // levels of Gaussian blurring before applying the Sobel operator and compare the results.

  // load image from file
  cv::Mat img;
  img = cv::imread("../images/img1gray.png");

  // convert image to grayscale
  cv::Mat imgGray;
  cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

  // create filter kernel
  float sobel[9];
  if (direction == "x") {
    // clang-format off
    float sobel_x[9] = {-1, 0, +1,
                        -2, 0, +2, 
                        -1, 0, +1};
    // clang-format on
    std::copy(std::begin(sobel_x), std::end(sobel_x), std::begin(sobel));
  } else if (direction == "y") {
    // clang-format off
    float sobel_y[9] = {-1, -2, -1, 
                         0, 0, 0, 
                        +1, +2, +1};
   std::copy(std::begin(sobel_y), std::end(sobel_y), std::begin(sobel));
    // clang-format on
  }
  cv::Mat kernel = cv::Mat(3, 3, CV_32F, sobel);

  // apply filter
  cv::Mat result;
  cv::filter2D(imgGray, result, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

  // show result
  string windowName = "Sobel operator (" + direction + "-direction)";
  cv::namedWindow(windowName, 1);  // create window
  cv::imshow(windowName, result);
  while ((cv::waitKey() & 0xEFFFFF) != 27) {
    continue;
  }  // wait for keyboard input before continuing
}

int main() {
  gradientSobel("x");
  gradientSobel("y");
}