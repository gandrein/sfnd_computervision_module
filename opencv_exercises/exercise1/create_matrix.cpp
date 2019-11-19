#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>


using namespace std;

void createMatrix1()
{
    // single-channel matrix with 8bit unsigned elements
    int nrows = 480, ncols = 640;
    cv::Mat m1_8u;
    m1_8u.create(nrows, ncols, CV_8UC1); 
    m1_8u.setTo(cv::Scalar(255));        // white


    cv::Mat m3_8u;
    m3_8u.create(nrows, ncols, CV_8UC3); 
    m3_8u.setTo(cv::Scalar(255, 0, 0));        

    // show result
    string windowName = "First steps in OpenCV (m1_8u)";
    cv::namedWindow(windowName, 1); // create window
    cv::imshow(windowName, m1_8u);
    cv::waitKey(0); // wait for keyboard input before continuing

    // show result
    windowName = "OpenCV Matrix (m3_8u)";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, m3_8u);
    cv::waitKey(0);
}


int main()
{
    createMatrix1();
    return 0;
}