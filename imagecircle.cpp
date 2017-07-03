#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <stdlib.h>
// #include <stdio.h>
#include <iostream>

using namespace cv;

/** @function main */
int main( int argc, char** argv )
{
  /// Load an image
  Mat src, image_gray;
  src = imread( argv[1] );
  if( !src.data ) { 
    return -1; 
  }
  cv::cvtColor(src, image_gray, CV_RGB2GRAY);
  std::vector<Vec3f> temp_circles, circles;

  // Detect 2 circles
  HoughCircles(image_gray, temp_circles, CV_HOUGH_GRADIENT, 2, 200, 100, 80, 80, 90);
  for (size_t i = 0; i < temp_circles.size(); i++) {
    circles.push_back(temp_circles[i]);
  }

  HoughCircles(image_gray, temp_circles, CV_HOUGH_GRADIENT, 2, 200, 100, 80, 120, 130);
  for (size_t i = 0; i < temp_circles.size(); i++) {
    circles.push_back(temp_circles[i]);
  }

  for (size_t i = 0; i < circles.size(); i++) {
    Point center (cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    circle(src, center, 2, Scalar(0, 255, 0), -1, 8, 0);
    circle(src, center, radius, Scalar(0, 0, 255), 3, 8, 0);
    std::cout << "Circle " << i << ": " << circles[i][0] << ", " 
      << circles[i][1] << ", " << circles[i][2] << std::endl;
  }

  namedWindow("Hough Circle Transform Demo", CV_WINDOW_FREERATIO);
  imshow( "Hough Circle Transform Demo", src);

  waitKey(0);
  return 0;
  }