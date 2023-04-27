#include "core/types.hpp"
#include <cstdio>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <system_error>

using namespace cv;
using namespace std;

// header files
#include "main.hpp"
#include "opencv_aee.hpp"

// defining values
#define DEBUG 1
#define IMPATH "/Users/leoriley/Documents/projects/line folowing/images/26.png"
#define IMAGEWIDTH 100
#define IMAGEHEIGHT 100
#define STRAITSTEERINGANGLE 56
#define ANGLEMULTIPLIER 1

// Include files for required libraries

// #include "main.hpp"     // You can use this file for declaring defined values
// and functions

vector<Mat> setup(void) {
  /// Setup camera won't work if you don't have a compatible webcam
  //        setupCamera(320, 240);  // Enable the camera for OpenCV

  // import symbols from /Symbol images
  vector<Mat> symbols;
  symbols.push_back(imread("/Users/leoriley/Documents/projects/line "
                           "folowing/Symbol images/Circle (Red Line).png"));
  symbols.push_back(imread("/Users/leoriley/Documents/projects/line "
                           "folowing/Symbol images/Star (Green Line).png"));
  symbols.push_back(imread("/Users/leoriley/Documents/projects/line "
                           "folowing/Symbol images/Triangle (Blue Line).png"));
  symbols.push_back(
      imread("/Users/leoriley/Documents/projects/line folowing/Symbol "
             "images/Umbrella (Yellow Line).png"));

  // convert symbols to masks
  for (int i = 0; i < symbols.size(); i++) {
    inRange(symbols[i], Scalar(0, 20, 20), Scalar(180, 255, 255), symbols[i]);
    // debugDisplay(symbols[i], "symbol");
  }
  return symbols;
}

Mat ConvertImageForLineFollowing(Mat input);
vector<Point> findStartAndEndPoints(cv::Mat &mask);

int main(int argc, char **argv) {
  vector<Mat> symbols = setup();
  Mat image, imageHSV, mask, result, BluredHSV;
  int colour = 0;
  while (1) {
    // get image
    image = getImage(DEBUG, IMPATH);
    cvtColor(image, imageHSV, COLOR_BGR2HSV);
    colour = DetectLineColour(image, symbols, colour);
    imageHSV = ConvertImageForLineFollowing(imageHSV);
    mask = maskBasedOnColor(imageHSV, colour);

    vector<Point> points;
    points = findStartAndEndPoints(mask);
    vector<int> angleSpeed;
    angleSpeed = calculateAngleAndOffset(points);
    // print out the angle and speed values
    printf("Angle: %d, Speed: %d\n", angleSpeed[0], angleSpeed[1]);

    if (DEBUG == 1) {
      break;
    }
  }

  return 0;
}

Mat maskBasedOnColour(int inputImage, int colour) {
  // 0 = black 1 = red 2 = green 3 = blue, 4 = yellow
  Mat mask, mask2;
  switch (colour) {
  case 0: {
    inRange(inputImage, Scalar(0, 0, 0), Scalar(180, 255, 50), mask);
    break;
  }
  case 1: {
    // mask the red color
    inRange(inputImage, Scalar(0, 50, 20), Scalar(30, 255, 255), mask);
    inRange(inputImage, Scalar(150, 50, 20), Scalar(180, 255, 255), mask2);
    bitwise_or(mask, mask2, mask);
    break;
  }
  case 2: {
    // mask green colour
    inRange(inputImage, Scalar(30, 50, 20), Scalar(90, 255, 255), mask);
    break;
  }
  case 3: {
    // mask blue colour
    inRange(inputImage, Scalar(90, 50, 20), Scalar(130, 255, 255), mask);
    break;
  }
  case 4: {
    // mask yellow colour
    inRange(inputImage, Scalar(10, 50, 20), Scalar(40, 255, 255), mask);
    break;
  }
  }
  return mask;
}

int DetectLineColour(Mat image, vector<Mat> symbols, int previousColour) {
  // change perspective
  cvtColor(image, image, COLOR_BGR2HSV);
  // isolate pink colour from image
  Mat mask;
  inRange(image, Scalar(120, 20, 20), Scalar(177, 255, 255), mask);
  debugDisplay(mask, "mat of color");
  // find contours
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  // find the corners of the largest contour
  vector<Point> corners;

  // uses algorithm to simplify the largest contour and find the corners
  double epsilon = 0.1 * arcLength(contours[0], true);
  vector<Point> fourCorners;
  approxPolyDP(contours[0], fourCorners, epsilon, true);
  if (fourCorners.size() != 4) {
    printf("no corners found\n");
    return previousColour;
  }
  Mat symbol = isolateSymbol(fourCorners, mask);

  printf("image is of size %d,%d\n", symbol.rows, symbol.cols);
  printf("symbol is of size %d,%d\n", symbols[0].rows, symbols[0].cols);
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < symbols.size(); i++) {
      float matchPercent = compareImages(symbol, symbols[i]);
      if (matchPercent > 80) {
        debugDisplay(symbols[i], "symbol match");
        return i;
      }
      printf("for image i match percent was %f\n", matchPercent);
    }
    rotate(symbol, symbol, ROTATE_90_CLOCKWISE);
  }

  return 1;
}
// given to us
float compareImages(Mat cameraImage, Mat librarySymbol) {
  float matchPercent =
      100 - (100 / ((float)librarySymbol.cols * (float)librarySymbol.rows) *
             (2 * (float)countNonZero(
                      librarySymbol ^
                      cameraImage))); // An incorrect pixel has double weighting
  return matchPercent;
}

void debugDisplay(Mat image, String title) {
  if (DEBUG == 1) {
    imshow(title, image);
    waitKey(0);
  }
}

vector<Point> findStartAndEndPoints(Mat &mask) {
  // Detect lines in the mask using Hough Line Transform
  vector<Vec4i> lines;
  vector<Point> points;
  cv::HoughLinesP(mask, lines, 1, CV_PI / 180, 50, 50, 10);

  if (!lines.empty()) {
    Point start(lines[0][0], lines[0][1]);
    Point end(lines[0][2], lines[0][3]);
    points.assign({start, end});

    // Draw a circle around the start and end points
    circle(mask, start, 5, Scalar(0, 255, 0), 2);
    circle(mask, end, 5, Scalar(0, 255, 0), 2);
    return points;
  }
  return points;
}

vector<int> calculateAngleAndOffset(vector<Point> points) {
  // calculate angle and offset
  // point 0 is at the bottom of the image

  // work out which point is further down the image

  if (points[0].y > points[1].y) {
    // swap points
    Point temp = points[0];
    points[0] = points[1];
    points[1] = temp;
  }
  int x1 = points[0].x;
  int y1 = points[0].y;
  int x2 = points[1].x;
  int y2 = points[1].y;
  // calculate if the line goes left or right
  bool goingLeft = false;
  if (x1 < x2) {
    goingLeft = true;
  }

  int angle = atan2(abs(x2 - x1), abs(y2 - y1)) * 180 / CV_PI;
  int offset = abs(x2 - IMAGEWIDTH / 2);
  vector<int> angleAndOffset;
  angleAndOffset.assign({angle, offset});
  if (DEBUG == 1) {
    printf("x1: %d, y1: %d, x2: %d, y2: %d\n", x1, y1, x2, y2);
    printf("Angle: %d\nOffset: %d\n", angle, offset);
    printf("line going left = %d\n", goingLeft);
  }
  if (goingLeft) {
    angle = -angle;
  }

  // calculating steering angle
  int steeringAng;
  steeringAng =
      STRAITSTEERINGANGLE + angle * ANGLEMULTIPLIER * ((offset * -0.015) + 1);
  int speed = 100 + (1 / abs(angle)) * 60;
  vector<int> angleSpeed;
  angleSpeed.assign({steeringAng, speed});
  return angleSpeed;
}

vector<Point> ContoursEdgeMatch(vector<Point> Contour) {
  // describe this function
  // find the corners of the largest contour
  for (int i = 0; i < Contour.size(); i++) {
    if (Contour[i].x == 0 || Contour[i].x == 500 || Contour[i].y == 0 ||
        Contour[i].y == 500) {
      Contour.erase(Contour.begin() + i);
    }
  }
  return Contour;
}

Mat getImage(int debug, string debugImage) {
  //   gets an image from the camera or if debug = 1 gets an image from a
  //   file
  //   returns the image as a matrix
  Mat image;
  if (debug == 0) {
    image = captureFrame();
  } else {
    image = imread(debugImage, IMREAD_COLOR);
  }
  // flip image vertically
  flip(image, image, -1);
  // image.convertTo(image, -1, 2, 0);

  if (debug == 1) {
    imshow("base image", image);
    waitKey(0);
  }

  return image;
}

Mat ConvertImageForLineFollowing(Mat input) {
  Mat output;
  output = cropImageTop(output);
  resize(input, output, Size(IMAGEWIDTH, IMAGEHEIGHT), INTER_LINEAR);

  return output;
}

Mat cropImageTop(Mat image) {
  // crop the image from the top

  // Define the region of interest (ROI) as a rectangle that covers the top
  // portion of the image
  int x = 0; // x-coordinate of the top-left corner of the rectangle
  int y =
      image.rows * 0.4; // y-coordinate of the top-left corner of the rectangle
  int width = image.cols; // width of the rectangle
  int height = image.rows -
               y; // height of the rectangle, adjust the percentage as needed
  Rect roi(x, y, width, height);

  // Crop the image using the ROI
  Mat cropped = image(roi);

  // Display the cropped image

  // imshow("Cropped Image", cropped);
  // waitKey(0);
  return cropped;
}

Mat isolateSymbol(vector<Point> corners, Mat image) {
  Mat warped;
  int imagewidth = 350;
  int imageheight = 350;
  Size symbolsize(350, 350);

  // convterting corners to point2f
  Point2f first[4];
  for (int i = 0; i < corners.size(); i++) {
    first[i] = static_cast<Point2f>(corners[i]);
  }

  Point2f second[] = {Point2f(0, 0), Point2f(imagewidth - 1, 0),
                      Point2f(imagewidth - 1, imageheight - 1),
                      Point2f(0, imageheight - 1)};
  Mat trasnform = getPerspectiveTransform(first, second);
  warpPerspective(image, warped, trasnform, symbolsize);
  debugDisplay(warped, "symbol");
  return warped;
}

Mat perspectiveWarp(Mat image) {
  int widthnumber = 500;
  // Define the source and destination points for the perspective transform
  Point2f first[] = {Point2f(0, 0), Point2f(image.cols - 1, 0),
                     Point2f(image.cols - 1, image.rows - 1),
                     Point2f(0, image.rows - 1)};
  Point2f second[] = {Point2f(0, 0), Point2f(image.cols - 1, 0),
                      Point2f(image.cols - widthnumber - 1, image.rows - 1),
                      Point2f(widthnumber, image.rows - 1)};

  // applies a transform with the other edge getting contracted
  // Point2f second[] = {
  //     Point2f(widthnumber, 0), Point2f(image.cols - widthnumber - 1, 0),
  //     Point2f(image.cols - 0 - 1, image.rows - 1), Point2f(0, image.rows -
  //     1)};

  // Apply the perspective transform
  Mat warped;

  Mat trasnform = getPerspectiveTransform(first, second);
  warpPerspective(image, warped, trasnform, image.size());

  // Display the warped image
  // if (DEBUG == 1) {
  //   imshow("Warped Image", warped);
  //   waitKey(0);
  // }
  return warped;
}

// calculate line angle from mask
double lineAngle(Mat mask) {
  // find the line
  vector<Vec4i> lines;
  HoughLinesP(mask, lines, 1, CV_PI / 180, 50, 50, 10);
  // draw the lines
  Mat lineImage = Mat::zeros(mask.size(), CV_8UC3);
  int x = 0;
  for (size_t i = 0; i < lines.size(); i++) {
    x += 5;
    Vec4i l = lines[i];
    line(lineImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(x, 255, 255),
         1, LINE_AA);
  }
  // find the angle
  double angle = 0;
  // loop through each line in lines
  for (size_t i = 0; i < lines.size(); i++) {
    Vec4i l = lines[i];
    angle += atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
  }
  angle = angle / lines.size();
  // display the line image
  if (DEBUG == 1) {
    imshow("lineImage", lineImage);
    waitKey(0);
  }
  return angle;
}

// void sendData(double angle)
// {
//     if (DEBUG == 0)
//     {
//     }
//     else if (DEBUG == 1)
//     {
//         count << format("The stearing angle should be {}\n", angle) <<
//         endl;
//     }
//     // Serial.println(angle);
//     // Serial.flush();
// }

// #include <opencv2/opencv.hpp>
// #include <iostream>

// using namespace std;
// using namespace cv;

// int detectShape(Mat image)
// {
//     Mat image = imread("OpenCV_Logo.png");
//     bool done = false;

//     if (image.data == NULL)
//     {
//         cout << "No image found! Check path." << endl;
//         return 1; // ERROR
//     }
//     else
//     {
//         namedWindow("Selected Image", CV_WINDOW_AUTOSIZE);
//         imshow("Selected Image", image);
//         waitKey(); // without this image won't be shown

//         while (!done)
//         {

//             Mat hsv;
//             Point2f first[] = {Point2f(0, 0), Point2f(image.cols - 1, 0),
//             Point2f(image.cols - 1, image.rows - 1), Point2f(0,
//             image.rows
//             - 1)}; Point2f second[] = {Point2f(0, 0), Point2f(400, 0),
//             Point2f(400, 400), Point2f(0, 400)}; cvtColor(image, hsv,
//             COLOR_BGR2HSV);

//             // Define the color range you want to detect (in this case,
//             red) Scalar lower_color = Scalar(0, 50, 50); Scalar
//             upper_color = Scalar(10, 255, 255);

//             // Create a mask to select only the color range
//             Mat mask;
//             inRange(hsv, lower_color, upper_color, mask);

//             GaussianBlur(hsv, hsv, Size(3, 3), 0);
//             Canny(hsv, hsv, 100, 200);
//             std::vector<std::vector<Point>> c;

//             findContours(hsv, c, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//             Mat trasnform = getPerspectiveTransform(first, second);
//             Mat output;
//             warpPerspective(image, output, trasnform, size(400, 400));

//             // testing
//             imshow("Original", image);
//             imshow("Transformed", output);
//         }
//     }
//     return 0; // OK
// }
