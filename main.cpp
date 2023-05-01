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
#include "I2C.hpp"

using namespace cv;
using namespace std;

// header files
#include "main.hpp"
#include "opencv_aee.hpp"

// defining values
#define DEBUG 0
#define IMPATH "/Users/leoriley/Documents/projects/line folowing/images/26.png"
#define IMAGEWIDTH 100
#define IMAGEHEIGHT 100
#define STRAITSTEERINGANGLE 56
#define ANGLEMULTIPLIER 1

VideoCapture cap(0);

// Include files for required libraries

// #include "main.hpp"     // You can use this file for declaring defined values
// and functions

vector<Mat> setup()
{
  if (!cap.isOpened())
  {
    std::cerr << "Error opening camera" << std::endl;
  }
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
  for (int i = 0; i < symbols.size(); i++)
  {
    inRange(symbols[i], Scalar(0, 20, 20), Scalar(180, 255, 255), symbols[i]);
    // debugDisplay(symbols[i], "symbol");
  }
  return symbols;
}

int main(int argc, char **argv)
{
  // variables defined
  vector<Mat> symbols = setup();
  Mat image, imageHSV, mask;
  vector<Point> points;
  vector<int> angleSpeed;
  int colour = 0;
  I2C I2Cconnection;
  // main loop for program
  while (1)
  {
    // Symbol Recognition
    image = getImage(DEBUG, IMPATH);
    cvtColor(image, imageHSV, COLOR_BGR2HSV); // Convert to HSV
    colour = DetectLineColour(
        image, symbols,
        colour); // Detect line colour that should be followed based on symbols
    // Line Detection
    imageHSV = ConvertImageForLineFollowing(
        imageHSV); // Lowers the resolution of the image to make it easier to
                   // process
    mask = maskBasedOnColour(imageHSV, colour);
    if (checkNumberOfMaskedPixels(mask) < 3)
    {
      // if the mask is empty then the car has finished folowing the line and so
      // returns to black line
      printf("Following Black line based on pixel Check\n");
      mask = maskBasedOnColour(imageHSV, 0);
      colour = 0;
    }

    // line direction detection
    if (findStartAndEndPoints(mask, points))
    { // if the line is found
      // calculate the angle and speed
      angleSpeed = calculateAngleAndOffset(points);
      // print out the angle and speed values
      printf("Angle: %d, Speed: %d\n", angleSpeed[0], angleSpeed[1]);
      // Code to Send information to car goes Here
    }
    else
    {
      // car continues strait
      printf("Line Not Found\n");
      angleSpeed[0] = STRAITSTEERINGANGLE;
      angleSpeed[1] = 200;

      I2Cconnection.SendData(angleSpeed[0], angleSpeed[1]);
    }

    if (DEBUG == 1)
    {
      break;
    }
  }
  I2Cconnection.CloseConnection();
  return 0;
}

Mat maskBasedOnColour(Mat inputImage, int colour)
{
  // 0 = black 1 = red 2 = green 3 = blue, 4 = yellow
  Mat mask, mask2;
  switch (colour)
  {
  case 0:
  {
    inRange(inputImage, Scalar(0, 0, 0), Scalar(180, 255, 50), mask);
    break;
  }
  case 1:
  {
    // mask the red color
    inRange(inputImage, Scalar(0, 50, 20), Scalar(30, 255, 255), mask);
    inRange(inputImage, Scalar(150, 50, 20), Scalar(180, 255, 255), mask2);
    bitwise_or(mask, mask2, mask);
    break;
  }
  case 2:
  {
    // mask green colour
    inRange(inputImage, Scalar(30, 50, 20), Scalar(90, 255, 255), mask);
    break;
  }
  case 3:
  {
    // mask blue colour
    inRange(inputImage, Scalar(90, 50, 20), Scalar(130, 255, 255), mask);
    break;
  }
  case 4:
  {
    // mask yellow colour
    inRange(inputImage, Scalar(10, 50, 20), Scalar(40, 255, 255), mask);
    break;
  }
  }
  return mask;
}

int checkNumberOfMaskedPixels(Mat mask)
{
  int count = 0;
  for (int i = 0; i < mask.rows; i++)
  {
    for (int j = 0; j < mask.cols; j++)
    {
      if (mask.at<uchar>(i, j) == 255)
      {
        count++;
      }
    }
  }
  return count;
}

int DetectLineColour(Mat image, vector<Mat> symbols, int previousColour)
{
  // change perspective
  cvtColor(image, image, COLOR_BGR2HSV);
  // isolate pink colour from image
  Mat mask;
  inRange(image, Scalar(120, 20, 20), Scalar(177, 255, 255),
          mask); // mask pink colour
  debugDisplay(mask, "mat of color");
  // find contours
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

  // uses algorithm to simplify the largest contour and find the corners
  double epsilon = 0.1 * arcLength(contours[0], true);
  vector<Point> fourCorners;
  approxPolyDP(contours[0], fourCorners, epsilon, true);
  if (fourCorners.size() != 4)
  {
    printf("no corners found\n");
    return previousColour;
  }
  Mat symbol = isolateSymbol(fourCorners, mask);

  for (int j = 0; j < 4; j++) // loop through all rotations of the symbol
  {
    for (int i = 0; i < symbols.size(); i++) // loop through all symbols
    {
      float matchPercent = compareImages(symbol, symbols[i]);
      if (matchPercent > 80)
      {
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
float compareImages(Mat cameraImage, Mat librarySymbol)
{
  float matchPercent =
      100 - (100 / ((float)librarySymbol.cols * (float)librarySymbol.rows) *
             (2 * (float)countNonZero(
                      librarySymbol ^
                      cameraImage))); // An incorrect pixel has double weighting
  return matchPercent;
}

void debugDisplay(Mat image, String title)
{
  // function that displays an image is the code is in debug mode
  if (DEBUG == 1)
  {
    imshow(title, image);
    waitKey(0);
  }
}

int findStartAndEndPoints(Mat &mask, vector<Point> &points)
{
  // Detect lines in the mask using Hough Line Transform
  vector<Vec4i> lines;
  HoughLinesP(mask, lines, 1, CV_PI / 180, 50, 50, 10);

  if (!lines.empty())
  {
    Point start(lines[0][0], lines[0][1]);
    Point end(lines[0][2], lines[0][3]);
    points.assign({start, end});
    return 1;
  }
  else
  {
    printf("no lines found\n");
    return 0;
  }
  return 0;
}

vector<int> calculateAngleAndOffset(vector<Point> points)
{
  // calculate angle and offset
  // point 0 is at the bottom of the image

  // work out which point is further down the image

  if (points[0].y > points[1].y)
  {
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
  if (x1 < x2)
  {
    goingLeft = true;
  }
  // calculate angle and offset
  int angle = atan2(abs(x2 - x1), abs(y2 - y1)) * 180 / CV_PI;
  int offset = abs(x2 - IMAGEWIDTH / 2);
  vector<int> angleAndOffset;
  angleAndOffset.assign({angle, offset});
  if (DEBUG == 1)
  {
    printf("x1: %d, y1: %d, x2: %d, y2: %d\n", x1, y1, x2, y2);
    printf("Angle: %d\nOffset: %d\n", angle, offset);
    printf("line going left = %d\n", goingLeft);
  }
  if (goingLeft)
  {
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

Mat getImage(int debug, string debugImage)
{
  //   gets an image from the camera or if debug = 1 gets an image from a
  //   file
  //   returns the image as a matrix
  Mat image;
  if (debug == 0)
  {
    cap >> image;
  }
  else
  {
    image = imread(debugImage, IMREAD_COLOR);
  }
  // flip image vertically
  flip(image, image, -1);

  if (debug == 1)
  {
    imshow("base image", image);
    waitKey(0);
  }

  return image;
}

Mat ConvertImageForLineFollowing(Mat input)
{
  Mat output;
  output = cropImageTop(output);
  resize(input, output, Size(IMAGEWIDTH, IMAGEHEIGHT), INTER_LINEAR);

  return output;
}

Mat cropImageTop(Mat image)
{
  // crop the image from the top

  // Define the region of interest (ROI) as a rectangle that covers the top
  // portion of the image
  int x = 0; // x-coordinate of the top-left corner of the rectangle
  int y =
      image.rows * 0.4;   // y-coordinate of the top-left corner of the rectangle
  int width = image.cols; // width of the rectangle
  int height = image.rows -
               y; // height of the rectangle, adjust the percentage as needed
  Rect roi(x, y, width, height);

  // Crop the image using the ROI
  Mat cropped = image(roi);

  return cropped;
}

Mat isolateSymbol(vector<Point> corners, Mat image)
{
  // isolates the symbol from the image and change the perspective and size to
  // match the reference images
  Mat warped;
  int imagewidth = 350;
  int imageheight = 350;
  Size symbolsize(350, 350);

  // convterting corners to point2f
  Point2f first[4];
  for (int i = 0; i < corners.size(); i++)
  {
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
