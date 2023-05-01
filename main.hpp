
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

// used functions
Mat getImage(int debug = 0, string debugImage = "/images/1.jpg");
Mat cropImageTop(Mat image);
int findStartAndEndPoints(Mat &mask, vector<Point> &points);
Mat maskBasedOnColour(Mat inputImage, int colour);
int checkNumberOfMaskedPixels(Mat mask);
Mat ConvertImageForLineFollowing(Mat input);
vector<int> calculateAngleAndOffset(vector<Point> points);
int DetectLineColour(Mat image, vector<Mat> symbols, int previousColour = 0);
Mat isolateSymbol(vector<Point> corners, Mat image);
float compareImages(Mat cameraImage, Mat librarySymbol);
void debugDisplay(Mat image, String title);
