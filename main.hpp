
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

Mat getImage(int debug = 0, string debugImage = "/images/1.jpg");
// declaring functions
Mat maskRed(Mat image);
Mat maskBlack(Mat image);
Mat maskBlack(Mat image);
Point findContourCentre(vector<Point> contour);
Mat cropImageTop(Mat image);
Mat perspectiveWarp(Mat image);
double lineAngle(Mat mask);
vector<vector<Point>> findContour(Mat image);
vector<Point> ContoursEdgeMatch(vector<Point> Contour);
vector<int> calculateAngleAndOffset(vector<Point> points);
int DetectLineColour(Mat image, vector<Mat> symbols, int previousColour = 0);
Mat isolateSymbol(vector<Point> corners, Mat image);
float compareImages(Mat cameraImage, Mat librarySymbol);
void debugDisplay(Mat image, String title);
Mat maskBasedOnColour(int inputImage, int colour);
int checkNumberOfMaskedPixels(Mat mask);