#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <stdlib.h>

using namespace cv;
using namespace std;

#define DEBUG 1
#define IMPATH "/Users/leoriley/Documents/projects/line folowing/images/7.jpg"

// Include files for required libraries
#include <stdio.h>

#include "opencv_aee.hpp"
// #include "main.hpp"     // You can use this file for declaring defined values and functions

void setup(void)
{
    /// Setup camera won't work if you don't have a compatible webcam
    //        setupCamera(320, 240);  // Enable the camera for OpenCV
}

Mat getImage(int debug = 0, string debugImage = "/images/1.jpg");
Mat maskRed(Mat image);
Mat maskBlack(Mat image);
int edgeDetection(Mat img);
Point findContourCentre(vector<Point> contour);
Mat cropImageTop(Mat image);
Mat perspectiveWarp(Mat image);
double lineAngle(Mat mask);

void display2Images(Mat image1, Mat image2);
vector<vector<Point>> findContour(Mat image);

int main(int argc, char **argv)
{
    setup();
    Mat image, imageHSV, mask, result;
    int colour = 1;
    while (1)
    {
        // get image
        image = getImage(DEBUG, IMPATH);
        // convert to HSV
        cvtColor(image, imageHSV, COLOR_BGR2HSV);
        // do singal recognition here and set colour to value with 0 being black 1 being red 2 being green 3 being blue
        if (colour == 0)
        {
            mask = maskBlack(imageHSV);
        }
        else if (colour == 1)
        {
            mask = maskRed(imageHSV);
        }
        else if (colour == 2)
        {
            // do green
        }
        // apples mask to image
        bitwise_and(image, image, result, mask);
        double angle = 69 - lineAngle(mask);
        cout << angle << endl;
        // find contour and the center of it
        vector<vector<Point>> contors = findContour(result);
        vector<vector<Point>> longContors;
        // loop through each vector in contors and if length is less than 20 remove it

        Point x = findContourCentre(contors[0]);
        if (DEBUG == 1)
        {
            // draws a cicle on the center point of the contour
            Mat output = image.clone();
            circle(output, x, 20, Scalar(0, 255, 0), -1);
            imshow("Output", output);
            waitKey(0);
        }
        if (DEBUG == 1)
        {
            break;
        }
    }

    return 0;
}

Mat getImage(int debug, string debugImage)
{
    //   gets an image from the camera or if debug = 1 gets an image from a
    //   file
    //   returns the image as a matrix
    Mat image;
    if (debug == 0)
    {
        // image = captureFrame();
    }
    else
    {
        image = imread(debugImage, IMREAD_COLOR);
    }
    // flip image vertically
    flip(image, image, -1);
    resize(image, image, Size(500, 500), INTER_LINEAR);
    // image.convertTo(image, -1, 2, 0);
    image = cropImageTop(image);
    imshow("initimage", image);
    waitKey(0);

    // image = perspectiveWarp(image);
    return image;
}

Mat maskRed(Mat image)
{
    Mat mask, mask2;
    // mask the red color
    inRange(image, Scalar(0, 50, 20), Scalar(20, 255, 255), mask);
    inRange(image, Scalar(160, 50, 20), Scalar(180, 255, 255), mask2);
    bitwise_or(mask, mask2, mask);
    if (DEBUG == 1)
    {
        imshow("mask", mask);
        waitKey(0);
    }
    return mask;
}

Mat maskBlack(Mat image)
{
    Mat mask, mask2;
    // mask the red color
    inRange(image, Scalar(0, 0, 0), Scalar(180, 255, 50), mask);
    // inRange(image, Scalar(160, 70, 20), Scalar(180, 255, 255), mask2);
    // bitwise_or(mask, mask2, mask);
    if (DEBUG == 1)
    {
        imshow("mask", mask);
        waitKey(0);
    }
    return mask;
}

void display2Images(Mat image1, Mat image2)
{
    Mat newImage;
    hconcat(image1, image2, newImage); // <---- place image side by side

    imshow("Display side by side", newImage);
    waitKey(0);
}

Point findContourCentre(vector<Point> contour)
{
    Moments foundRegion; // Variables to store the region moment and the centre point
    Point centre;
    foundRegion = moments(contour, false);          // Calculate the moment for the contour
    centre.x = (foundRegion.m10 / foundRegion.m00); // Calculate the X and Y positions
    centre.y = (foundRegion.m01 / foundRegion.m00);

    return centre;
}

vector<vector<Point>> findContour(Mat image)
{
    // Convert the image to grayscale
    Mat gray;
    // gray = image;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    if (DEBUG == 1)
    {
        imshow("gray", gray);
        waitKey(0);
    }
    // Apply thresholding to convert the image to a binary image
    Mat binary;
    threshold(gray, binary, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // Find the contours in the binary image
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(binary, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Point> centroids;
    vector<vector<Point>> contoursWithoutBorder;
    for (const auto &contour : contours)
    {
        int number_of_points_on_Border = 0;
        bool isBig = false;
        for (const auto &point : contour)
        {
            if (point.x == 0 || point.x == image.cols - 1 || point.y == 0 || point.y == image.rows - 1)
            {
                number_of_points_on_Border += 1;
                break;
            }
        }
        if (contour.size() > 100)
        {
            isBig = true;
        }

        if (number_of_points_on_Border < 10 && isBig)
        {
            contoursWithoutBorder.push_back(contour);
        }
    }

    // Draw the contours on the original image

    // Display the original image and the contours image
    if (DEBUG == 1)
    {
        Mat contoursImage = image.clone();
        drawContours(contoursImage, contoursWithoutBorder, -1, Scalar(255, 255, 255), 2);
        imshow("Original", image);
        imshow("Contours", contoursImage);
        waitKey(0);
    }

    return contoursWithoutBorder;
}

Mat cropImageTop(Mat image)
{
    // crop the image from the top

    // Define the region of interest (ROI) as a rectangle that covers the top portion of the image
    int x = 0;                   // x-coordinate of the top-left corner of the rectangle
    int y = image.rows * 0.4;    // y-coordinate of the top-left corner of the rectangle
    int width = image.cols;      // width of the rectangle
    int height = image.rows - y; // height of the rectangle, adjust the percentage as needed
    Rect roi(x, y, width, height);

    // Crop the image using the ROI
    Mat cropped = image(roi);

    // Display the cropped image

    // imshow("Cropped Image", cropped);
    // waitKey(0);
    return cropped;
}

Mat perspectiveWarp(Mat image)
{
    // Define the source and destination points for the perspective transform
    Point2f src[4], dst[4];
    src[0] = Point2f(0, 0);
    src[1] = Point2f(image.cols, image.rows);
    src[2] = Point2f(image.cols * 0.25, 0); // bottom right
    src[3] = Point2f(image.cols * 0.75, 0); // bottom left

    dst[0] = Point2f(image.cols * 0.2, image.rows);
    dst[1] = Point2f(image.cols * 0.8, image.rows);
    dst[2] = Point2f(0, 0);
    dst[3] = Point2f(image.cols, 0);

    // Compute the perspective transform
    Mat M = getPerspectiveTransform(src, dst);

    // Apply the perspective transform
    Mat warped;
    warpPerspective(image, warped, M, image.size());

    // Display the warped image
    if (DEBUG == 1)
    {
        imshow("Warped Image", warped);
        waitKey(0);
    }
    return warped;
}

// calculate line angle from mask
double lineAngle(Mat mask)
{
    // find the line
    vector<Vec4i> lines;
    HoughLinesP(mask, lines, 1, CV_PI / 180, 50, 50, 10);
    // draw the lines
    Mat lineImage = Mat::zeros(mask.size(), CV_8UC3);
    int x = 0;
    for (size_t i = 0; i < lines.size(); i++)
    {
        x += 5;
        Vec4i l = lines[i];
        line(lineImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(x, 255, 255), 1, LINE_AA);
    }
    // find the angle
    double angle = 0;
    // loop through each line in lines
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        angle += atan2(l[3] - l[1], l[2] - l[0]) * 180 / CV_PI;
    }
    angle = angle / lines.size();
    // display the line image
    if (DEBUG == 1)
    {
        imshow("lineImage", lineImage);
        waitKey(0);
    }
    return angle;
}