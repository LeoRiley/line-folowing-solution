// int main1( int argc, char** argv )
//     {
//     setup();    // Call a setup function to prepare IO and devices

//     cv::namedWindow("Photo");   // Create a GUI window called photo

// //        while(1)    // Main loop to perform image processing

//     Mat frame;

//     // Can't capture frames without a camera attached. Use static images instead
//     while(frame.empty())
//     {
//         /// Can't capture frames without a camera attached. Use static images instead
// //                frame = captureFrame(); // Capture a frame from the camera and store in a new matrix variable
//         string imagelocation = "/Users/leoriley/Documents/projects/opencv3/opencvpractice/opencvpractice/OpenCV_Logo.png";
//         frame = imread(imagelocation, IMREAD_COLOR);
//     }
// //

//     Mat temp2;
//     int oldi = 0;
//     int highestValues = 0;
//     int hue = 0;
//     for (int j = 0;j <sizeof(photos);j++){
//         string imagelocation = "/Users/leoriley/Pictures/images2/"+photos[j];
//         frame = imread(imagelocation, IMREAD_COLOR);
//         highestValues = 0;
//         hue = 0;
//         for (int i = 10; i<=180; i += 10){

//             cvtColor(frame, framehsv, COLOR_BGR2HSV);

//             inRange(framehsv, Scalar(oldi,100,0), Scalar(i,255,255), frametemp);
//             int numberofpixles = countNonZero(frametemp);
// //            printf("h: %d-%d ,pixles:%d,\n",oldi,i,);
//             if (highestValues < numberofpixles) {
//                 highestValues = numberofpixles;
//                 hue = i;
//             }
//             cvtColor(frametemp , frametemp, COLOR_GRAY2BGR);
//             hconcat(frame, frametemp, temp2);
//             cv::Mat mat(480, 640, CV_8UC3, cv::Scalar(255,0,255));
// //            cv::imshow("Photo", mat);

// //            while (1){
// //                int key = cv::waitKey(1);   // Wait 1ms for a keypress (required to update windows)
// //
// //                key = (key==255) ? -1 : key;    // Check if the ESC key has been pressed
// //                if (key == 27) break;
// //            }

//             oldi = i;

//         }
//         printf("the most comon hue is %d\n",hue);
//     }

// //            cv::imshow("Photo", frame); //Display the image in the window

//         closeCV();  // Disable the camera and close any windows

//         return 0;
//     }

int edgeDetection(Mat img)
{
    // Convert to graycsale
    Mat img_gray;
    cvtColor(img, img_gray, COLOR_BGR2GRAY);
    // Blur the image for better edge detection
    Mat img_blur;
    GaussianBlur(img_gray, img_blur, Size(3, 3), 0);

    // Sobel edge detection
    Mat sobelx, sobely, sobelxy;
    Sobel(img_blur, sobelx, CV_64F, 1, 0, 5);
    Sobel(img_blur, sobely, CV_64F, 0, 1, 5);
    Sobel(img_blur, sobelxy, CV_64F, 1, 1, 5);
    // Display Sobel edge detection images
    imshow("Sobel X", sobelx);
    waitKey(0);
    imshow("Sobel Y", sobely);
    waitKey(0);
    imshow("Sobel XY using Sobel() function", sobelxy);
    waitKey(0);

    // Canny edge detection
    Mat edges;
    Canny(img_blur, edges, 100, 200, 3, false);
    // Display canny edge detected image
    imshow("Canny edge detection", edges);
    waitKey(0);

    destroyAllWindows();
    return 0;
}