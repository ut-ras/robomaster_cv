#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
 
using namespace cv;
using namespace std;

bool rect_sort_function (Rect first, Rect second)
{
    return first.tl().x < second.tl().x;
}

int main( int argc, char** argv ) 
{
    namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"
    moveWindow("Control", 1620, 0);

    // Blue current using low 80 high 140
    int iLowH = 80;
    int iHighH = 140;

    // Blue current using low 100 high 255
    int iLowS = 100; 
    int iHighS = 255;

    // Blue current using low 175 high 255
    int iLowV = 175;
    int iHighV = 255;

    //Create trackbars in "Control" window
    createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH, 179);

    createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS, 255);

    createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV, 255);

    int iLastX = -1; 
    int iLastY = -1;

    while (true) 
    {
        bool isClosed = 0;
        // VideoCapture cap("../../resources/rmna2.mp4"); //xcapture the video from webcam

        if (!cap.isOpened() )  // if not success, exit program
        {
            cout << "Cannot open the web cam" << endl;
            return -1;
        }

        //Capture a temporary image from the camera
        Mat imgTmp;
        cap.read(imgTmp); 

        while (true) 
        {
            Mat imgOriginal;
            bool bSuccess = cap.read(imgOriginal); // read a new frame from video

            if (!bSuccess) //if not success, break loop
            {
                cout << "Cannot read a frame from video stream" << endl;
                break;
            }

            Mat imgHSV;
            cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
            Mat color_threshold;
            inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), color_threshold); //Threshold the image

            // Red Mask
            Mat mask1, mask2;
            inRange(imgHSV, Scalar(0, 70, 50), Scalar(10, 255, 255), mask1);
            inRange(imgHSV, Scalar(170, 70, 50), Scalar(180, 255, 255), mask2);
            color_threshold = mask1 | mask2;
            
            //morphological opening (removes small objects from the foreground)
            erode(color_threshold, color_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            dilate(color_threshold, color_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

            //morphological closing (removes small holes from the foreground)
            dilate(color_threshold, color_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
            erode(color_threshold, color_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            Mat image_copy = imgOriginal.clone();
            Mat image_contours = imgOriginal.clone();
            Mat image_all_bounded_boxes = imgOriginal.clone();

            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            findContours(color_threshold, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
            drawContours(image_contours, contours, -1, Scalar(0, 255, 0), 2);

            vector<Rect> accepted_rects;
            for (size_t i = 0; i < contours.size(); i++) // iterate through each contour.
            {
                Rect bounding_rect = boundingRect(contours[i]);
                if (bounding_rect.size().height / bounding_rect.size().width > 1
                    && bounding_rect.size().height / bounding_rect.size().width < 7)
                {
                    accepted_rects.push_back(bounding_rect);
                }
            }

            sort(accepted_rects.begin(), accepted_rects.end(), rect_sort_function);

            if (accepted_rects.size() >= 2)
            {
                Rect first = accepted_rects[0];
                Rect second = accepted_rects[1];

                int distance = abs(second.tl().y - first.tl().y);

                for (size_t i = 1; i + 1 < accepted_rects.size(); i++)
                {
                    // if (accepted_rects[i].tl().x - first_tallest.tl().x < 1000)
                        // continue;

                    if (abs(accepted_rects[i].tl().y - accepted_rects[i + 1].tl().y) < distance)
                    {
                        distance = abs(accepted_rects[i].tl().y - accepted_rects[i + 1].tl().y);
                        first = accepted_rects[i];
                        second = accepted_rects[i + 1];
                    }
                }

                rectangle(image_copy, first.tl(), second.br(), Scalar(0, 255, 0), 5);   
                // rectangle(image_copy, second.tl(), second.br(), Scalar(0, 255, 0), 5);  
                float x = (first.tl().x + second.br().x) / 2;
                float y = (first.tl().y + second.br().y) / 2;
                circle(image_copy, Point(x, y), 2, Scalar(0, 0, 255), 8);
            }

            for (size_t i = 0; i < accepted_rects.size(); i++) // iterate through each contour.
            {
                rectangle(image_all_bounded_boxes, accepted_rects[i].tl(), accepted_rects[i].br(), Scalar(0, 255, 0), 5); 
            }
            
            imshow("Bounded Box", image_copy); //show the modified image
            imshow("Accepted Bounding Boxes", image_all_bounded_boxes);
            // imshow("Thresholded Color", color_threshold);
            // imshow("Contours", image_contours);

            if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            {
                cout << "esc key is pressed by user" << endl;
                isClosed = 1;
                break; 
            }

            if (isClosed) 
            {
                break;
            }
        }
    }
    
    return 0;
}