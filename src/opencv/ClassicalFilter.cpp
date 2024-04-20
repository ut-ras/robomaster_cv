#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
 
using namespace cv;
using namespace std;

int main( int argc, char** argv ) 
{
    namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"
    moveWindow("Control", 1620, 0);

    // Blue current using low 94 high 120
    int iLowH = 94;
    int iHighH = 120;

    int iLowS = 150; 
    int iHighS = 255;

    int iLowV = 60;
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
        VideoCapture cap("../../resources/IMG_2129.MOV"); //capture the video from webcam

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
            Mat imgLines = Mat::zeros(imgTmp.size(), CV_8UC3);;
            bool bSuccess = cap.read(imgOriginal); // read a new frame from video

            if (!bSuccess) //if not success, break loop
            {
                cout << "Cannot read a frame from video stream" << endl;
                break;
            }

            Mat imgHSV;
            cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
            Mat imgThresholded;

            inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
            
            //morphological opening (removes small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

            //morphological closing (removes small holes from the foreground)
            dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
            erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            Mat image_copy = imgOriginal.clone();
            Mat image_contours = imgOriginal.clone();
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            findContours(imgThresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
            drawContours(image_contours, contours, -1, Scalar(0, 255, 0), 2);

            vector<Rect> bounding_rects(contours.size());
            vector<Rect> accepted_rects;
            for (size_t i = 0; i < contours.size(); i++) // iterate through each contour.
            {
                bounding_rects[i] = boundingRect(contours[i]);
                if (bounding_rects[i].size().height / bounding_rects[i].size().width > 1.5)
                {
                    accepted_rects.push_back(bounding_rects[i]);
                }
            }

            if (accepted_rects.size() >= 2)
            {
                Rect first_tallest = accepted_rects[0];
                Rect second_tallest = accepted_rects[1];

                for (size_t i = 2; i < accepted_rects.size(); i++) // iterate through each contour.
                {
                    if (first_tallest.size().height < accepted_rects[i].size().height)
                    {
                        first_tallest = accepted_rects[i];
                        second_tallest = first_tallest;
                    }
                }

                rectangle(image_copy, first_tallest.tl(), first_tallest.br(), Scalar(0, 0, 255), 2);   
                rectangle(image_copy, second_tallest.tl(), second_tallest.br(), Scalar(0, 0, 255), 2);  
                float x = (first_tallest.tl().x + second_tallest.br().x) / 2;
                float y = (first_tallest.tl().y + second_tallest.br().y) / 2;
                circle(image_copy, Point(x, y), 2, Scalar(0, 0, 255), 8);
            }
            else
            {
                for (size_t i = 0; i < accepted_rects.size(); i++) // iterate through each contour.
                {
                    rectangle(image_copy, accepted_rects[i].tl(), accepted_rects[i].br(), Scalar(0, 0, 255), 2);   
                }
            }

            imshow("Bounded Box", image_copy); //show the modified image
            // imshow("Thresholded", imgThresholded);
            imshow("Contours", image_contours);

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