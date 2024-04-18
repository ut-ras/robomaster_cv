#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
 
using namespace cv;
using namespace std;

int main( int argc, char** argv ) {
    namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"
    moveWindow("Control", 1620, 0);

    int iLowH = 1;
    int iHighH = 179;

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

    while (true) {
        bool isClosed = 0;
        // VideoCapture cap("../../resources/IMG_2129.MOV"); //capture the video from webcam
        VideoCapture cap("../../resources/IMG_2127.MOV"); //capture the video from webcam
        // VideoCapture cap(0);

        if (!cap.isOpened() )  // if not success, exit program
        {
            cout << "Cannot open the web cam" << endl;
            return -1;
        }

        //Capture a temporary image from the camera
        Mat imgTmp;
        cap.read(imgTmp); 

        while (true) {
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
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            findContours(imgThresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
            drawContours(image_copy, contours, -1, Scalar(0, 255, 0), 2);

            namedWindow("Contours", WINDOW_AUTOSIZE); //create a window called "Contours"
            moveWindow("Contours", 1080, 0);
            resize(image_copy, image_copy, Size(540, 960));
            imshow("Contours", image_copy);

            //Calculate the moments of the thresholded image
            Moments oMoments = moments(imgThresholded);

            double dM01 = oMoments.m01;
            double dM10 = oMoments.m10;
            double dArea = oMoments.m00;

            // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
            // if (dArea > 10000)
            // {
            //     //calculate the position of the ball
            //     int posX = dM10 / dArea;
            //     int posY = dM01 / dArea;        
                        
            //     if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0)
            //     {
            //         circle(imgLines, Point(posX, posY), 10, Scalar(0, 255, 0), FILLED, 2);
            //     }

            //     iLastX = posX;
            //     iLastY = posY;
            //     cout << "X, Y pos is " << posX << ", " << posY << "\n";
            // }

            imgOriginal = imgOriginal + imgLines;

            namedWindow("Original", WINDOW_AUTOSIZE); //create a window called "Original"
            moveWindow("Original", 0, 0);
            resize(imgOriginal, imgOriginal, Size(540, 960));
            imshow("Original", imgOriginal); //show the original image

            namedWindow("Thresholded", WINDOW_AUTOSIZE); //create a window called "Thresholded"
            moveWindow("Thresholded", 540, 0);
            resize(imgThresholded, imgThresholded, Size(540, 960));
            imshow("Thresholded", imgThresholded);

            if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            {
                cout << "esc key is pressed by user" << endl;
                isClosed = 1;
                break; 
            }
        }
        if (isClosed) {
            break;
        }
    }
    return 0;
}