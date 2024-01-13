#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H
#include <tuple>
#include <vector>
#include <ctime>
#include <time.h>
#include <stdio.h>
#include <iostream>

class BoundingBox
{
    /*
     * Bounding Box class for Stampede Robomasters Team - 2023
     * Written by Tanay Garg
     *
     * Initializes a Bounding box object used by the ObjectLog and ArmorPlate classes
     */
private:
    double _xCenter;
    double _yCenter;
    double _depthVal;
    double _width;
    double _height;
    std::tuple<double, double, double> _position;
    time_t _timeStamp;

public:
    BoundingBox(double xCenter, double yCenter, double depthVal, double width, double height);
    BoundingBox();
    void setXCenter(double xCenter);
    void setYCenter(double yCenter);
    void setDepthVal(double depthVal);
    void setWidth(double width);
    void setHeight(double height);
    void setPosition(std::tuple<double, double, double> position);
    void setTimeStamp(time_t timeStamp);
    double getXCenter();
    double getYCenter();
    double getDepthVal();
    double getWidth();
    double getHeight();
    std::tuple<double, double, double> getPosition();
    time_t getTimeStamp();
    int main();

};

#endif