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
    float _xCenter;
    float _yCenter;
    float _depthVal;
    float _width;
    float _height;
    std::tuple<float, float, float> _position;
    time_t _timeStamp;

public:
    BoundingBox(float xCenter, float yCenter, float depthVal, float width, float height);
    BoundingBox();
    void setXCenter(float xCenter);
    void setYCenter(float yCenter);
    void setDepthVal(float depthVal);
    void setWidth(float width);
    void setHeight(float height);
    void setPosition(std::tuple<float, float, float> position);
    void setTimeStamp(time_t timeStamp);
    float getXCenter();
    float getYCenter();
    float getDepthVal();                                               
    float getWidth();
    float getHeight();
    std::tuple<float, float, float> getPosition();
    time_t getTimeStamp();
    int main();

};

#endif