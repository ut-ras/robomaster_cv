#include "BoundingBox.h"

BoundingBox::BoundingBox(double xCenter, double yCenter, double depthVal, double width, double height)
{
    _xCenter = xCenter;
    _yCenter = yCenter;
    _depthVal = depthVal;
    _width = width;
    _height = height;
    _position = std::tuple<double, double, double>((xCenter - (width/2)), yCenter - (height/2), depthVal);
    _timeStamp = time(0);
    
}

BoundingBox::BoundingBox(){
    _xCenter = 0;
    _yCenter = 0;
    _depthVal = 0;
    _width = 0;
    _height = 0;
    _position = std::tuple<double, double, double>(0, 0, 0);
    _timeStamp = time(0);
}

void BoundingBox::setXCenter(double xCenter){
    _xCenter = xCenter;
}

void BoundingBox::setYCenter(double yCenter){
    _yCenter = yCenter;
}

void BoundingBox::setDepthVal(double depthVal){
    _depthVal = depthVal;
}

void BoundingBox::setWidth(double width){
    _width = width;
}

void BoundingBox::setHeight(double height){
    _height = height;
}

void BoundingBox::setPosition(std::tuple<double, double, double> position){
    _position = position;
}

void BoundingBox::setTimeStamp(time_t timeStamp){
    _timeStamp = timeStamp;
}

double BoundingBox::getXCenter(){
    return _xCenter;
}

double BoundingBox::getYCenter(){
    return _yCenter;
}

double BoundingBox::getDepthVal(){
    return _depthVal;
}

double BoundingBox::getWidth(){
    return _width;
}

double BoundingBox::getHeight(){
    return _height;
}

std::tuple<double, double, double> BoundingBox::getPosition(){
    return _position;
}

time_t BoundingBox::getTimeStamp(){
    return _timeStamp;
}

int BoundingBox::main(){
    return 0;
}