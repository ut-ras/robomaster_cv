// #include "object-log/BoundingBox.h"
#include "BoundingBox.h"

BoundingBox::BoundingBox(float xCenter, float yCenter, float depthVal, float width, float height)
{
    _xCenter = xCenter;
    _yCenter = yCenter;
    _depthVal = depthVal;
    _width = width;
    _height = height;
    _position = std::tuple<float, float, float>((xCenter - (width/2)), yCenter - (height/2), depthVal);
    _timeStamp = time(0);
    
}

BoundingBox::BoundingBox(){
    _xCenter = 0;
    _yCenter = 0;
    _depthVal = 0;
    _width = 0;
    _height = 0;
    _position = std::tuple<float, float, float>(0, 0, 0);
    _timeStamp = time(0);
}

void BoundingBox::setXCenter(float xCenter){
    _xCenter = xCenter;
}

void BoundingBox::setYCenter(float yCenter){
    _yCenter = yCenter;
}

void BoundingBox::setDepthVal(float depthVal){
    _depthVal = depthVal;
}

void BoundingBox::setWidth(float width){
    _width = width;
}

void BoundingBox::setHeight(float height){
    _height = height;
}

void BoundingBox::setPosition(std::tuple<float, float, float> position){
    _position = position;
}

void BoundingBox::setTimeStamp(time_t timeStamp){
    _timeStamp = timeStamp;
}

float BoundingBox::getXCenter(){
    return _xCenter;
}

float BoundingBox::getYCenter(){
    return _yCenter;
}

float BoundingBox::getDepthVal(){
    return _depthVal;
}

float BoundingBox::getWidth(){
    return _width;
}

float BoundingBox::getHeight(){
    return _height;
}

std::tuple<float, float, float> BoundingBox::getPosition(){
    std::tuple<float, float, float> position = std::make_tuple(_xCenter, _yCenter, _depthVal);
    return position;
}

time_t BoundingBox::getTimeStamp(){
    return _timeStamp;
}

// int BoundingBox::main(){
//     return 0;
// }