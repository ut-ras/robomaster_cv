#include <stdio.h>
#include <iostream>
#include "BoundingBox.h"

class BoundingBox
{
    /*
     * Bounding Box class for Stampede Robomasters Team - 2023
     * Written by Tanay Garg
     *
     * Initializes an Armor Plate object that can be used to store information about a single armor plate based on CV data gathered from a
     * classifier and a predictor model.
     */

private:
    double x_center;
    double y_center;
    double depthValue;
    double height;
    double width;
    std::tuple<double, double, double> position;
    time_t currTime;

public:
    BoundingBox(double x_center, double y_center, double depthValue, double height)
    {
        this->x_center = x_center;
        this->y_center = y_center;
        this->depthValue = depthValue;
        this->height = height;
        this->width = width;
        this->position = std::tuple<double, double, double>(x_center, y_center, depthValue);
        this->currTime = time(0);
    }

    int main()
    {
        std::cout << "Hello, world!" << std::endl;
        return 0;
    }
};