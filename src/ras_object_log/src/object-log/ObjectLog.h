#ifndef _OBJECTLOG_H
#define _OBJECTLOG_H

#include <tuple>
#include <vector>
#include <ctime>
#include <time.h>
#include <cmath>
#include <limits>
#include <stdio.h>
#include <chrono>

// #include "object-log/ArmorPlate.h"
// #include "object-log/BoundingBox.h"
#include "ArmorPlate.h"
#include "BoundingBox.h"

const int MAX_X = 150;
const int MAX_Y = 150;
const int MAX_Z = 150;
const int MIN_X = -1;
const int MIN_Y = -1;
const int MIN_Z = -1;
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 720;

// minimum area allowed
const int MIN_AREA = 10;

const int KILL_THRESHOLD = 250;

const float MARGIN_OF_ERR = 4; // to be fine tuned

// const float error_margin = 0; // to be fine tuned

// const int kill_threshold = -1; // to be fine tuned

class ObjectLog
{

public:
    ObjectLog();

    /**
    Object Log:
    Purpose: System to hold current value of armor plate.
    Fields:
        - plates: array of currently active armor plates (armor plate objects)
        - id: ID associated with them
        - Timestamp: The last timestamp they have.
    Primary functions in pipeline:
        - Take in input from depth/ML and associate armor plates
        - Filter out old plates
        - Write to a log file
    */

    /* *
     * Input from depth
     * Input:
        - boxList: an array of of bounding box objects
        - timestamp: timestamp of the boundingBoxes
                     the plates unless the closest distance is greater than some margin of error
    */
    int boxesInput(std::vector<BoundingBox> boxList, uint64_t currTime);

    /*
    Checks and returns if the armor plate's area is significant enough to be targeted
    and added/associated into the list of active armor plates
    */
    bool sizeCheck(BoundingBox *box);

    /*
    Takes in an armor plate, makes sure that it is valid and not predicted to be out of
    range, and then associates it with the armor plate whose predicted position is closest
    to the position of this armor plate
    */
    int assign_plate(BoundingBox *box, std::vector<ArmorPlate> plates);

    /*
    Get_Distance returns the distance in pixels between two points
    input is two 3x1 matrices, each representing point in (x,y,z)
    Output is a float which is the distance between the two objects
    */
    float get_distance(std::tuple<float, float, float> p1, std::tuple<float, float, float> p2);

    std::vector<ArmorPlate> get_plates();

    // kill all plates and log it
    void kill_all();

    void kill_plate(int id);

    /// @brief Returns the Best Armor plate to shoot at
    /// @return a vector consisting of position (x,y,z), velocity (x, y, z), acceleration (x, y, z)
    std::vector<float> getFinalArmorPlateState();

private:
    std::vector<ArmorPlate> _plates;
    int _idAssign;
    FILE *_outputLog;
};

#endif