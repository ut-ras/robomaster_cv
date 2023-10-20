#ifndef _OBJECTLOG_H
#define _OBJECTLOG_H

#include <tuple>
#include <vector>
#include <ctime>
#include <time.h>

#include "BoundingBox.h" // name may be subject to change
#include "ArmorPlate.h" 

const int MAX_X = 150;
const int MAX_Y = 150;
const int MAX_Z = 150;
const int MIN_X = -1;
const int MIN_Y = -1;
const int MIN_Z = -1;

// minimum area allowed
const MIN_AREA = 10;

error_margin = 0; // to be fine tuned

kill_threshold = -1; // to be fine tuned

class ObjectLog {
    
    public:
        ObjectLog();

        /*
        Input from depth module
        Input: 
        - boxList: an array of of bounding box objects
        - timestamp: timestamp of the boundingBoxes
        the plates unless the closest distance is greater than some margin of error
        returns -1 if failure and 0 if success
        */
        int boxes_input(std::vector<BoundingBox> boxList, time_t currTime);

        /*
        Checks and returns if the armor plate's area is significant enough to be targeted
        and added/associated into the list of active armor plates
        */
        bool size_check(BoundingBox *box);

        /*
        Takes in an armor plate, makes sure that it is valid and not predicted to be out of
        range, and then associates it with the armor plate whose predicted position is closest
        to the position of this armor plate
        */
        int assign_plate(BoundingBox *box, std::vector<ArmorPlate> plates);

        /*
        Get_Distance returns the distance in pixels between two points
        input is two 3x1 matrices, each representing point in (x,y,z)
        Output is a double which is the distance between the two objects 
        */
        double get_distance(std::tuple<double, double, double> p1, std::tuple<double, double, double> p2);

        //kill all plates and 
        void kill_all();

        std::vector<ArmorPlate> get_plates();

        void kill_plate(int id);
};

#endif