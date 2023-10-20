#include <cmath>
#include <limits>

#include "armorplate.h" 
#include "boundingbox.h"
#include "objectlog.h"

ObjectLog::ObjectLog() {
    plates = {};
    idAssign = 0;
    outputLog = fopen("ObjectLog.txt", "w");
}

ObjectLog::void boxesInput(std::vector<BoundingBox> boxList, float currTime) {
    if(boxList.empty()) {
        return -1;
    }

    if(plates.empty()) {
        for(int i = 0; i < boxList.size(); i++) {
            box = box[i];
            if(!sizeCheck(box)) continue;
            if((box.get_x_value() < 0) || (box.get_y_value() < 0) || (box.get_depth() < 0) || (box.get_height() < 0) || (box.get_width() < 0)) {
                return -1; // maybe change to just a continue?
            }
            ArmorPlate newPlate = new ArmorPlate(box, idAssign);
            newPlate.addArmorPlate(newPlate, currTime);
            plates.push_back(newPlate);
            idAssign++;
        }
    }
    else {
        for(int i = 0; i < boxList.size(); i++) {
            box = box[i];
            // std::cout << box << std::endl; //C++ does not have the same printing properties as Python so prints may be useless
            if(!sizeCheck(boxList[i])) continue;


            if((boxList[i].get_x_value() < 0) || (boxList[i].get_y_value() < 0) || (boxList[i].get_depth() < 0) || (boxList[i].get_height() < 0) || (boxList[i].get_width() < 0)) {
                return -1; // maybe change to just a continue?
            }
            int assoc = assign_plate(&box, plates);
            ArmorPlate newAP = new ArmorePlate(box, idAssign);
            if(assoc == -1) {

            }
            else if (assoc == -2) {

            }
            else if (assoc == -3) {

            }
            else {
                ArmorPlate assocPlate = plates[assoc];
                assocPlate.addArmorPlate(newAP, currTime);
                assocPlate.timeBuffer = 0;
                idAssign++;
            }
            
        }

        for(int i = 0; i < plates.size(); i++) {
            plate = plates[i];
            if(plate.timeBuffer != 0) {
                plate.timeBuffer++;
                if(plate.timeBuffer == kill_threshold) {
                    kill_plate(plate.getID()); // originally kill_plate(i) but I think that is wrong
                }
            }
        }
    }
}

// Basic check to see if a bounding box meets the basic requirements (size does matter)
ObjectLog::bool size_check(BoundingBox *box) {
    return box->get_height() * box->get_width() >= MIN_AREA;
}

ObjectLog::int assign_plate(BoundingBox *box, std::vector<ArmorPlate> plates) {
    if(box == NULL || plates == NULL) return -2;

    std::vector<int> position = box->get_position();
    float shortest_dist = std::numeric_limits<float>::max();
    int shortest_plate = -1;

    if(((position[0] + margin_of_err) > MAX_X) || ((position[1] + margin_of_err) > MAX_Y) || ((position[2] + margin_of_err) > MAX_Z) || ((position[0] - margin_of_err) < MIN_X) || ((position[1] - margin_of_err) < MIN_Y) || ((position[2] - margin_of_err) < MIN_Z)) {
        return -3;
    }

    for(int i = 0; i < plates.size(); i++) {
        dist = get_distance(position, plates[i].getPosition());
        if(dist < shortest_dist) {
            shortest_plate = i;
            shortest_dist = dist;
        }
    }

    if(shortest_dist > margin_of_error) {
        return -1;
    }

    return shortest_plate;
    
}

// Distance formula (basically Pythagorean theorem in 3D space)
ObjectLog::float det_distance(std::vector<int> p1, std::vector<int> p2) {
    return sqrt(pow((p1[0] - p2[0]), 2)
              + pow((p1[1] - p2[1]), 2)
              + pow((p1[2] - p2[2]), 2));
}

ObjectLog::void kill_all() {
    for(int i = 0; i < plates.size(); i++) {
        plate[i].writeToHistory(outputLog)
    }
    self.plates.clear();
    fclose(outputLog)
    return;
}

ObjectLog::std::vector<ArmorPlate> get_plates() {
    return plates;
}

ObjectLog::void kill_plate(int id) {
    for(int i = 0; i < plates.size(); i++) {
        if(plates[i].getID() == id) {
            plates[i].writeToHistory(outputLog);
            plates.erase(i);
            break;
        }
    }
}
