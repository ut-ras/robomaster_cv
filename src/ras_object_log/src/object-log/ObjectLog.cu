
// #include "object-log/ArmorPlate.h"
// #include "object-log/ObjectLog.h"
#include "ArmorPlate.h"
#include "ObjectLog.h"
#include <unistd.h>
#include <chrono>
#include <memory>

ObjectLog::ObjectLog() : _plates(std::vector<ArmorPlate>()), _idAssign(0), _outputLog(fopen("ObjectLog.txt", "w")) {}

//
int ObjectLog::boxesInput(std::vector<BoundingBox> boxList, uint64_t currTime)
{
    if (boxList.empty())
    {
        return -1;
    }

    // for(int i = 0; i < boxList.size(); i++) {
    //     printf("box %d: (%f, %f, %f)\n", i+1,  boxList[i].getXCenter(), boxList[i].getYCenter(), boxList[i].getDepthVal());
    // }

    // ! Remove later *******
    for (int i = 0; i < boxList.size(); i++)
    {
        printf("box %d: (%f, %f, %f)\n", i + 1, boxList[i].getXCenter(), boxList[i].getYCenter(), boxList[i].getDepthVal());
    }

    if (_plates.empty())
    {
        for (int i = 0; i < boxList.size(); i++)
        {
            BoundingBox box = boxList[i];
            if (!sizeCheck(&box)) {
                printf("Failed size check");
                continue;
            }
            if ((box.getXCenter() < 0) || (box.getYCenter() < 0) || (box.getDepthVal() < 0) || (box.getHeight() < 0) || (box.getWidth() < 0))
            {
                printf("Negative values");
                continue;
            }
            auto newPlate = std::shared_ptr<ArmorPlate>(new ArmorPlate(_idAssign));
            newPlate->setLastTime(currTime);
            newPlate->setPosition(box.getPosition(), currTime);
            printf("box position: (%f, %f, %f)\n", std::get<0>(box.getPosition()), std::get<1>(box.getPosition()), std::get<2>(box.getPosition()));
            printf("newPlate position: (%f, %f, %f)\n", std::get<0>(newPlate->getPosition()), std::get<1>(newPlate->getPosition()), std::get<2>(newPlate->getPosition()));
            // newPlate.addArmorPlate(newPlate, currTime);
            _plates.push_back(*newPlate);
            _idAssign++;
        }
    }
    else
    {
        for (int i = 0; i < boxList.size(); i++)
        {
            BoundingBox box = boxList[i];
            printf("Processing box: (%f, %f, %f)\n", box.getXCenter(), box.getYCenter(), box.getDepthVal());
            if (!sizeCheck(&box))
                continue;

            if ((box.getXCenter() < 0) || (box.getYCenter() < 0) || (box.getDepthVal() < 0) || (box.getHeight() < 0) || (box.getWidth() < 0))
            {
                return -1;
            }
            int assoc = assign_plate(&box, _plates);
            printf("ASSOC %d\n", assoc);
            if (assoc == -1)
            {
                if (_plates.size() < 9)
                {
                    auto newAP = std::shared_ptr<ArmorPlate>(new ArmorPlate(_idAssign));
                    newAP->setLastTime(currTime);
                    newAP->setPosition(std::tuple<float, float, float>(box.getXCenter(), box.getYCenter(), box.getDepthVal()), currTime);
                    printf("New Armor Plate position: (%f, %f, %f)\n", std::get<0>(newAP->getPosition()), std::get<1>(newAP->getPosition()), std::get<2>(newAP->getPosition()));
                    _plates.push_back(*newAP);
                    _idAssign++;
                }
                else
                {
                    std::cout << "need space" << std::endl;
                }
            }
            else if (assoc == -2)
            {
                std::cout << "panic" << std::endl;
            }
            else if (assoc == -3)
            {
                std::cout << "out of range" << std::endl;
            }
            else
            {
                // We made an association
                printf("Previous plate velocity: (%f, %f, %f)\n", std::get<0>(_plates[assoc].getVelocity()), std::get<1>(_plates[assoc].getVelocity()), std::get<2>(_plates[assoc].getVelocity()));
                printf("Previous plate position: (%f, %f, %f)\n", std::get<0>(_plates[assoc].getPosition()), std::get<1>(_plates[assoc].getPosition()), std::get<2>(_plates[assoc].getPosition()));
                printf("Previous plate acceleration: (%f, %f, %f)\n", std::get<0>(_plates[assoc].getAcceleration()), std::get<1>(_plates[assoc].getAcceleration()), std::get<2>(_plates[assoc].getAcceleration()));
                printf("Previous plate last time: %ld\n", _plates[assoc].getLastTime());

                _plates[assoc].setIsActive(true);
                _plates[assoc].updatePositionVelAcc();
                _plates[assoc].setPosition(box.getPosition(), currTime);
                _plates[assoc].setLastTime(currTime);
                printf("ASSOC %d\n", assoc);
                printf("POS: (%f, %f, %f)\n", std::get<0>(_plates[assoc].getPosition()), std::get<1>(_plates[assoc].getPosition()), std::get<2>(_plates[assoc].getPosition()));

                // _idAssign++;
            }
        }

        for (int i = 0; i < _plates.size(); i++)
        {
            ArmorPlate plate = _plates[i];
            printf("Current time: %ld, plate.getLastTime(): %ld\n", currTime, plate.getLastTime());
            if (currTime - plate.getLastTime() > KILL_THRESHOLD)
            {
                kill_plate(plate.getId());
                // plate.timeBuffer++;
                // if (plate.timeBuffer == kill_threshold)
                // {
                //     kill_plate(plate.getID()); // originally kill_plate(i) but I think that is wrong
                // }
            }
        }
    }
    // for(int i = 0; i < _plates.size(); i++) {
    //     std::tuple<float, float, float> pos = _plates[i].getPosition();
    // printf("plate %d: (%f, %f, %f)\n", _plates[i].getId(), std::get<0>(pos), std::get<1>(pos), std::get<2>(pos));
    // }
    return 0;
}

// a function to decide which Armor Plate to shoot at
std::vector<float> ObjectLog::getFinalArmorPlateState()
{
    int center_x = FRAME_WIDTH / 2;
    int center_y = FRAME_HEIGHT / 2;

    float distance = std::numeric_limits<float>::max();
    int best_index = -1;
    std::cout << "_plate.size(): " << _plates.size() << std::endl;
    for (size_t i = 0; i < _plates.size(); i++)
    {
        float plate_distance = get_distance(_plates[i].getPosition(), std::tuple<float, float, float>(center_x, center_y, std::get<2>(_plates[i].getPosition())));
        if (plate_distance < distance)
        {
            best_index = i;
        }
    }

    // get the position velocity and accelaration from the plate
    // pack it into a float vector
    std::vector<float> plate_state;
    if (best_index >= 0) {
        plate_state.push_back(std::get<0>(_plates[best_index].getPosition()));
        plate_state.push_back(std::get<1>(_plates[best_index].getPosition()));
        plate_state.push_back(std::get<2>(_plates[best_index].getPosition()));
        plate_state.push_back(std::get<0>(_plates[best_index].getVelocity()));
        plate_state.push_back(std::get<1>(_plates[best_index].getVelocity()));
        plate_state.push_back(std::get<2>(_plates[best_index].getVelocity()));
        plate_state.push_back(std::get<0>(_plates[best_index].getAcceleration()));
        plate_state.push_back(std::get<1>(_plates[best_index].getAcceleration()));
        plate_state.push_back(std::get<2>(_plates[best_index].getAcceleration()));
    }
    return plate_state;
}

// Basic check to see if a bounding box meets the basic requirements (size does matter)
bool ObjectLog::sizeCheck(BoundingBox *box)
{
    return (box->getHeight() * box->getWidth()) >= MIN_AREA;
}

int ObjectLog::assign_plate(BoundingBox *box, std::vector<ArmorPlate> plates)
{
    if (box == NULL || &plates == NULL)
        return -2;

    std::tuple<float, float, float> position = box->getPosition();
    printf("BOX: (%f, %f, %f)\n", std::get<0>(position), std::get<1>(position), std::get<2>(position));
    float shortest_dist = std::numeric_limits<float>::max();
    int shortest_plate = -1;
    printf("Condition 1: %d\n", ((std::get<0>(position) + MARGIN_OF_ERR) > MAX_X));
    printf("Condition 2: %d\n", ((std::get<1>(position) + MARGIN_OF_ERR) > MAX_Y));
    printf("Condition 3: %d\n", ((std::get<2>(position) + MARGIN_OF_ERR) > MAX_Z));
    printf("Condition 4: %d\n", ((std::get<0>(position) - MARGIN_OF_ERR) < MIN_X));
    printf("Condition 5: %d\n", ((std::get<1>(position) - MARGIN_OF_ERR) < MIN_Y));
    printf("Condition 6: %d\n", ((std::get<2>(position) - MARGIN_OF_ERR) < MIN_Z));
    printf("Min check X: %f\n", (std::get<0>(position) - MARGIN_OF_ERR));
    printf("Min check Y: %f\n", (std::get<1>(position) - MARGIN_OF_ERR));
    printf("Min check Z: %f\n", (std::get<2>(position) - MARGIN_OF_ERR));
    // if (((std::get<0>(position) + MARGIN_OF_ERR) > MAX_X) || ((std::get<1>(position) + MARGIN_OF_ERR) > MAX_Y) || ((std::get<2>(position) + MARGIN_OF_ERR) > MAX_Z) || ((std::get<0>(position) - MARGIN_OF_ERR) < MIN_X) || ((std::get<1>(position) - MARGIN_OF_ERR) < MIN_Y) || ((std::get<2>(position) - MARGIN_OF_ERR) < MIN_Z))
    // {
    //     return -3;
    // }

    for (int i = 0; i < plates.size(); i++)
    {
        float dist = get_distance(position, plates[i].getPosition());
        printf("Plate %d: (%f, %f, %f)\n", i, std::get<0>(plates[i].getPosition()), std::get<1>(plates[i].getPosition()), std::get<2>(plates[i].getPosition()));
        printf("Position: (%f, %f, %f)\n", std::get<0>(position), std::get<1>(position), std::get<2>(position));
        printf("Distance: %f\n", dist);
        if (dist < shortest_dist)
        {
            shortest_plate = i;
            shortest_dist = dist;
        }
    }

    float full_mog = sqrt(3 * pow(MARGIN_OF_ERR, 2)); // full mog represents the margin of error extended to 3d space
    printf("full mog: %f\n", full_mog);
    printf("shortest dist: %f\n", shortest_dist);
    if (shortest_dist > full_mog)
    {
        return -1;
    }
    return shortest_plate;
}

void ObjectLog::kill_all()
{
    // for (int i = 0; i < _plates.size(); i++)
    // {
    //     _plates[i].writeToHistory(_outputLog)
    // }
    _plates.clear();
    fclose(_outputLog);
    return;
}

std::vector<ArmorPlate> ObjectLog::get_plates()
{
    return _plates;
}

void ObjectLog::kill_plate(int id)
{
    for (int i = 0; i < _plates.size(); i++)
    {
        if (_plates[i].getId() == id)
        {
            // _plates[i].writeToHistory(_outputLog);
            // delete &(_plates[i]);
            _plates.erase(_plates.begin() + i);
            break;
        }
    }
}

// Distance formula (basically Pythagorean theorem in 3D space)
float ObjectLog::get_distance(std::tuple<float, float, float> p1, std::tuple<float, float, float> p2)
{
    return sqrt(pow((std::get<0>(p1) - std::get<0>(p2)), 2) + pow((std::get<1>(p1) - std::get<1>(p2)), 2) + pow((std::get<2>(p1) - std::get<2>(p2)), 2));
}

// int main()
// {
//     std::vector<BoundingBox> boxList;
//     BoundingBox box = BoundingBox();
//     float x = 50.0;
//     float y = 50.0;
//     float z = 50.0;
//     ObjectLog *log = new ObjectLog();
//     for (int i = 0; i < 10; i++)
//     {
//         box.setXCenter(x);
//         box.setYCenter(y);
//         box.setDepthVal(z);
//         box.setWidth(10.0);
//         box.setHeight(10.0);
//         boxList.push_back(box);
//         time_t currTime;
//         time(&currTime);
//         log->boxesInput(boxList, double(currTime));
//         printf("Time: %ld\n", currTime);
//         std::vector<float> plate_state = log->getFinalArmorPlateState();
//         printf("target: \n");
//         for (int i = 0; i < plate_state.size(); i++)
//         {
//             printf("%f\n", plate_state[i]);
//         }

//         // sleep for 0.01 seconds
//         x += 0.02;
//         y += 0.02;
//         z += 0.01;
//         boxList = std::vector<BoundingBox>();
//         sleep(1);
//     }
//     return 0;
// }