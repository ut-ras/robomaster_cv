
#include "ArmorPlate.h"
#include "ObjectLog.h"

ObjectLog::ObjectLog() : _plates(std::vector<ArmorPlate>()), _idAssign(0), _outputLog(fopen("ObjectLog.txt", "w")) {}

// 
int ObjectLog::boxesInput(std::vector<BoundingBox> boxList, time_t currTime)
{
    if (boxList.empty())
    {
        return -1;
    }

    // for(int i = 0; i < boxList.size(); i++) {
    //     printf("box %d: (%f, %f, %f)\n", i+1,  boxList[i].getXCenter(), boxList[i].getYCenter(), boxList[i].getDepthVal());
    // }

    if (_plates.empty())
    {
        for (int i = 0; i < boxList.size(); i++)
        {
            BoundingBox box = boxList[i];
            if (!sizeCheck(&box))
                continue;
            if ((box.getXCenter() < 0) || (box.getYCenter() < 0) || (box.getDepthVal() < 0) || (box.getHeight() < 0) || (box.getWidth() < 0))
            {
                return -1; // maybe change to just a continue?
            }
            ArmorPlate *newPlate = new ArmorPlate(_idAssign);
            newPlate->setLastTime(currTime);
            newPlate->setPosition(box.getPosition());
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
            if (!sizeCheck(&box))
                continue;

            if ((box.getXCenter() < 0) || (box.getYCenter() < 0) || (box.getDepthVal() < 0) || (box.getHeight() < 0) || (box.getWidth() < 0))
            {
                return -1;
            }
            int assoc = assign_plate(&box, _plates);
            // printf("ASSOC %d\n", assoc);
            if (assoc == -1)
            {
                ArmorPlate *newAP = new ArmorPlate(_idAssign);
                newAP->setLastTime(currTime);
                newAP->setPosition(std::tuple<float, float, float>(box.getXCenter(), box.getYCenter(), box.getDepthVal()));
                if (_plates.size() < 9)
                {
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
                _plates[assoc].setLastTime(currTime);
                _plates[assoc].setIsActive(true);
                _plates[assoc].setPosition(box.getPosition());
                // _idAssign++;
            }
        }

        for (int i = 0; i < _plates.size(); i++)
        {
            ArmorPlate plate = _plates[i];
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
    for(int i = 0; i < _plates.size(); i++) {
        std::tuple<float, float, float> pos = _plates[i].getPosition();
        // printf("plate %d: (%f, %f, %f)\n", _plates[i].getId(), std::get<0>(pos), std::get<1>(pos), std::get<2>(pos));
    }
    return 0;
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
    // printf("BOX: (%f, %f, %f)\n", std::get<0>(position), std::get<1>(position), std::get<2>(position));
    float shortest_dist = std::numeric_limits<float>::max();
    int shortest_plate = -1;
    if (((std::get<0>(position) + MARGIN_OF_ERR) > MAX_X) || ((std::get<1>(position) + MARGIN_OF_ERR) > MAX_Y) || ((std::get<2>(position) + MARGIN_OF_ERR) > MAX_Z) 
            || ((std::get<0>(position) - MARGIN_OF_ERR) < MIN_X) || ((std::get<1>(position) - MARGIN_OF_ERR) < MIN_Y) || ((std::get<2>(position) - MARGIN_OF_ERR) < MIN_Z))
    {
        return -3;
    }

    for (int i = 0; i < plates.size(); i++)
    {
        float dist = get_distance(position, plates[i].getPosition());
        if (dist < shortest_dist)
        {   
            shortest_plate = i;
            shortest_dist = dist;
            // printf("shortest distance: %f\n", shortest_dist);
        }
    }

    float full_mog = sqrt(3 * pow(MARGIN_OF_ERR, 2)); // full mog represents the margin of error extended to 3d space
    if (shortest_dist > full_mog)
    {
        // printf("RETURN -1, %f %f\n", shortest_dist, full_mog);
        return -1;
    }
    // printf("SHORTEST PLATE: %d, SHORTEST DIST: %f\n", shortest_plate, shortest_dist);
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