
#include "ArmorPlate.h"
#include "ObjectLog.h"

ObjectLog::ObjectLog() : _plates(std::vector<ArmorPlate>()), _idAssign(0), _outputLog(fopen("ObjectLog.txt", "w")) {}

int ObjectLog::boxesInput(std::vector<BoundingBox> boxList, time_t currTime)
{
    if (boxList.empty())
    {
        return -1;
    }

    if (_plates.empty())
    {
        for (int i = 0; i < boxList.size(); i++)
        {
            box = box[i];
            if (!sizeCheck(box))
                continue;
            if ((box.get_x_value() < 0) || (box.get_y_value() < 0) || (box.get_depth() < 0) || (box.get_height() < 0) || (box.get_width() < 0))
            {
                return -1; // maybe change to just a continue?
            }
            ArmorPlate newPlate = new ArmorPlate(box, this->idAssign);
            newPlate.addArmorPlate(newPlate, currTime);
            this->plates.push_back(newPlate);
            this->idAssign++;
        }
    }
    else
    {
        for (int i = 0; i < boxList.size(); i++)
        {
            box = box[i];
            // std::cout << box << std::endl; //C++ does not have the same printing properties as Python so prints may be useless
            if (!sizeCheck(boxList[i]))
                continue;

            if ((boxList[i].get_x_value() < 0) || (boxList[i].get_y_value() < 0) || (boxList[i].get_depth() < 0) || (boxList[i].get_height() < 0) || (boxList[i].get_width() < 0))
            {
                return -1; // maybe change to just a continue?
            }
            int assoc = assign_plate(&box, this->plates);
            ArmorPlate newAP = new ArmorPlate(box, this->idAssign);

            if (assoc == -1)
            {
                if (this->plates.size() < 9)
                {
                    this->plates.push_back(newAP);
                    this->idAssign++;
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
                ArmorPlate assocPlate = this->plates[assoc];
                assocPlate.addArmorPlate(newAP, currTime);
                assocPlate.timeBuffer = 0;
                this->idAssign++;
            }
        }

        for (int i = 0; i < this->plates.size(); i++)
        {
            plate = this->plates[i];
            if (plate.timeBuffer != 0)
            {
                plate.timeBuffer++;
                if (plate.timeBuffer == kill_threshold)
                {
                    kill_plate(plate.getID()); // originally kill_plate(i) but I think that is wrong
                }
            }
        }
    }
}

// Basic check to see if a bounding box meets the basic requirements (size does matter)
bool ObjectLog::sizeCheck(BoundingBox *box)
{
    return (box->getHeight() * box->getWidth()) >= MIN_AREA;
}

int assign_plate(BoundingBox *box, std::vector<ArmorPlate> plates)
{
    if (box == NULL || plates == NULL)
        return -2;

    std::tuple<double, double, double> position = box->get_position();
    float shortest_dist = std::numeric_limits<float>::max();
    int shortest_plate = -1;

    if (((position[0] + margin_of_err) > MAX_X) || ((position[1] + margin_of_err) > MAX_Y) || ((position[2] + margin_of_err) > MAX_Z) || ((position[0] - margin_of_err) < MIN_X) || ((position[1] - margin_of_err) < MIN_Y) || ((position[2] - margin_of_err) < MIN_Z))
    {
        return -3;
    }

    for (int i = 0; i < plates.size(); i++)
    {
        double dist = get_distance(position, plates[i].getPosition());
        if (dist < shortest_dist)
        {
            shortest_plate = i;
            shortest_dist = dist;
        }
    }

    if (shortest_dist > margin_of_error)
    {
        return -1;
    }

    return shortest_plate;
}

// Distance formula (basically Pythagorean theorem in 3D space)
double det_distance(std::tuple<double, double, double> p1, std::tuple<double, double, double> p2)
{
    return sqrt(pow((std::get<0>(p1) - std::get<0>(p2)), 2) + pow((std::get<1>(p1) - std::get<1>(p2)), 2) + pow((std::get<2>(p1) - std::get<2>(p2)), 2));
}

void kill_all()
{
    for (int i = 0; i < this->plates.size(); i++)
    {
        this->plate[i].writeToHistory(this->outputLog)
    }
    this->plates.clear();
    fclose(this->outputLog) return;
}

std::vector<ArmorPlate> get_plates()
{
    return plates;
}

void kill_plate(int id)
{
    for (int i = 0; i < this->plates.size(); i++)
    {
        if (this->plates[i].getID() == id)
        {
            this->plates[i].writeToHistory(this->outputLog);
            this->plates.erase(i);
            break;
        }
    }
}
