#ifndef ARMORPLATE_H
#define ARMORPLATE_H
#define IDX2C(i,j,ld) (((j)*(ld))+(i)) 
#include <tuple>
#include <vector>
#include <ctime>
#include <time.h>
#include <chrono>
// #include "object-log/BoundingBox.h"
// #include "object-log/Kalman.h"
#include "BoundingBox.h"
#include "Kalman.h"


class ArmorPlate
{
    /*
     * ArmorPlate class for Stampede Robomasters Team - 2023
     * Written by Tanay Garg
     *
     * Initializes an Armor Plate object that can be used to store information about a single armor plate based on CV data gathered from a
     * classifier and a predictor model.
     */
public:
    // **** Constructors ****
    ArmorPlate(int id);
    // ~ArmorPlate();

    // **** Getters ****
    int getId();
    std::tuple<float, float, float> getPosition();
    std::tuple<float, float, float> getVelocity();
    std::tuple<float, float, float> getAcceleration();
    BoundingBox getBoundingBox();
    bool getIsActive();
    bool getSeenThisIteration();
    std::tuple<float, float, float> getNextPosition();
    uint64_t getLastTime();
    std::vector<BoundingBox> getAssociatedBoxes();
    // KalmanFilter getKalmanFilter();

    // **** Setters ****
    void setId(int id);
    void setPosition(std::tuple<float, float, float> position, uint64_t currTime);
    void setVelocity(std::tuple<float, float, float> velocity);
    void setAcceleration(std::tuple<float, float, float> acceleration);
    void setIsActive(bool isActive);
    void setSeenThisIteration(bool seenThisIteration);
    void setNextPosition(std::tuple<float, float, float> next_position);
    void setLastTime(uint64_t lastTime);
    // void setAssociatedBoxes(std::vector<BoundingBox> associatedBoxes);
    // void setKalmanFilter(KalmanFilter kalmanFilter);

    /*  *  Velocity and acceleration are sets of three values.
     * Input from Armorplate
     * Input:
     *  - velocity: (x, y, z)
     *  - acceleration: (x, y, z)
     * Output:
     *  - Predicated velocity vector of armor plate
     *  - Predicated acceleration vector of armor plate
     */

    // gets position, velocity, accelaration from kalman filter and updates the private variables
    void updatePositionVelAcc();
    /*
     * predictPosition uses position, velocity, and delta time to predict where armor plate have moved to since the last check;
     * this method is subject to change depending on how PVA is implemented (currently assumed to be world positions)
     * assumes that time is being kept track of in per second units while velocity/acceleration are per millisecond units.
     */
    void predictPosition(uint64_t currentTime);

    // ! TOOO update_box takes in a boundingBox and updates all the status variables involved
    // void updateBox(BoundingBox boundingBox, time_t currentTime);

    /*
     * @brief setDeltaTime sets the delta time for the armor plate to be used in calculations 
    */
    void setDeltaTime(float deltaTime);

private:
    int _id;
    std::tuple<float, float, float> _position;
    std::tuple<float, float, float> _velocity;
    std::tuple<float, float, float> _acceleration;
    BoundingBox _boundingBox;
    bool _isActive;
    bool _seenThisIteration;
    std::tuple<float, float, float> _delta_position;
    uint64_t _lastTime;
    std::vector<BoundingBox> _associatedBoxes;
    Kalman *_kalmanFilter;
};
#endif