#ifndef ARMORPLATE_H
#define ARMORPLATE_H
#include <tuple>
#include <vector>
#include <ctime>
#include <time.h>

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

    // **** Getters ****
    int getId();
    std::tuple<double, double, double> getPosition();
    std::tuple<double, double, double> getVelocity();
    std::tuple<double, double, double> getAcceleration();
    // BoundingBox getBoundingBox();
    bool getIsActive();
    bool getSeenThisIteration();
    std::tuple<double, double, double> getNextPosition();
    time_t getLastTime();
    // std::vector<> getAssociatedBoxes();
    // KalmanFilter getKalmanFilter();

    // **** Setters ****
    void setId(int id);
    void setPosition(std::tuple<double, double, double> position);
    void setVelocity(std::tuple<double, double, double> velocity);
    void setAcceleration(std::tuple<double, double, double> acceleration);
    void setIsActive(bool isActive);
    void setSeenThisIteration(bool seenThisIteration);
    void setNextPosition(std::tuple<double, double, double> next_position);
    void setLastTime(time_t lastTime);
    // void setAssociatedBoxes(std::vector<> associatedBoxes);
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
    void predictPosition(time_t currentTime);

    // ! TOOO update_box takes in a boundingBox and updates all the status variables involved
    // void updateBox(BoundingBox boundingBox, time_t currentTime);

private:
    int _id;
    std::tuple<double, double, double> _position;
    std::tuple<double, double, double> _velocity;
    std::tuple<double, double, double> _acceleration;
    // BoundingBox boundingBox;
    bool _isActive;
    bool _seenThisIteration;
    std::tuple<double, double, double> _next_position;
    time_t _lastTime;

    // std::vector<BoundingBox> _associatedBoxes;
    // KalmanFilter _kalmanFilter;
};
#endif