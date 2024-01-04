#include <stdio.h>
#include "ArmorPlate.h"
#include "cublas.h"


    ArmorPlate::ArmorPlate(int id) 
    : _id(id), 
    _position(std::tuple<double, double, double>(0, 0, 0)), 
    _velocity(std::tuple<double, double, double>(0, 0, 0)), 
    _acceleration(std::tuple<double, double, double>(0, 0, 0)), 
    // _boundingBox(BoundingBox()), 
    _isActive(true), 
    _seenThisIteration(false), 
    _next_position(std::tuple<double, double, double>(0, 0, 0)), 
    _lastTime(time(0))
    // _associatedBoxes(std::vector<BoundingBox>()), 
    // _kalmanFilter(new KalmanFilter())
    {
        /*
         * Initializes the armor plate
         * Rundown of the fields of the object:
         * position, holds the position of the bounding box in a x, y, z system (camera relative)
         * velocity, last velocity of the target
         * acceleration, last acceleration of the target
         * boundingbox, boundingbox object
         * id, id of the armor plate (for debugging purposes)
         * activity, boolean on the plate on if it is currently alive
         */

        // this->id = id;
        // this->position = std::tuple<double, double, double>(0, 0, 0);
        // this->velocity = std::tuple<double, double, double>(0, 0, 0);
        // this->acceleration = std::tuple<double, double, double>(0, 0, 0);
        // this->boundingBox = boundingBox;
        // this->isActive = true;
        // this->seenThisIteration = false;
        // this->next_position = std::tuple<double, double, double>(0, 0, 0);
        // this->lastTime = time(0);
        // this->associatedBoxes = std::vector<BoundingBox>();
        // this->kalmanFilter = new KalmanFilter();
    }

    int ArmorPlate::getId()
    {
        return _id;
    }

    std::tuple<double, double, double> ArmorPlate::getPosition()
    {
        return _position;
    }

    std::tuple<double, double, double> ArmorPlate::getVelocity()
    {
        return _velocity;
    }

    std::tuple<double, double, double> ArmorPlate::getAcceleration()
    {
        return _acceleration;
    }

    bool ArmorPlate::getIsActive()
    {
        return _isActive;
    }

    bool ArmorPlate::getSeenThisIteration()
    {
        return _seenThisIteration;
    }

    std::tuple<double, double, double> ArmorPlate::getNextPosition()
    {
        return _next_position;
    }

    time_t ArmorPlate::getLastTime()
    {
        return _lastTime;
    }

    // BoundingBox getBoundingBox()

    void ArmorPlate::setId(int id)
    {
        ArmorPlate::_id = id;
    }

    void ArmorPlate::setPosition(std::tuple<double, double, double> position)
    {
        ArmorPlate::_position = position;
    }

    void ArmorPlate::setVelocity(std::tuple<double, double, double> velocity)
    {
        ArmorPlate::_velocity = velocity;
    }

    void ArmorPlate::setAcceleration(std::tuple<double, double, double> acceleration)
    {
        ArmorPlate::_acceleration = acceleration;
    }

    void ArmorPlate::setIsActive(bool isActive)
    {
        ArmorPlate::_isActive = isActive;
    }

    void ArmorPlate::setSeenThisIteration(bool seenThisIteration)
    {
        ArmorPlate::_seenThisIteration = seenThisIteration;
    }

    void ArmorPlate::setNextPosition(std::tuple<double, double, double> next_position)
    {
        ArmorPlate::_next_position = next_position;
    }

    void ArmorPlate::setLastTime(time_t lastTime)
    {
        ArmorPlate::_lastTime = lastTime;
    }

    // void setBoundingBox(BoundingBox boundingBox) {}
    // void setKalmanFilter(KalmanFilter kalmanFilter) {}

    void ArmorPlate::updatePositionVelAcc()
    {
        /*
         * Updates the position, velocity, and acceleration of the armor plate
         * Uses the kalman filter to do so
         */

        // ! TODO implement this
        // get the predicted position from kalman filter
        // get predicted vel and acc from the kalman filter
        // set the position, vel, and acc to the predicted values
    }

    void ArmorPlate::predictPosition(time_t currentTime)
    {
        /*
         * Predicts the position of the armor plate at the current time
         * Uses the kalman filter to do so
         */

        // ! TODO implement this
        // get the predicted position from kalman filter
        // set the position to the predicted value
        time_t timeDiff = currentTime - ArmorPlate::_lastTime;
        // kinematics ut + 0.5at^2
        std::tuple<double, double, double> deltaVel = std::tuple<double, double, double>(0, 0, 0);
        // ? I am thinking about implementing LAPACK to do matrix multiplication but there is a curve to it imo
        // https://scicomp.stackexchange.com/questions/26395/how-to-start-using-lapack-in-c
        
        setNextPosition(getNextPosition()); // ! TODO finish this
        
    }

    int main()
    {
        return 0;
    }