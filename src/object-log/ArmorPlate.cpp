#include <stdio.h>
#include "ArmorPlate.h"
class ArmorPlate
{
    /*
     * ArmorPlate class for Stampede Robomasters Team - 2023
     * Written by Tanay Garg
     *
     * Initializes an Armor Plate object that can be used to store information about a single armor plate based on CV data gathered from a
     * classifier and a predictor model.
     */

private:
    int id;
    std::tuple<double, double, double> position;
    std::tuple<double, double, double> velocity;
    std::tuple<double, double, double> acceleration;
    // BoundingBox boundingBox;
    bool isActive;
    bool seenThisIteration;
    std::tuple<double, double, double> next_position;
    time_t lastTime;
    
    // std::vector<BoundingBox> associatedBoxes;
    // KalmanFilter kalmanFilter;

public:
    ArmorPlate(int id)
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

        this->id = id;
        this->position = std::tuple<double, double, double>(0, 0, 0);
        this->velocity = std::tuple<double, double, double>(0, 0, 0);
        this->acceleration = std::tuple<double, double, double>(0, 0, 0);
        // this->boundingBox = boundingBox;
        this->isActive = true;
        this->seenThisIteration = false;
        this->next_position = std::tuple<double, double, double>(0, 0, 0);
        this->lastTime = time(0);
        // this->associatedBoxes = std::vector<BoundingBox>();
        // this->kalmanFilter = new KalmanFilter();
    }

    int getId()
    {
        return this->id;
    }

    std::tuple<double, double, double> getPosition()
    {
        return this->position;
    }

    std::tuple<double, double, double> getVelocity()
    {
        return this->velocity;
    }

    std::tuple<double, double, double> getAcceleration()
    {
        return this->acceleration;
    }

    bool getIsActive()
    {
        return this->isActive;
    }

    bool getSeenThisIteration()
    {
        return this->seenThisIteration;
    }

    std::tuple<double, double, double> getNextPosition()
    {
        return this->next_position;
    }

    time_t getLastTime()
    {
        return this->lastTime;
    }

    // BoundingBox getBoundingBox()

    void setId(int id)
    {
        this->id = id;
    }

    void setPosition(std::tuple<double, double, double> position)
    {
        this->position = position;
    }

    void setVelocity(std::tuple<double, double, double> velocity)
    {
        this->velocity = velocity;
    }

    void setAcceleration(std::tuple<double, double, double> acceleration)
    {
        this->acceleration = acceleration;
    }

    void setIsActive(bool isActive)
    {
        this->isActive = isActive;
    }

    void setSeenThisIteration(bool seenThisIteration)
    {
        this->seenThisIteration = seenThisIteration;
    }

    void setNextPosition(std::tuple<double, double, double> next_position)
    {
        this->next_position = next_position;
    }

    void setLastTime(time_t lastTime)
    {
        this->lastTime = lastTime;
    }

    // void setBoundingBox(BoundingBox boundingBox) {}
    // void setKalmanFilter(KalmanFilter kalmanFilter) {}

    void updatePositionVelAcc()
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

    void predictPosition(time_t currentTime)
    {
        /*
         * Predicts the position of the armor plate at the current time
         * Uses the kalman filter to do so
         */

        // ! TODO implement this
        // get the predicted position from kalman filter
        // set the position to the predicted value
        time_t timeDiff = currentTime - this->lastTime;
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
};