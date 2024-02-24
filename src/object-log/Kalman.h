#ifndef KALMAN_H
#define KALMAN_H
#include "cublas_v2.h"
class Kalman {
    /*
     * ArmorPlate class for Stampede Robomasters Team - 2023
     * Written by Tanay Garg
     *
     * Encapsulates all Kalman Filter Calculations required 
     * to be done for the Object Log
     */

    public:
    Kalman(float deltaTime);
    void kinematicPredict(float deltaTime);
    std::tuple<float, float, float, float, float, float> getVA();
    std::tuple<float, float, float> getPos();
    void kinematicUpdate(std::tuple<float, float, float> position);
    private:
    
}

#endif