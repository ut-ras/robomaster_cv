#ifndef KALMAN_H
#define KALMAN_H
#define IDX2C(i, j, ld) (((j) * (ld)) + (i))
#include "cublas_v2.h"
#include <tuple>

#define VECTOR_SIZE 9
#define TIME_FACTOR (0.5f * deltaTime * deltaTime)

class Kalman
{
    /*
     * ArmorPlate class for Stampede Robomasters Team - 2023
     * Written by Tanay Garg
     *
     * Encapsulates all Kalman Filter Calculations required
     * to be done for the Object Log
     * 
     * ALL MATRICES ARE STORED IN A COLUMN MAJOR FORMAT FOR CUBAS GEMMV OPERATIONS
     * 
     */

private:
    int _device_count;
    cudaError_t _error;
    cublasHandle_t _handle;
    cublasStatus_t _status;

    float **_state_n;
    float **_state_n_1;
    float *_state_transition_matrix;

    
public:
    Kalman(float deltaTime);
    void updateStateTransitionMatrix(float *matrix, float deltaTime);
    void kinematicPredict(float deltaTime);
    std::tuple<float, float, float, float, float, float> getVA();
    std::tuple<float, float, float> getPos();
    void kinematicUpdate(std::tuple<float, float, float> position);
};

#endif