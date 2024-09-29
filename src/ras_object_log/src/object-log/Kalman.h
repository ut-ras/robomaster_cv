#ifndef KALMAN_H
#define KALMAN_H
#define IDX2C(i, j, ld) (((j) * (ld)) + (i)) 
#include "cublas_v2.h"
#include <tuple>

#define VECTOR_SIZE 9
#define STATE_SIZE 3
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
    float *_state_n;
    float *_state_n_1;
    float *_state_transition_matrix;

    /* *
     * Takes src and stores it to dest in column major format
     */
    void storeColumnMajor(float src[][VECTOR_SIZE], float *dest);

public:
    /* *
     * Initializes cuBLAS handle, state transition matrix
     */
    Kalman(float deltaTime);

    /* *
     * Updates the State Transition matrix in case the deltaTime changes (due to change in FPS, etc.)
     */
    void updateStateTransitionMatrix(float *matrix, float deltaTime);
    void kinematicPredict(float deltaTime);
    float *getVA(); 
    float *getPos();
    void kinematicUpdate(float *position);
    void setDeltaTime(float deltaTime);
    /*
     * @brief Pass in 3 3x1 vectors for position, velocity and acceleration
    */
    void set_state_n(float *position, float *velocity, float *acceleration);

    /*
     * @brief calculates the current state of the object
     */
    void predict_state_n_1();

    /*
     * @return the current state of the object
    */
    float *get_state_n_1();
    /*
     * @brief updates the state of the object
    */
    void update_state_n();

    /*
     * @return a pointer to the current state of the object
     */
    float *get_state_n();

    /*
     * Sets the velocity of the state_n variable because we can't calculate it using 
     * the state transition matrix
    */
    void set_velocity(float *velocity);

    /*
     * Sets the acceleration of the state_n variable because we can't calculate it using
     * the state transition matrix
    */
    void set_acceleration(float *acceleration);
};

#endif