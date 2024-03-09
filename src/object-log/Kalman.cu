#include "Kalman.h"
#include <stdio.h>
#include "cublas_v2.h"
#include <stdio.h>
#include <iostream>

void storeColumnMajor(float src[][VECTOR_SIZE], float *dest) {
    float *dest_copy = dest;
    for(int i = 0; i < VECTOR_SIZE; i++){
        // columns
        for(int j = 0; j < VECTOR_SIZE; j++) {
            // rows
            *dest_copy++ = src[j][i]; 
        }
    }
}
Kalman::Kalman(float deltaTime)
{

    _error = cudaGetDeviceCount(&_device_count);
    _status = cublasCreate_v2(&_handle);
    if (_status != CUBLAS_STATUS_SUCCESS)
    {
        std::cout << "CUBLAS initialization failed!" << std::endl;
        return;
    }

    float time_factor = 0.5f * deltaTime * deltaTime;
    float _state_transition_matrix_init[][VECTOR_SIZE] = {
        {1.0, 0.0, 0.0, deltaTime, 0.0, 0.0, time_factor, 0, 0},
        {0.0, 1.0, 0.0, 0.0, deltaTime, 0.0, 0.0, time_factor, 0},
        {0.0, 0.0, 1.0, 0.0, 0.0, deltaTime, 0.0, 0.0, time_factor},
        {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, deltaTime, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, deltaTime, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, deltaTime},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
    };
    _state_transition_matrix = (float *) malloc(VECTOR_SIZE * VECTOR_SIZE * sizeof(float*));
    storeColumnMajor(_state_transition_matrix_init, _state_transition_matrix);

    // for(int i = 0; i < VECTOR_SIZE * VECTOR_SIZE; i++) {
    //     std::cout<<_state_transition_matrix[i]<<" "<<std::endl;
    // }
};

void Kalman::updateStateTransitionMatrix(float *matrix, float deltaTime)
{
    float _state_transition_matrix_init[][VECTOR_SIZE] = {
        {1.0, 0.0, 0.0, deltaTime, 0.0, 0.0, TIME_FACTOR, 0, 0},
        {0.0, 1.0, 0.0, 0.0, deltaTime, 0.0, 0.0, TIME_FACTOR, 0},
        {0.0, 0.0, 1.0, 0.0, 0.0, deltaTime, 0.0, 0.0, TIME_FACTOR},
        {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, deltaTime, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, deltaTime, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, deltaTime},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
    };
    
    storeColumnMajor(_state_transition_matrix_init, matrix);

}

int main()
{
    Kalman *k = new Kalman(1.2);
}
