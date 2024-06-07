// #include "object-log/Kalman.h"
#include "Kalman.h"
#include <stdio.h>
#include "cublas_v2.h"
#include <stdio.h>
#include <iostream>
#include <time.h>

/*
 * @param src: 2D array to be stored in column major format
 * @param dest: pointer to the destination array
 * @return void
 */
void Kalman::storeColumnMajor(float src[][VECTOR_SIZE], float *dest)
{
    float *dest_copy = dest;
    for (int i = 0; i < VECTOR_SIZE; i++)
    {
        // columns
        for (int j = 0; j < VECTOR_SIZE; j++)
        {
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

    _state_n_1 = (float *)calloc(sizeof(float *), VECTOR_SIZE);
    _state_n = (float *)calloc(sizeof(float *), VECTOR_SIZE);

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
    _state_transition_matrix = (float *)malloc(VECTOR_SIZE * VECTOR_SIZE * sizeof(float *));
    storeColumnMajor(_state_transition_matrix_init, _state_transition_matrix);
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

    // stores the newly initialized matrix in column major format
    storeColumnMajor(_state_transition_matrix_init, matrix);
}

void Kalman::set_state_n(float *position, float *velocity, float *acceleration)
{
    memcpy(_state_n, position, STATE_SIZE * sizeof(float));
    memcpy(_state_n + STATE_SIZE, velocity, STATE_SIZE * sizeof(float));
    memcpy(_state_n + 2 * STATE_SIZE, acceleration, STATE_SIZE * sizeof(float));
}

void Kalman::predict_state_n_1()
{
    // device pointers for all the vectors and matrices
    float *d_state_transition_matrix, *d_state_n, *d_state_n_1;

    cudaMalloc((void **)&d_state_transition_matrix, VECTOR_SIZE * VECTOR_SIZE * sizeof(float));
    cudaMalloc((void **)&d_state_n, VECTOR_SIZE * sizeof(float));
    cudaMalloc((void **)&d_state_n_1, VECTOR_SIZE * sizeof(float));

    cublasStatus_t copy_status;
    copy_status = cublasSetMatrix(VECTOR_SIZE, VECTOR_SIZE, sizeof(float), _state_transition_matrix, VECTOR_SIZE, d_state_transition_matrix, VECTOR_SIZE);
    if (copy_status != CUBLAS_STATUS_SUCCESS)
    {
        std::cout << "Error copying state transition matrix to device" << std::endl;
        return;
    }

    copy_status = cublasSetVector(VECTOR_SIZE, sizeof(float), _state_n, 1, d_state_n, 1);
    if (copy_status != CUBLAS_STATUS_SUCCESS)
    {
        std::cout << "Error copying state_n to device" << std::endl;
        return;
    }

    // perform matrix multiplication
    float alpha = 1.0f;
    float beta = 0.0f;
    cublasSgemv(_handle, CUBLAS_OP_N, VECTOR_SIZE, VECTOR_SIZE, &alpha, d_state_transition_matrix, VECTOR_SIZE, d_state_n, 1, &beta, d_state_n_1, 1);

    // copy the result back to host
    copy_status = cublasGetVector(VECTOR_SIZE, sizeof(float), d_state_n_1, 1, _state_n_1, 1);
    if (copy_status != CUBLAS_STATUS_SUCCESS)
    {
        std::cout << "Error copying state_n_1 to host" << std::endl;
        return;
    }
}

/*
 * @brief Updates the state transition matrix based on deltaTime
*/
void Kalman::setDeltaTime(float deltaTime) {
    updateStateTransitionMatrix(_state_transition_matrix, deltaTime);
}

float *Kalman::get_state_n_1()
{
    return _state_n_1;
}

void Kalman::update_state_n()
{
    memcpy(_state_n, _state_n_1, VECTOR_SIZE * sizeof(float));
}

float *Kalman::get_state_n()
{
    return _state_n;
}

void Kalman::set_velocity(float *velocity)
{
    memcpy(_state_n + STATE_SIZE, velocity, STATE_SIZE * sizeof(float));
}

void Kalman::set_acceleration(float *acceleration)
{
    memcpy(_state_n + 2 * STATE_SIZE, acceleration, STATE_SIZE * sizeof(float));
}

// int main()
// {
//     /*
//      * TESTING
//      */
//     float DELTA_TIME = 1.2;
//     Kalman *k = new Kalman(DELTA_TIME);

//     float position[] = {1.0, 2.0, 3.0};
//     float velocity[] = {4.0, 5.0, 6.0};
//     float acceleration[] = {7.0, 8.0, 9.0};

//     time_t start = time(0);
//     std::cout << "Starting time: " << start << std::endl;
//     for (int i = 0; i < 20; i++)
//     {
//         k->set_state_n(position, velocity, acceleration);
//         k->predict_state_n_1();
//         float *output = k->get_state_n_1();
//         k->update_state_n();
//         output = k->get_state_n();
//         for (int i = 0; i < VECTOR_SIZE; i++)
//         {
//             std::cout << output[i] << std::endl;
//         }
//         position[0] = output[0];
//         position[1] = output[1];
//         position[2] = output[2];
//         velocity[0] = output[3];
//         velocity[1] = output[4];
//         velocity[2] = output[5];
//         acceleration[0] = output[6];
//         acceleration[1] = output[7];
//         acceleration[2] = output[8];
//         std::cout<<"----------iteration ended---------\n";
//     }
//     time_t end = time(0);
//     std::cout << "Ending time: " << end << std::endl;
//     std::cout << "Time taken: " << end - start << std::endl;
//     return 0;
// }
