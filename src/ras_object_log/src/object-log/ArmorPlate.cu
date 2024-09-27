#include <stdio.h>
// #include "object-log/ArmorPlate.h"
#include "ArmorPlate.h"
#include "cublas_v2.h"
#include <iostream>
#include <chrono>

ArmorPlate::ArmorPlate(int id)
    : _id(id),
      _position(std::tuple<float, float, float>(0, 0, 0)),
      _velocity(std::tuple<float, float, float>(0, 0, 0)),
      _acceleration(std::tuple<float, float, float>(0, 0, 0)),
      _boundingBox(BoundingBox()),
      _isActive(true),
      _seenThisIteration(false),
      _delta_position(std::tuple<float, float, float>(0, 0, 0)),
      _lastTime(time(0)),
      _associatedBoxes(std::vector<BoundingBox>()),
      _kalmanFilter(new Kalman(0.0))
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
}

// ArmorPlate::~ArmorPlate() {
//     delete _kalmanFilter;
// }

int ArmorPlate::getId()
{
    return _id;
}

std::tuple<float, float, float> ArmorPlate::getPosition()
{
    return _position;
}

std::tuple<float, float, float> ArmorPlate::getVelocity()
{
    return _velocity;
}

std::tuple<float, float, float> ArmorPlate::getAcceleration()
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

std::tuple<float, float, float> ArmorPlate::getNextPosition()
{
    return _delta_position;
}

uint64_t ArmorPlate::getLastTime()
{
    return _lastTime;
}

BoundingBox ArmorPlate::getBoundingBox()
{
    return _boundingBox;
}

std::vector<BoundingBox> ArmorPlate::getAssociatedBoxes()
{
    return _associatedBoxes;
}

void ArmorPlate::setId(int id)
{
    ArmorPlate::_id = id;
}

void ArmorPlate::setPosition(std::tuple<float, float, float> position, uint64_t currentTime)
{
    printf("Current Time: %ld\n", currentTime);
    printf("Last Time: %ld\n", _lastTime);
    time_t deltaTime = currentTime - _lastTime;
    printf("Delta Time: %ld\n", deltaTime);
    std::tuple<float, float, float> new_velocity = std::tuple<float, float, float>((std::get<0>(position) - std::get<0>(ArmorPlate::_position) / double(deltaTime)),
                                                                                   (std::get<1>(position) - std::get<1>(ArmorPlate::_position) / double(deltaTime)),
                                                                                   (std::get<2>(position) - std::get<2>(ArmorPlate::_position) / double(deltaTime)));
    printf("Old Position: %f, %f, %f\n", std::get<0>(ArmorPlate::_position), std::get<1>(ArmorPlate::_position), std::get<2>(ArmorPlate::_position));
    printf("New Position: %f, %f, %f\n", std::get<0>(position), std::get<1>(position), std::get<2>(position));

    printf("New Velocity: %f, %f, %f\n", std::get<0>(new_velocity), std::get<1>(new_velocity), std::get<2>(new_velocity));
    std::tuple<float, float, float> new_accelaration = std::tuple<float, float, float>((std::get<0>(new_velocity) - std::get<0>(ArmorPlate::_velocity) / double(deltaTime)),
                                                                                       (std::get<1>(new_velocity) - std::get<1>(ArmorPlate::_velocity) / double(deltaTime)),
                                                                                       (std::get<2>(new_velocity) - std::get<2>(ArmorPlate::_velocity) / double(deltaTime)));
    printf("New Acceleration: %f, %f, %f\n", std::get<0>(new_accelaration), std::get<1>(new_accelaration), std::get<2>(new_accelaration));
    this->predictPosition(currentTime);
    // set position to the average of position and _next_position
    // ArmorPlate::_position = std::tuple<float, float, float>((std::get<0>(position) + std::get<0>(ArmorPlate::_delta_position)) / 2,
    //                                                         (std::get<1>(position) + std::get<1>(ArmorPlate::_delta_position)) / 2,
    //                                                         (std::get<2>(position) + std::get<2>(ArmorPlate::_delta_position)) / 2);
    
    if (deltaTime != 0)
    {
        this->setVelocity(new_velocity);
        this->setAcceleration(new_accelaration);
    }
    printf("Predicted delta Position: %f, %f, %f\n", std::get<0>(ArmorPlate::_delta_position), std::get<1>(ArmorPlate::_delta_position), std::get<2>(ArmorPlate::_delta_position));
    std::tuple<float, float, float> new_position = std::tuple<float, float, float>((std::get<0>(position) + std::get<0>(ArmorPlate::_delta_position)),
                                                                                   (std::get<1>(position) + std::get<1>(ArmorPlate::_delta_position)),
                                                                                   (std::get<2>(position) + std::get<2>(ArmorPlate::_delta_position)));
    printf("New Predicted Position: %f, %f, %f\n", std::get<0>(new_position), std::get<1>(new_position), std::get<2>(new_position));
    
    // set the position to the average of new_position and position
    ArmorPlate::_position = std::tuple<float, float, float>((std::get<0>(new_position) + std::get<0>(position)) / 2,
                                                            (std::get<1>(new_position) + std::get<1>(position)) / 2,
                                                            (std::get<2>(new_position) + std::get<2>(position)) / 2);
    printf("Final Position: %f, %f, %f\n", std::get<0>(ArmorPlate::_position), std::get<1>(ArmorPlate::_position), std::get<2>(ArmorPlate::_position));
}

void ArmorPlate::setVelocity(std::tuple<float, float, float> velocity)
{
    ArmorPlate::_velocity = velocity;
}

void ArmorPlate::setAcceleration(std::tuple<float, float, float> acceleration)
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

void ArmorPlate::setNextPosition(std::tuple<float, float, float> next_position)
{
    ArmorPlate::_delta_position = next_position;
}

void ArmorPlate::setLastTime(uint64_t lastTime)
{
    ArmorPlate::_lastTime = lastTime;
}

// void setBoundingBox(BoundingBox boundingBox) {}
// void setKalmanFilter(KalmanFilter kalmanFilter) {}

// TODO (IMP) Whenever we make a callback function for the topic that listens to the position,
// TODO we need to calculate instantaneous velocity and acceleration
// TODO make two new functions for this that are similar to the functions in Kalman

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
    float *position = (float *)malloc(3 * sizeof(float));
    float *velocity = (float *)malloc(3 * sizeof(float));
    float *acceleration = (float *)malloc(3 * sizeof(float));

    position[0] = (std::get<0>(ArmorPlate::_position));
    position[1] = (std::get<1>(ArmorPlate::_position));
    position[2] = (std::get<2>(ArmorPlate::_position));

    velocity[0] = (std::get<0>(ArmorPlate::_velocity));
    velocity[1] = (std::get<1>(ArmorPlate::_velocity));
    velocity[2] = (std::get<2>(ArmorPlate::_velocity));

    acceleration[0] = (std::get<0>(ArmorPlate::_acceleration));
    acceleration[1] = (std::get<1>(ArmorPlate::_acceleration));
    acceleration[2] = (std::get<2>(ArmorPlate::_acceleration));

    _kalmanFilter->set_state_n(position, velocity, acceleration);
    _kalmanFilter->predict_state_n_1();
    float *output = _kalmanFilter->get_state_n_1();
    _kalmanFilter->update_state_n();
    output = _kalmanFilter->get_state_n();
    printf("Kalman output: %f, %f, %f, %f, %f, %f, %f, %f, %f\n", output[0], output[1], output[2], output[3], output[4], output[5], output[6], output[7], output[8]);

    ArmorPlate::_position = std::tuple<float, float, float>(output[0], output[1], output[2]);
    ArmorPlate::_velocity = std::tuple<float, float, float>(output[3], output[4], output[5]);
    ArmorPlate::_acceleration = std::tuple<float, float, float>(output[6], output[7], output[8]);
}

/*
 * @brief Predicts the position of the armor plate at the current time
 * Uses the kalman filter to do so
 */
void ArmorPlate::predictPosition(uint64_t currentTime)
{

    // get the predicted position from kalman filter
    // set the position to the predicted value
    uint64_t timeDiff = currentTime - ArmorPlate::_lastTime;
    // kinematics ut + 0.5at^2
    std::tuple<float, float, float> deltaVel = std::tuple<float, float, float>(0, 0, 0);
    float *velocity, *acceleration, *deltaVel_h;

    // TODO we need to get velocity and acceleration from the kalman filter
    velocity = (float *)malloc(3 * sizeof(float));
    acceleration = (float *)malloc(3 * sizeof(float));
    deltaVel_h = (float *)malloc(3 * sizeof(float));

    velocity[0] = (std::get<0>(ArmorPlate::_velocity));
    velocity[1] = (std::get<1>(ArmorPlate::_velocity));
    velocity[2] = (std::get<2>(ArmorPlate::_velocity));

    acceleration[0] = (std::get<0>(ArmorPlate::_acceleration));
    acceleration[1] = (std::get<1>(ArmorPlate::_acceleration));
    acceleration[2] = (std::get<2>(ArmorPlate::_acceleration));

    deltaVel_h[0] = (std::get<0>(deltaVel));
    deltaVel_h[1] = (std::get<1>(deltaVel));
    deltaVel_h[2] = (std::get<2>(deltaVel));

    cublasHandle_t handle;
    cublasStatus_t status = cublasCreate_v2(&handle);

    float *velocity_d, *acceleration_d, *deltaVel_d;
    cudaMalloc((void **)&velocity_d, 3 * sizeof(float));
    cudaMalloc((void **)&acceleration_d, 3 * sizeof(float));
    cudaMalloc((void **)&deltaVel_d, 3 * sizeof(float));

    cublasSetVector(3, sizeof(float), velocity, 1, velocity_d, 1);
    cublasSetVector(3, sizeof(float), acceleration, 1, acceleration_d, 1);
    cublasSetVector(3, sizeof(float), deltaVel_h, 1, deltaVel_d, 1);

    float alpha = 0.5 * timeDiff * timeDiff;

    cublasSaxpy_v2(handle, 3, &alpha, acceleration_d, 1, deltaVel_d, 1);

    alpha = timeDiff;
    cublasSaxpy_v2(handle, 3, &alpha, velocity_d, 1, deltaVel_d, 1);

    cublasGetVector(3, sizeof(float), deltaVel_d, 1, deltaVel_h, 1);

    ArmorPlate::_velocity = deltaVel;
    setNextPosition(std::tuple<float, float, float>(deltaVel_h[0], deltaVel_h[1], deltaVel_h[2]));
}

/*
 * @brief Updates the state transition matrix of the Kalman Filter
 */
void ArmorPlate::setDeltaTime(float deltaTime)
{
    _kalmanFilter->setDeltaTime(deltaTime);
}

int experimentCUDA(int n)
{
    int deviceCount;
    cudaError_t err = cudaGetDeviceCount(&deviceCount);
    cublasHandle_t handle;
    cublasStatus_t status = cublasCreate_v2(&handle);
    if (status != CUBLAS_STATUS_SUCCESS)
    {
        std::cout << "CUBLAS initialization failed!" << std::endl;
        return 1;
    }

    // * Vector pointers for device and host
    float *d_A, *d_B, *h_A, *h_B, *h_C;

    // * Allocate memory for host vectors
    h_A = (float *)malloc(n * sizeof(float));
    h_B = (float *)malloc(n * sizeof(float));
    h_C = (float *)malloc(n * sizeof(float));

    // * Allocate memory for device vectors
    cudaMalloc((void **)&d_A, n * sizeof(float));
    cudaMalloc((void **)&d_B, n * sizeof(float));

    // * Initialize host vectors
    for (int i = 0; i < n; i++)
    {
        h_A[i] = rand() / (float)RAND_MAX;
        h_B[i] = rand() / (float)RAND_MAX;
    }

    // * Copy host vectors to device
    cublasSetVector(n, sizeof(float), h_A, 1, d_A, 1);
    cublasSetVector(n, sizeof(float), h_B, 1, d_B, 1);

    // TODO look up cudaStreams because cudaSetVectorAsync can be used

    // * Perform vector addition
    float alpha = 1.0;

    time_t start = time(0);
    std::cout << "Starting GPU" << start << std::endl;
    cublasSaxpy_v2(handle, n, &alpha, d_A, 1, d_B, 1);
    time_t end = time(0);
    std::cout << "Finished GPU" << end << std::endl;
    std::cout << "GPU Time: " << end - start << std::endl;

    // * Copy result back to host
    cublasGetVector(n, sizeof(float), d_B, 1, h_C, 1);

    // * Perform vector addition on CPU
    start = time(0);
    std::cout << "Starting CPU" << start << std::endl;
    for (int i = 0; i < n; i++)
    {
        h_C[i] = h_A[i] + h_B[i];
    }
    end = time(0);
    std::cout << "Finished CPU" << end << std::endl;
    std::cout << "CPU Time: " << end - start << std::endl;

    // * Print result
    // for(int i = 0; i < n; i++) {
    //     std::cout << h_A[i] << " + " << h_B[i] << " = " << h_C[i] << std::endl;
    // }

    // * Free device memory
    cudaFree(d_A);
    cudaFree(d_B);

    // * Free host memory
    free(h_A);
    free(h_B);
    free(h_C);
    std::cout << "Success" << std::endl;
    cublasDestroy_v2(handle);
    return 0;
}

// int main()
// {

//     // * Size of vector
//     int n = 500000000;
//     int step = 10000;
//     for (int i = 0; i < 100; i++)
//     {
//         std::cout << "n: " << n << std::endl;
//         experimentCUDA(n);
//         n += step;
//     }
//     return 0;
// }
