#ifndef POSE_PACKET_HPP_
#define POSE_PACKET_HPP_

#include <cstdint>

struct OdometryData
{
    float xPos;
    float yPos;
    float zPos;

    float chassisPitch;
    float chassisYaw;
    float chassisRoll;

    float turretPitch;
    float turretYaw;
};

#endif  
