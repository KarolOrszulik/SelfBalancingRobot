#pragma once

#include "IOrientation.hpp"
#include "IMU/IIMU.hpp"

class ComplementaryFilterOrientation : public IOrientation
{
public:
    ComplementaryFilterOrientation(IIMU* imu, float alpha = 0.1)
        : imu(imu), alpha(alpha)
    {}

    void update() override
    {
        const uint32_t now = millis();
        const float dt = (lastUpdate == 0) ? (0.f) : ((now - lastUpdate) / 1000.f);
        lastUpdate = now;

        IMUData imuData = imu->getData();

        data.pitchRate = imuData.pitchRate;
        data.rollRate = imuData.rollRate;

        float accelPitch = atan2(imuData.accelY, imuData.accelZ);
        float accelRoll = atan2(-imuData.accelX, imuData.accelZ);

        data.pitch = alpha * accelPitch + (1.f - alpha) * (data.pitch + data.pitchRate * dt);
        data.roll  = alpha * accelRoll  + (1.f - alpha) * (data.roll  + data.rollRate  * dt);
    }

    OrientationData getData() override
    {
        return data;
    }

private:
    OrientationData data;
    float alpha;
    
    IIMU* imu;
    uint32_t lastUpdate = 0;
};