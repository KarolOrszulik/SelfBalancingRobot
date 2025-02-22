#pragma once

struct IMUData
{
    float accelX, accelY, accelZ;
    float pitchRate, rollRate, yawRate;
};

class IIMU
{
public:
    virtual void update() = 0;
    virtual IMUData getData() = 0;
};