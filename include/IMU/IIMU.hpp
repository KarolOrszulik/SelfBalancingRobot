#pragma once

struct IMUData
{
    float accelX = 0.f, accelY = 0.f, accelZ = 0.f;
    float pitchRate = 0.f, rollRate = 0.f, yawRate = 0.f;
};

class IIMU
{
public:
    virtual void update() = 0;
    virtual IMUData getData() = 0;
};