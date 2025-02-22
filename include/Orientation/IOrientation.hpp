#pragma once

#include <Arduino.h>

struct OrientationData
{
    float pitch, roll;
    float pitchRate, rollRate;
};

class IOrientation
{
public:
    virtual void update() = 0;
    virtual OrientationData getData() = 0;
};