#pragma once

#include <Arduino.h>

struct OrientationData
{
    float pitch = 0.f, roll = 0.f;
    float pitchRate = 0.f, rollRate = 0.f;
};

class IOrientation
{
public:
    virtual void update() = 0;
    virtual OrientationData getData() = 0;
};