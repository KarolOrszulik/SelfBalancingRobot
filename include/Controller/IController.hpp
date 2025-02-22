#pragma once

class IController
{
public:
    virtual float update(float measurement) = 0;
    virtual float update(float measurement, float derivative) = 0;
    virtual void setSetpoint(float setpoint) = 0;
};