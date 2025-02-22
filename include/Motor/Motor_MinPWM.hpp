#pragma once

#include <Arduino.h>

#include "IMotor.hpp"

class Motor_MinPWM : public IMotor
{
public:
    Motor_MinPWM(IMotor* motor, int minPWM)
        : motor(motor), minPWM(minPWM)
    {
    }

    void stop() override
    {
        motor->stop();
    }

    void idle() override
    {
        motor->idle();
    }

    void setMinPWM(int minPWM)
    {
        this->minPWM = minPWM;
    }

private:
    void forward(uint8_t speed) override
    {
        motor->setSpeed(remapSpeed(speed));
    }

    void backward(uint8_t speed) override
    {
        motor->setSpeed(-remapSpeed(speed));
    }

    int remapSpeed(int speed)
    {
        return map(speed, 0, 255, minPWM, 255);
    }

    IMotor* motor;
    int minPWM;
};