#pragma once

#include <Arduino.h>

class IMotor
{
public:
    virtual void setSpeed(int speed)
    {
        speed = constrain(speed, -255, 255);
        if (speed > 0)
            forward(speed);
        else if (speed < 0)
            backward(-speed);
        else
            stop();
    }
    
    virtual void stop() = 0;
    virtual void idle() = 0;
private:
    virtual void forward(uint8_t speed) = 0;
    virtual void backward(uint8_t speed) = 0;
};