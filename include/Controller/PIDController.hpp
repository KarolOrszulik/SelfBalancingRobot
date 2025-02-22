#pragma once

#include <Arduino.h>
#include "IController.hpp"

class PIDController : public IController
{
public:
    PIDController(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd)
    {}

    float update(float measurement) override
    {
        return 0.f;
    }

    float update(float measurement, float derivative) override
    {
        const uint32_t now = millis();
        const float dt = lastUpdate == 0 ? 0.f : (now - lastUpdate) / 1000.f;
        lastUpdate = now;

        const float error = setpoint - measurement;

        const float pOut = kp * error;

        integral += 0.5f * ki * (error + lastError) * dt;
        float minLimit = (-1.f < pOut) ? -1.f - pOut : 0.f;
        float maxLimit = (1.f > pOut) ? 1.f - pOut : 0.f;
        integral = constrain(integral, minLimit, maxLimit);
        const float iOut = integral;

        const float dOut = kd * derivative;

        lastError = error;
        return constrain(pOut + iOut + dOut, -1.f, 1.f);
    }

    void setSetpoint(float setpoint) override
    {
        this->setpoint = setpoint;
    }

    void setGains(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

private:
    float kp, ki, kd;
    float setpoint;
    float integral = 0;
    float lastError = 0;
    uint32_t lastUpdate = 0;
};