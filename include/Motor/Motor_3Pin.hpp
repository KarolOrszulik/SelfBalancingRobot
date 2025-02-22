#pragma once

#include "Imotor.hpp"

class Motor_3Pin : public IMotor
{
public:
    Motor_3Pin(int pinA, int pinB, int pinEn)
        : pinA(pinA), pinB(pinB), pinEn(pinEn)
    {
        pinMode(pinA, OUTPUT);
        pinMode(pinB, OUTPUT);
        pinMode(pinEn, OUTPUT);
    }

    void forward(uint8_t speed) override
    {
        analogWrite(pinEn, 0);
        digitalWrite(pinA, HIGH);
        digitalWrite(pinB, LOW);
        analogWrite(pinEn, speed);
    }

    void backward(uint8_t speed) override
    {
        analogWrite(pinEn, 0);
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, HIGH);
        analogWrite(pinEn, speed);
    }

    void stop() override
    {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
        analogWrite(pinEn, 255);
    }

    void idle() override
    {
        digitalWrite(pinA, LOW);
        digitalWrite(pinB, LOW);
        analogWrite(pinEn, 0);
    }

private:
    int pinA, pinB, pinEn;
};