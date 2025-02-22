#pragma once

struct MotorPins
{
    int a, b, en;
};

constexpr MotorPins MOTOR_L_PINS = {17, 16, 33};
constexpr MotorPins MOTOR_R_PINS = {19, 18, 32};


struct I2CPins
{
    int sda, scl;
};

constexpr I2CPins I2C_PINS = {26, 27};

constexpr int IMU_INT_PIN = 25;