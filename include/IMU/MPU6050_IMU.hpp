#pragma once

#include "IIMU.hpp"

#include <Arduino.h>
#include <Wire.h>

class MPU6050_IMU : public IIMU
{
public:
    MPU6050_IMU(TwoWire* wire)
        : wire(wire)
    {
        writeToRegister(MPU6050_PWR_MGMT_1,  0b00000000);
        writeToRegister(MPU6050_GYRO_CONFIG, 0b00010000);
        writeToRegister(MPU6050_ACCEL_CONFIG,0b00010000);
        writeToRegister(MPU6050_CONFIG,      0b00000010);
        writeToRegister(MPU6050_INT_PIN_CFG, 0b10011000);
        writeToRegister(MPU6050_INT_ENABLE,  0b00000001);
    }

    void update() override
    {
        wire->beginTransmission(MPU6050_ADDR);
        wire->write(MPU6050_ACCEL_XOUT_H);
        wire->endTransmission(false);
        wire->requestFrom(MPU6050_ADDR, 14, true);

        int16_t accelX = wire->read() << 8 | wire->read();
        int16_t accelY = wire->read() << 8 | wire->read();
        int16_t accelZ = wire->read() << 8 | wire->read();
        data.accelX = accelX / 4096.f;
        data.accelY = accelY / 4096.f;
        data.accelZ = accelZ / 4096.f;

        wire->read(); wire->read(); // Temperature

        int16_t gyroX = wire->read() << 8 | wire->read();
        int16_t gyroY = wire->read() << 8 | wire->read();
        int16_t gyroZ = wire->read() << 8 | wire->read();
        data.pitchRate = gyroX / 65.5f * PI / 180.f;
        data.rollRate  = gyroY / 65.5f * PI / 180.f;
        data.yawRate   = gyroZ / 65.5f * PI / 180.f;
    }

    IMUData getData() override
    {
        return data;
    }

private:
    static constexpr byte MPU6050_ADDR = 0x68;

    static constexpr byte MPU6050_PWR_MGMT_1 = 0x6B;
    static constexpr byte MPU6050_GYRO_CONFIG = 0x1B;
    static constexpr byte MPU6050_ACCEL_CONFIG = 0x1C;
    static constexpr byte MPU6050_CONFIG = 0x1A;
    static constexpr byte MPU6050_INT_PIN_CFG = 0x37;
    static constexpr byte MPU6050_INT_ENABLE = 0x38;

    static constexpr byte MPU6050_ACCEL_XOUT_H = 0x3B;
    static constexpr byte MPU6050_TEMP_OUT_H = 0x41;
    static constexpr byte MPU6050_GYRO_XOUT_H = 0x43;

    void writeToRegister(byte reg, byte data)
    {
        wire->beginTransmission(MPU6050_ADDR);
        wire->write(reg);
        wire->write(data);
        wire->endTransmission(true);
    }

    TwoWire* wire = nullptr;
    IMUData data;
};