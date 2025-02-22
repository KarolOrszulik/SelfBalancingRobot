#include <Arduino.h>
#include <WiFi.h>

#include "secret_wifi.hpp"
#include "pins.hpp"

#include "Motor/Motor_3Pin.hpp"
#include "Motor/Motor_MinPWM.hpp"
#include "IMU/MPU6050_IMU.hpp"
#include "Orientation/ComplementaryFilterOrientation.hpp"
#include "Controller/PIDController.hpp"

static IMotor* motorL;
static IMotor* motorR;

static IIMU* imu;
static IOrientation* orientation;

static IController* controller;

static volatile bool shouldUpdate = false;

static hw_timer_t* timer;

void createMotors()
{
    motorL = new Motor_MinPWM(
        new Motor_3Pin(MOTOR_L_PINS.a, MOTOR_L_PINS.b, MOTOR_L_PINS.en),
        80);
    
    motorR = new Motor_MinPWM(
        new Motor_3Pin(MOTOR_R_PINS.a, MOTOR_R_PINS.b, MOTOR_R_PINS.en),
        90);
}

void createIMU()
{
    Wire.begin(I2C_PINS.sda, I2C_PINS.scl);
    imu = new MPU6050_IMU(&Wire);
}

void createOrientation()
{
    orientation = new ComplementaryFilterOrientation(imu);
}

void IRAM_ATTR isr()
{
    shouldUpdate = true;
}

void createUpdateInterrupt()
{
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, isr, true);
    timerAlarmWrite(timer, 10000, true);
    timerAlarmEnable(timer);
}

void createController()
{
    controller = new PIDController(3.f, 0.4f, -0.4f);
    controller->setSetpoint(-0.025f);
}

void setup()
{
    Serial.begin(115200);

    createMotors();
    createIMU();
    createOrientation();
    createUpdateInterrupt();
    createController();

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("connected with local IP: " + WiFi.localIP().toString());
}

void loop()
{
    if (!shouldUpdate)
        return;

    shouldUpdate = false;

    imu->update();
    orientation->update();

    IMUData imuData = imu->getData();
    OrientationData orientationData = orientation->getData();
    
    Serial.printf(">ax: %f\n>ay: %f\n>az: %f\n", imuData.accelX, imuData.accelY, imuData.accelZ);
    Serial.printf(">pr: %f\n>rr: %f\n>yr: %f\n", imuData.pitchRate, imuData.rollRate, imuData.yawRate);
    Serial.printf(">p: %f\n>r: %f\n", orientationData.pitch, orientationData.roll);

    const float output = controller->update(orientationData.roll, imuData.rollRate);
    Serial.printf(">o: %f\n", output);

    motorL->setSpeed(255 * output);
    motorR->setSpeed(255 * output);
}