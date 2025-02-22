#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>

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

static WebServer server(80);
static WiFiUDP udp;

static float outputAlpha = 0.9f;

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
    hw_timer_t* timer = timerBegin(0, 80, true);
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
    Serial.printf("Connecting to WiFi %s", WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("connected with local IP: " + WiFi.localIP().toString());

    if (!MDNS.begin("self-balancing-robot"))
    {
        Serial.println("Error setting up MDNS responder!");
        while (1)
        {
            delay(1000);
        }
    }

    server.on("/", HTTP_GET, [] {
        server.send(200, "text/plain", "Self Balancing Robot");
    });
    server.on("/settings", HTTP_GET, [] {
        String message;

        if (server.hasArg("p") && server.hasArg("i") && server.hasArg("d"))
        {
            const float p = server.arg("p").toFloat();
            const float i = server.arg("i").toFloat();
            const float d = server.arg("d").toFloat();
            ((PIDController*)controller)->setGains(p, i, d);

            message += "PID gains set to: kp=" + String(p) + ", ki=" + String(i) + ", kd=" + String(d) + "\n";
        }

        if (server.hasArg("lmin") && server.hasArg("rmin"))
        {
            const int lmin = server.arg("lmin").toInt();
            const int rmin = server.arg("rmin").toInt();
            ((Motor_MinPWM*)motorL)->setMinPWM(lmin);
            ((Motor_MinPWM*)motorR)->setMinPWM(rmin);

            message += "Min PWM set to: lmin=" + String(lmin) + ", rmin=" + String(rmin) + "\n";
        }

        if (server.hasArg("setpoint"))
        {
            const float setpoint = server.arg("setpoint").toFloat();
            controller->setSetpoint(setpoint);

            message += "Setpoint set to: " + String(setpoint, 3) + "\n";
        }

        if (server.hasArg("alpha"))
        {
            outputAlpha = server.arg("alpha").toFloat();

            message += "Output alpha set to: " + String(outputAlpha) + "\n";
        }

        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", message);
    });
    server.begin();

    delay(2000);
}

void loop()
{
    server.handleClient();

    if (!shouldUpdate)
        return;

    shouldUpdate = false;

    imu->update();
    orientation->update();

    IMUData imuData = imu->getData();
    OrientationData orientationData = orientation->getData();

    static float output = 0.f;
    const float newOutput = controller->update(orientationData.roll, imuData.rollRate);

    output = outputAlpha * output + (1.f - outputAlpha) * newOutput;

    udp.beginPacket("192.168.1.189", 47269);
    udp.printf(
        "ax:%f\n"
        "ay:%f\n"
        "az:%f\n"
        "pr:%f\n"
        "rr:%f\n"
        "yr:%f\n"
        "p:%f\n"
        "r:%f\n"
        "o:%f\n",
        imuData.accelX,
        imuData.accelY,
        imuData.accelZ,
        imuData.pitchRate,
        imuData.rollRate,
        imuData.yawRate,
        orientationData.pitch,
        orientationData.roll,
        output
    );
    udp.endPacket();

    motorL->setSpeed(255 * output);
    motorR->setSpeed(255 * output);
}