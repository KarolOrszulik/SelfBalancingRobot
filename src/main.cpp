#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <EEPROM.h>

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


struct RobotSettings
{
    float p;
    float i;
    float d;
    int lmin;
    int rmin;
    float setpoint;
    float alpha;
};

static RobotSettings robotSettings;


void createMotors()
{
    motorL = new Motor_MinPWM(
        new Motor_3Pin(MOTOR_L_PINS.a, MOTOR_L_PINS.b, MOTOR_L_PINS.en),
        robotSettings.lmin);
    
    motorR = new Motor_MinPWM(
        new Motor_3Pin(MOTOR_R_PINS.a, MOTOR_R_PINS.b, MOTOR_R_PINS.en),
        robotSettings.rmin);
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
    controller = new PIDController(robotSettings.p, robotSettings.i, robotSettings.d);
    controller->setSetpoint(robotSettings.setpoint);
}

void loadSettingsFromFlash()
{
    EEPROM.begin(sizeof(RobotSettings));
    EEPROM.get(0, robotSettings);
    EEPROM.end();
}

void saveSettingsToFlash()
{
    EEPROM.begin(sizeof(RobotSettings));
    EEPROM.put(0, robotSettings);
    EEPROM.commit();
    EEPROM.end();
}

void applyAllSettings()
{
    ((Motor_MinPWM*)motorL)->setMinPWM(robotSettings.lmin);
    ((Motor_MinPWM*)motorR)->setMinPWM(robotSettings.rmin);
    ((PIDController*)controller)->setGains(robotSettings.p, robotSettings.i, robotSettings.d);
    ((PIDController*)controller)->setSetpoint(robotSettings.setpoint);
}

void setup()
{
    Serial.begin(115200);

    loadSettingsFromFlash();

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
    }

    server.on("/", HTTP_GET, [] {
        server.send(200, "text/plain", "Self Balancing Robot");
    });

    server.on("/getSettings", HTTP_GET, [] {
        String message =
              "p=" + String(robotSettings.p) +
            + "i=" + String(robotSettings.i) +
            + "d=" + String(robotSettings.d) +
            + "lmin=" + String(robotSettings.lmin) +
            + "rmin=" + String(robotSettings.rmin) +
            + "setpoint=" + String(robotSettings.setpoint) +
            + "alpha=" + String(robotSettings.alpha);

        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", message);
    });

    server.on("/settings", HTTP_GET, [] {
        String message;

        if (server.hasArg("p") && server.hasArg("i") && server.hasArg("d"))
        {
            const float p = server.arg("p").toFloat();
            const float i = server.arg("i").toFloat();
            const float d = server.arg("d").toFloat();
            robotSettings.p = p;
            robotSettings.i = i;
            robotSettings.d = d;

            message += "PID gains set to: kp=" + String(p) + ", ki=" + String(i) + ", kd=" + String(d) + "\n";
        }

        if (server.hasArg("lmin") && server.hasArg("rmin"))
        {
            const int lmin = server.arg("lmin").toInt();
            const int rmin = server.arg("rmin").toInt();
            robotSettings.lmin = lmin;
            robotSettings.rmin = rmin;

            message += "Min PWM set to: lmin=" + String(lmin) + ", rmin=" + String(rmin) + "\n";
        }

        if (server.hasArg("setpoint"))
        {
            const float setpoint = server.arg("setpoint").toFloat();
            robotSettings.setpoint = setpoint;

            message += "Setpoint set to: " + String(setpoint, 3) + "\n";
        }

        if (server.hasArg("alpha"))
        {
            const float alpha = server.arg("alpha").toFloat();
            robotSettings.alpha = alpha;

            message += "Output alpha set to: " + String(alpha) + "\n";
        }

        saveSettingsToFlash();
        applyAllSettings();

        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", message);
    });
    server.begin();
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

    output = robotSettings.alpha * output + (1.f - robotSettings.alpha) * newOutput;

    udp.beginPacket("192.168.1.189", 47269);
    udp.printf(
        // "ax:%f\n"
        // "ay:%f\n"
        // "az:%f\n"
        // "pr:%f\n"
        "rr:%f\n"
        // "yr:%f\n"
        // "p:%f\n"
        "r:%f\n"
        "o:%f\n",
        // imuData.accelX,
        // imuData.accelY,
        // imuData.accelZ,
        // imuData.pitchRate,
        imuData.rollRate,
        // imuData.yawRate,
        // orientationData.pitch,
        orientationData.roll,
        255 * output
    );
    udp.endPacket();

    motorL->setSpeed(255 * output);
    motorR->setSpeed(255 * output);
}