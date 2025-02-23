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

static IController* motorController;
static IController* rollSetpointController;

static volatile bool shouldUpdate = false;

static WebServer server(80);
static WiFiUDP udp;


struct RobotSettings
{
    struct PidGains
    {
        float p;
        float i;
        float d;
    };

    PidGains kMotor;
    PidGains kRoll;

    int lmin;
    int rmin;

    float motorAlpha;
    float rollAlpha;
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

void createMotorController()
{
    motorController = new PIDController(robotSettings.kMotor.p, robotSettings.kMotor.i, robotSettings.kMotor.d);
    motorController->setSetpoint(0.f);
}

void createRollSetpointController()
{
    rollSetpointController = new PIDController(robotSettings.kRoll.p, robotSettings.kRoll.i, robotSettings.kRoll.d);
    rollSetpointController->setSetpoint(0.f);
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
    ((PIDController*)rollSetpointController)->setGains(robotSettings.kRoll.p, robotSettings.kRoll.i, robotSettings.kRoll.d);
    ((PIDController*)motorController)->setGains(robotSettings.kMotor.p, robotSettings.kMotor.i, robotSettings.kMotor.d);
}

void setup()
{
    Serial.begin(115200);

    loadSettingsFromFlash();

    createMotors();
    createIMU();
    createOrientation();
    createUpdateInterrupt();
    createMotorController();
    createRollSetpointController();

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
        String message = "Settings:\n";
        message += "Motor PID gains: kp=" + String(robotSettings.kMotor.p) + ", ki=" + String(robotSettings.kMotor.i) + ", kd=" + String(robotSettings.kMotor.d) + "\n";
        message += "Roll PID gains: kp=" + String(robotSettings.kRoll.p) + ", ki=" + String(robotSettings.kRoll.i) + ", kd=" + String(robotSettings.kRoll.d) + "\n";
        message += "Min PWM: lmin=" + String(robotSettings.lmin) + ", rmin=" + String(robotSettings.rmin) + "\n";
        message += "Motor alpha: " + String(robotSettings.motorAlpha) + "\n";
        message += "Roll alpha: " + String(robotSettings.rollAlpha) + "\n";
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "text/plain", message);
    });

    server.on("/settings", HTTP_GET, [] {
        String message;

        if (server.hasArg("mp") && server.hasArg("mi") && server.hasArg("md"))
        {
            const float p = server.arg("mp").toFloat();
            const float i = server.arg("mi").toFloat();
            const float d = server.arg("md").toFloat();
            robotSettings.kMotor.p = p;
            robotSettings.kMotor.i = i;
            robotSettings.kMotor.d = d;

            message += "Motor PID gains set to: kp=" + String(p) + ", ki=" + String(i) + ", kd=" + String(d) + "\n";
        }
        else if (server.hasArg("rp") && server.hasArg("ri") && server.hasArg("rd"))
        {
            const float p = server.arg("rp").toFloat();
            const float i = server.arg("ri").toFloat();
            const float d = server.arg("rd").toFloat();
            robotSettings.kRoll.p = p;
            robotSettings.kRoll.i = i;
            robotSettings.kRoll.d = d;

            message += "Roll PID gains set to: kp=" + String(p) + ", ki=" + String(i) + ", kd=" + String(d) + "\n";
        }
        else if (server.hasArg("lmin") && server.hasArg("rmin"))
        {
            const int lmin = server.arg("lmin").toInt();
            const int rmin = server.arg("rmin").toInt();
            robotSettings.lmin = lmin;
            robotSettings.rmin = rmin;

            message += "Min PWM set to: lmin=" + String(lmin) + ", rmin=" + String(rmin) + "\n";
        }
        else if (server.hasArg("malpha") && server.hasArg("ralpha"))
        {
            const float motorAlpha = server.arg("malpha").toFloat();
            robotSettings.motorAlpha = motorAlpha;

            const float rollAlpha = server.arg("ralpha").toFloat();
            robotSettings.rollAlpha = rollAlpha;

            message += "Output alpha set to: " + String(motorAlpha) + "\n";
            message += "Roll alpha set to: " + String(rollAlpha) + "\n";
        }
        else
        {
            message += "Invalid settings\n";
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

    static float motorOutput = 0.f;
    const float newMotorOutput = motorController->update(orientationData.roll, imuData.rollRate);
    motorOutput = (1.f - robotSettings.motorAlpha) * motorOutput + robotSettings.motorAlpha * newMotorOutput;

    constexpr float MAX_SETPOINT = 0.08f;
    static float rollSetpoint = 0.f;
    const float newRollSetpoint = MAX_SETPOINT * rollSetpointController->update(motorOutput, 0);
    rollSetpoint = (1.f - robotSettings.rollAlpha) * rollSetpoint + robotSettings.rollAlpha * newRollSetpoint;

    motorL->setSpeed(255 * motorOutput);
    motorR->setSpeed(255 * motorOutput);

    motorController->setSetpoint(rollSetpoint);

    udp.beginPacket(IPAddress(192, 168, 1, 189), 47269);
    udp.printf(
        "rr:%f\n"
        "r:%f\n"
        "pwm:%d\n"
        "rollSetpoint:%f\n",
        imuData.rollRate,
        orientationData.roll,
        (int)(255.f * motorOutput),
        rollSetpoint
    );
    udp.endPacket();
}