#include <Arduino.h>
#include <RoboClaw.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <HTTPClient.h>
#include <u8x8lib.h>
#include <ESP32_ISR_Servo.h>
#include "config.h"

#define USE_SERIAL Serial
#define TIMER_INTERRUPT_DEBUG 1
//#define ISR_SERVO_DEBUG             1

// Roboclaw constants
#define ROBOCLAW_ADDRESS 0x80
#define POWER_SCALING 126

// Limit Switch Values
#define BOTTOM_LIMIT_PIN 23
#define TOP_LIMIT_PIN 19
#define LIMIT_SWITCH_PRESSED 0
#define LIMIT_SWITCH_RELEASED 1

// LED Relay Values
#define LED_PIN 14
#define LIGHTS_OFF 1
#define LIGHTS_ON 0

// Values to Set motors to
#define RETRACT 1
#define DEPLOY -1
#define STOP 0

// Garage States
#define DEPLOYED 0
#define RETRACTED 1
#define MOVING 2
#define ELEVATOR_UNKNOWN 3

// Lock States
#define LOCKED 0
#define UNLOCKED 1
#define LOCK_UNKNOWN 2

// Servo Parameters
#define MIN_MICROS 500
#define MAX_MICROS 2500
#define USE_ESP32_TIMER_NO 3
#define LEFT_SERVO_PIN 33
#define RIGHT_SERVO_PIN 25

// Creat Motor Controller
RoboClaw roboclaw(&Serial2, 10000);

WiFiMulti WiFiMulti;
SocketIOclient socketIO;

// Setup Screen
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/15, /* data=*/4, /* reset=*/16);

String token = "";
String command = "";

// System State Variables
uint8_t lightState = LIGHTS_OFF;
uint8_t lockState = LOCK_UNKNOWN;
uint8_t elevatorState = MOVING; // Set to moving to account for unknown state

// System Desired Variables
uint8_t lockDesired = LOCK_UNKNOWN;
uint8_t elevatorDesired = ELEVATOR_UNKNOWN;

uint64_t lockUpdateTime = 0;
uint8_t lockSetState = LOCK_UNKNOWN;
double leftMotorPower = 0;
double rightMotorPower = 0;
int rightServoIndex = -1;
int leftServoIndex = -1;

String getStateString()
{
    if (elevatorState == DEPLOYED)
    {
        return "deployed";
    }
    else if (elevatorState == RETRACTED)
    {
        if (lockState == UNLOCKED)
        {
            return "retracted_unlatched";
        }
        else if (lockState == LOCKED)
        {
            return "retracted_latched";
        }
        else
        {
            return "unavailable";
        }
    }
    else
    {
        if (elevatorDesired == DEPLOYED)
        {
            return "in_motion_deploy";
        }
        else if (elevatorDesired == RETRACT)
        {
            return "in_motion_retract";
        }
        else
        {
            return "unavailable";
        }
    }
}

void sendMotorPowers()
{
    if (leftMotorPower >= 0)
    {
        roboclaw.ForwardM1(ROBOCLAW_ADDRESS, (uint8_t)(POWER_SCALING * leftMotorPower));
    }
    else
    {
        roboclaw.BackwardM1(ROBOCLAW_ADDRESS, (uint8_t)(POWER_SCALING * -leftMotorPower));
    }

    if (rightMotorPower >= 0)
    {
        roboclaw.ForwardM2(ROBOCLAW_ADDRESS, (uint8_t)(POWER_SCALING * rightMotorPower));
    }
    else
    {
        roboclaw.BackwardM2(ROBOCLAW_ADDRESS, (uint8_t)(POWER_SCALING * -rightMotorPower));
    }
}

void setMotors(double power)
{

    if (power > 1)
    {
        power = 1;
    }
    else if (power < -1)
    {
        power = -1;
    }

    leftMotorPower = power;
    rightMotorPower = power;
}

void setLock(int state)
{
    // This needs to be tuned.
    lockSetState = state;
    if (state == LOCKED)
    {
        ESP32_ISR_Servos.setPosition(rightServoIndex, 45);
        ESP32_ISR_Servos.setPosition(leftServoIndex, 10);
    }
    else if (state == UNLOCKED)
    {
        ESP32_ISR_Servos.setPosition(rightServoIndex, 45);
        ESP32_ISR_Servos.setPosition(leftServoIndex, 80);
    }
    else
    {
    }
}

void updateLimitSwitches()
{
    int topLimit = digitalRead(TOP_LIMIT_PIN);
    int bottomLimit = digitalRead(BOTTOM_LIMIT_PIN);

    if (topLimit == LIMIT_SWITCH_PRESSED)
    {
        elevatorState = RETRACTED;
        if (rightMotorPower > 0)
        {
            Serial.println("Stopping Right, Top Limit Switch Hit");
            rightMotorPower = 0;
        }
        if (leftMotorPower > 0)
        {
            Serial.println("Stopping Left, Top Limit Switch Hit");
            leftMotorPower = 0;
        }
    }
    else if (bottomLimit == LIMIT_SWITCH_PRESSED)
    {
        elevatorState = DEPLOYED;
        lockState = UNLOCKED;
        if (rightMotorPower < 0)
        {
            Serial.println("Stopping Right, Bottom Limit Switch Hit");
            rightMotorPower = 0;
        }
        if (leftMotorPower < 0)
        {
            Serial.println("Stopping Left, Bottom Limit Switch Hit");
            leftMotorPower = 0;
        }
    }
    else
    {
        elevatorState = MOVING;
    }
}

void updateLock()
{
    uint64_t now = millis();

    if (lockState != lockDesired && lockDesired != LOCK_UNKNOWN)
    {
        if (elevatorState == RETRACTED)
        {
            setLock(lockDesired);
            lockUpdateTime = millis();
        }
    }
    else if (lockState == LOCK_UNKNOWN)
    {
        setLock(UNLOCKED);
    }

    if (now - lockUpdateTime > 1000 && lockSetState != LOCK_UNKNOWN)
    {
        lockState = lockSetState;
    }
}

void updateElevator()
{
    if (elevatorDesired != elevatorState && elevatorDesired != ELEVATOR_UNKNOWN)
    {
        if (lockState == UNLOCKED)
        {
            if (elevatorDesired == DEPLOYED)
            {
                //Serial.println("Trying to Deploy");
                setMotors(DEPLOY);
            }
            else if (elevatorDesired == RETRACT)
            {
                //Serial.println("Trying to Retract");
                setMotors(RETRACT);
            }
            else
            {
                //Serial.println("Stopping Unknown Objective");
                setMotors(STOP);
            }
        }
        else
        {
            //Serial.printf("Stopping Garage is Locked %i\n", lockState);
            setMotors(STOP);
        }
    }
    else
    {
        Serial.println("Stopping Elevator is in Desired State");
        setMotors(STOP);
    }

    sendMotorPowers();
}

void deployElevator()
{
    lockDesired = UNLOCKED;
    elevatorDesired = DEPLOYED;
    lightState = LIGHTS_ON;
}

void retractElevator()
{
    elevatorDesired = RETRACTED;
    lockDesired = LOCKED;
    lightState = LIGHTS_OFF;
}

void drawStatus()
{

    IPAddress address = WiFi.localIP();
    String addressString = address.toString();

    u8x8.drawString(0, 0, "Connected To:");
    u8x8.drawString(0, 1, ssid);
    u8x8.drawString(0, 3, "Current IP: ");
    u8x8.drawString(0, 4, addressString.c_str());
    if (token != "")
    {
        u8x8.drawString(0, 6, "Token: Acquired");
    }
    else
    {
        u8x8.drawString(0, 6, "Token: Missing");
    }
}

void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case sIOtype_DISCONNECT:
        USE_SERIAL.printf("[IOc] Disconnected!\n");
        // USE_SERIAL.printf("%s", payload);
        break;
    case sIOtype_CONNECT:
        Serial.println("Connect");
        USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);

        // join default namespace (no auto join in Socket.IO V3)
        socketIO.send(sIOtype_CONNECT, "/");
        break;
    case sIOtype_EVENT:
    {
        char *sptr = NULL;
        int id = strtol((char *)payload, &sptr, 10);
        USE_SERIAL.printf("[IOc] get event: %s id: %d\n", payload, id);
        if (id)
        {
            payload = (uint8_t *)sptr;
        }
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, payload, length);
        if (error)
        {
            USE_SERIAL.print(F("deserializeJson() failed: "));
            USE_SERIAL.println(error.c_str());
            return;
        }

        String eventName = doc[0];
        String eventBody = doc[1];

        DynamicJsonDocument innerDoc(1024);
        error = deserializeJson(innerDoc, eventBody);

        if (error)
        {
            USE_SERIAL.print(F("deserializeJson() failed: "));
            USE_SERIAL.println(error.c_str());
            return;
        }

        const char *cmd = innerDoc["command"];
        String command = String(cmd);

        if (command == "retract")
        {
            Serial.println("retract");
            retractElevator();
        }
        else if (command == "deploy")
        {
            Serial.println("Deploying!");
            deployElevator();
        }
        else if (command == "stop")
        {
            Serial.println("Stopping!");
            setMotors(STOP);
            lightState = LIGHTS_OFF;
        }
        else if (command == "lights_on")
        {
            lightState = LIGHTS_ON;
        }
        else if (command == "lights_off")
        {
            lightState = LIGHTS_OFF;
        }
        else
        {
            Serial.printf("Unknown Command: %s\n", command.c_str());
        }

        // Message Includes a ID for a ACK (callback)
        if (id)
        {
            // creat JSON message for Socket.IO (ack)
            DynamicJsonDocument docOut(1024);
            JsonArray array = docOut.to<JsonArray>();

            // add payload (parameters) for the ack (callback function)
            JsonObject param1 = array.createNestedObject();
            param1["now"] = millis();

            // JSON to String (serializion)
            String output;
            output += id;
            serializeJson(docOut, output);

            // Send event
            socketIO.send(sIOtype_ACK, output);
        }
    }
    break;
    case sIOtype_ACK:
        USE_SERIAL.printf("[IOc] get ack: %u\n", length);
        break;
    case sIOtype_ERROR:
        USE_SERIAL.printf("[IOc] get error: %u\n", length);
        break;
    case sIOtype_BINARY_EVENT:
        USE_SERIAL.printf("[IOc] get binary: %u\n", length);
        break;
    case sIOtype_BINARY_ACK:
        USE_SERIAL.printf("[IOc] get binary ack: %u\n", length);
        break;
    }
}

void connectToNetwork()
{
    u8x8.clearDisplay();
    u8x8.drawString(0, 2, "Connecting to:");
    u8x8.drawString(0, 4, ssid);
    int retryCount = 0;
    Serial.println("Connecting to Network");
    while (WiFiMulti.run() != WL_CONNECTED)
    {
        delay(500);

        if (retryCount > 16)
        {
            retryCount = 0;
            u8x8.clearLine(6);
        }
        u8x8.drawString(retryCount, 6, ".");

        retryCount += 1;
        if (retryCount > 3)
        {
            ESP.restart();
        }
    }
    String ip = WiFi.localIP().toString();
    USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());
    u8x8.clearDisplay();
    drawStatus();
}

void getToken()
{
    while (token == "")
    {
        HTTPClient http;
        String endpoint = "http://" + apiHost + ":" + apiPort + "/token";
        String auth = "grant_type=&username=" + username + "&password=" + apiPassword + "&scope=&client_id=&client_secret=";

        http.begin(endpoint.c_str());
        http.setTimeout(10000);
        http.setConnectTimeout(10000);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        http.addHeader("Content-Length", "" + strlen(auth.c_str()));

        Serial.println("Sending Request for Token");
        Serial.println(endpoint);
        int httpResponseCode = http.POST(auth);
        if (httpResponseCode > 0 && httpResponseCode < 400)
        {
            String payload = http.getString();

            DynamicJsonDocument doc(4096);
            DeserializationError error = deserializeJson(doc, payload);

            if (error)
            {
                Serial.println("Encountered Error in JSON Parsing");
                Serial.print(error.f_str());
            }

            if (doc.containsKey("access_token"))
            {
                Serial.println("Found Access Token");
                const char *access_token = doc["access_token"];
                token = String(access_token);
            }
            else
            {
                Serial.println("Failed to find access token");
            }
        }
        else
        {
            Serial.print("Error code: ");
            Serial.println(httpResponseCode);
            delay(2000);
        }
        http.end();
        drawStatus();
        delay(1000);
    }
}

void setupScreen()
{
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.clearDisplay();
}

void setupSerial()
{
    USE_SERIAL.begin(115200);
    USE_SERIAL.setDebugOutput(true);
}

void setupRoboclaw()
{
    roboclaw.begin(38400);
    roboclaw.clear();
    setMotors(STOP);
    sendMotorPowers();
}

void setupLimitSwitches()
{
    pinMode(BOTTOM_LIMIT_PIN, INPUT);
    pinMode(TOP_LIMIT_PIN, INPUT);
}

void setupLights()
{
    pinMode(LED_PIN, OUTPUT);
    lightState = LIGHTS_OFF;
    digitalWrite(LED_PIN, lightState);
}

void setupServos()
{
    ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);
    rightServoIndex = ESP32_ISR_Servos.setupServo(RIGHT_SERVO_PIN, MIN_MICROS, MAX_MICROS);
    leftServoIndex = ESP32_ISR_Servos.setupServo(LEFT_SERVO_PIN, MIN_MICROS, MAX_MICROS);
}

void setup()
{

    // Setup RoboClaw
    setupRoboclaw();

    // Setup Screen for Operation
    setupScreen();

    // Setup Serial Bus
    setupSerial();

    // Setup Top and Bottom Soft Limits
    setupLimitSwitches();

    // Setup Lights
    setupLights();

    // Setup Servos
    setupServos();

    for (uint8_t t = 4; t > 0; t--)
    {
        USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }

    Serial.printf("%i, %i\n", rightServoIndex, leftServoIndex);

    WiFiMulti.addAP(ssid, wifiPassword);

    connectToNetwork();

    getToken();
    String headers = "ID:" + garageID + "\nDEVICE_TYPE:garage" + "\nTOKEN:" + token;

    socketIO.setExtraHeaders(headers.c_str());
    socketIO.begin(apiHost, apiPort, "/ws/socket.io/?EIO=4");
    socketIO.onEvent(socketIOEvent);
    socketIO.send(sIOtype_CONNECT, "/");
}
unsigned long messageTimestamp = 0;

void loop()
{

    socketIO.loop();

    uint64_t now = millis();

    if (now - messageTimestamp > 5000)
    {

        // int32_t enc1= roboclaw.ReadEncM1(0x80);
        // Serial.println("Motor Position"+String(enc1));
        messageTimestamp = now;

        // creat JSON message for Socket.IO (event)
        DynamicJsonDocument doc(1024);
        JsonArray array = doc.to<JsonArray>();

        // add evnet name
        // Hint: socket.on('event_name', ....
        array.add("data");

        // add payload (parameters) for the event
        JsonObject param1 = array.createNestedObject();
        param1["now"] = (uint32_t)now;
        param1["garage_id"] = garageID;
        param1["linked_rover_id"] = "rover_1";
        param1["state"] = getStateString();
        param1["health"] = "healthy";
        param1["health_details"] = "healthy";

        if (lightState == LIGHTS_ON)
        {
            param1["lights_on"] = true;
        }
        else
        {
            param1["lights_on"] = false;
        }
        Serial.println(getStateString());
        Serial.println(lockState);

        // param1["light_state"] = lightState;

        // JSON to String (serializion)
        String output;
        serializeJson(doc, output);

        // Send event
        socketIO.sendEVENT(output);

        // Print JSON for debugging
        USE_SERIAL.println(output);
    }

    digitalWrite(LED_PIN, lightState);

    // Update Limit Switches to check platform state
    updateLimitSwitches();
    updateLock();
    updateElevator();
}