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
#define TIMER_INTERRUPT_DEBUG 0
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
#define LIGHTS_OFF 0
#define LIGHTS_ON 1

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
#define USE_ESP32_TIMER_NO 1
#define LEFT_SERVO_PIN 25
#define RIGHT_SERVO_PIN 33

// Garage Sensor Parameters
#define ROVER_SENSOR_PIN 34
#define UNBROKEN 0
#define BROKEN 1

// Creat Motor Controller
RoboClaw roboclaw(&Serial2, 10000);

//WiFiMulti WiFiMulti;
SocketIOclient socketIO;

// Setup Screen
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/15, /* data=*/4, /* reset=*/16);

String token = "";
String command = "";

// System State Variables
uint8_t lightState = LIGHTS_OFF;
uint8_t lockState = LOCK_UNKNOWN;
uint8_t elevatorState = MOVING; // Set to moving to account for unknown state
uint8_t roverSensorState = UNBROKEN;

// System Desired Variables
uint8_t lockDesired = LOCK_UNKNOWN;
uint8_t elevatorDesired = ELEVATOR_UNKNOWN;

uint64_t lockUpdateTime = 0;
uint8_t lockSetState = LOCK_UNKNOWN;
double leftMotorPower = 0;
double rightMotorPower = 0;
int rightServoIndex = -1;
int leftServoIndex = -1;


// Additional Statistics
float temperature = 0;
float voltage = 0;


void scanWifiNetworks(){
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int n = WiFi.scanNetworks();
    Serial.println(" scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i) {
        // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(") Enc:");
            Serial.print(WiFi.encryptionType(i));
            Serial.print(", BSSID: ");
            Serial.print(WiFi.BSSIDstr(i));
            Serial.print(", Channel: ");
            Serial.print(WiFi.channel(i));
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
            delay(10);
        }
    }
    Serial.println("");
}

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
        ESP32_ISR_Servos.setPosition(rightServoIndex, 140);//172
        ESP32_ISR_Servos.setPosition(leftServoIndex, 32);//28
    }
    else if (state == UNLOCKED)
    {
        ESP32_ISR_Servos.setPosition(rightServoIndex, 100);//140
        ESP32_ISR_Servos.setPosition(leftServoIndex,65);//75
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

void updateRoboClaw(){
    uint16_t temp = 0;
    bool error = roboclaw.ReadTemp(ROBOCLAW_ADDRESS, temp);
    
    delay(10);
    temperature = (float)temp/10.0;
    
    
    int read_voltage = 0;
    read_voltage = roboclaw.ReadMainBatteryVoltage(0x80);
    
    delay(10);
    voltage = (float)read_voltage/10.0;
    

    int16_t current1 = 0;
    int16_t current2 = 0;
    bool currentError = roboclaw.ReadCurrents(0x80, current1, current2);
}

void updateLock()
{
    uint64_t now = millis();

    if (lockState != lockDesired && lockDesired != LOCK_UNKNOWN)
    {
        if (elevatorState == RETRACTED or lockDesired == UNLOCKED)
        {
            setLock(lockDesired);
            lockUpdateTime = millis();
        }
    }
    else if (lockState == LOCK_UNKNOWN)
    {
        setLock(UNLOCKED);
    }

    if (now - lockUpdateTime > 2000 && lockSetState != LOCK_UNKNOWN)
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
                setMotors(DEPLOY);
            }
            else if (elevatorDesired == RETRACT)
            {
                setMotors(RETRACT);
            }
            else
            {
                setMotors(STOP);
            }
        }
        else
        {
            setMotors(STOP);
        }
    }
    else
    {
        setMotors(STOP);
    }

    sendMotorPowers();
}

void updateRoverSensor(){
    roverSensorState = digitalRead(ROVER_SENSOR_PIN);
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

    if(WiFi.status() == WL_CONNECTED){
        IPAddress address = WiFi.localIP();
        String addressString = address.toString();

        u8x8.drawString(0, 0, "Connected To:");
        u8x8.drawString(0, 1, ssid);
        u8x8.drawString(0, 3, "Current IP: ");
        u8x8.drawString(0, 4, addressString.c_str());
    } else{
        u8x8.drawString(0, 0, "Connecting To:");
        u8x8.drawString(0, 1, ssid);
        u8x8.drawString(0, 3, "Current IP: ");
        u8x8.drawString(0, 4, "Not Connected");
    }
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
            Serial.println("Lights On!");
            lightState = LIGHTS_ON;
        }
        else if (command == "lights_off")
        {
            Serial.println("Lights Off!");
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

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  //Serial.println(info.wps_fail_reason);
  //Serial.println(info.prov_fail_reason);
  switch (event) {
    case SYSTEM_EVENT_WIFI_READY: 
      Serial.println("WiFi interface ready");
      break;
    case SYSTEM_EVENT_SCAN_DONE:
      Serial.println("Completed scan for access points");
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("WiFi client started");
      break;
    case SYSTEM_EVENT_STA_STOP:
      Serial.println("WiFi clients stopped");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("Connected to access point");
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("Disconnected from WiFi access point");
      delay(1000);
      WiFi.disconnect(true);
      delay(1000);
      WiFi.reconnect();
      break;
    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
      Serial.println("Authentication mode of access point has changed");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.print("Obtained IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.println("Lost IP address and IP address is reset to 0");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
      Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
      Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
      Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
      break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
      Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
      break;
    case SYSTEM_EVENT_AP_START:
      Serial.println("WiFi access point started");
      break;
    case SYSTEM_EVENT_AP_STOP:
      Serial.println("WiFi access point  stopped");
      break;
    case SYSTEM_EVENT_AP_STACONNECTED:
      Serial.println("Client connected");
      break;
    case SYSTEM_EVENT_AP_STADISCONNECTED:
      Serial.println("Client disconnected");
      break;
    case SYSTEM_EVENT_AP_STAIPASSIGNED:
      Serial.println("Assigned IP address to client");
      break;
    case SYSTEM_EVENT_AP_PROBEREQRECVED:
      Serial.println("Received probe request");
      break;
    case SYSTEM_EVENT_GOT_IP6:
      Serial.println("IPv6 is preferred");
      break;
    case SYSTEM_EVENT_ETH_START:
      Serial.println("Ethernet started");
      break;
    case SYSTEM_EVENT_ETH_STOP:
      Serial.println("Ethernet stopped");
      break;
    case SYSTEM_EVENT_ETH_CONNECTED:
      Serial.println("Ethernet connected");
      break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
      Serial.println("Ethernet disconnected");
      break;
    case SYSTEM_EVENT_ETH_GOT_IP:
      Serial.println("Obtained IP address");
      break;
    default: break;
}}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  //Serial.println(info.);
  Serial.println("Trying to Reconnect");
  //WiFi.begin(ssid, password);
}
/*
void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  Serial.println("Disconnected from WIFI access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.eth_connected);
  Serial.println("Reconnecting...");
  WiFi.begin(ssid, wifiPassword);
}
*/
void connectToNetwork()
{
    u8x8.clearDisplay();
    u8x8.drawString(0, 2, "Connecting to:");
    u8x8.drawString(0, 4, ssid);
    int retryCount = 0;
    Serial.println("Connecting to Network");


    WiFi.disconnect(true);
    WiFi.onEvent(WiFiEvent);
    WiFi.begin(ssid, wifiPassword);
    
    

    //while (WiFiMulti.run() != WL_CONNECTED)
    while (WiFi.status() != WL_CONNECTED)
    {

        WiFi.disconnect(true);
        WiFi.begin( ssid, wifiPassword );
        delay(10000);
        Serial.println(WiFi.status());
        
        

        if (retryCount > 16)
        {
            retryCount = 0;
            u8x8.clearLine(6);
        }
        u8x8.drawString(retryCount, 6, ".");

        retryCount += 1;
        if (retryCount > 10)
        {
        //    ESP.restart();
        }
    }
    String ip = WiFi.localIP().toString();
    USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());

    String dns = WiFi.dnsIP().toString();
    USE_SERIAL.printf("[SETUP] DNS Connected %s\n", dns.c_str());

    u8x8.clearDisplay();
    drawStatus();
}

void getToken()
{
    while (token == "")
    {

        HTTPClient http;
        String endpoint = "https://" + apiHost + ":" + apiPort + "/token";
        
        String auth = "grant-type=&username=" + username + "&password=" + apiPassword + "&scope=&client-id=&client-secret=";

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
    setLock(UNLOCKED);
}

void setupRoverSensor(){
    pinMode(ROVER_SENSOR_PIN, INPUT_PULLDOWN);
    roverSensorState = digitalRead(ROVER_SENSOR_PIN);
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

    // Setup Rover Sensor
    setupRoverSensor();

    for (uint8_t t = 4; t > 0; t--)
    {
        USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }

    scanWifiNetworks();
    
    // // Code for Servo Position Tuning
    // bool forward = false;
    // for(;;){
    //     forward = !forward;
    //     if(forward){
    //         setLock(UNLOCKED);
    //         Serial.println("Lights on");
    //         //lightState = LIGHTS_ON;
            
    //     }else{
    //         setLock(LOCKED);
    //         Serial.println("Lights off");
    //         //lightState = LIGHTS_OFF;
    //     }
    //     updateRoverSensor();
    //     Serial.println(roverSensorState);
    //     digitalWrite(LED_PIN, lightState);
    //     delay(5000);
    // }
    
   
    connectToNetwork();

    getToken();
    String headers = "ID:" + garageID + "\nDEVICE-TYPE:garage" + "\nTOKEN:" + token;

    socketIO.setExtraHeaders(headers.c_str());
    socketIO.beginSSL(apiHost, apiPort, "/ws/socket.io/?EIO=4");
    socketIO.onEvent(socketIOEvent);
    socketIO.send(sIOtype_CONNECT, "/");
    
    
}
unsigned long messageTimestamp = 0;

void loop()
{

    if(WiFi.status() != WL_CONNECTED){
        Serial.println("Lost Wifi Connecting. Reconnecting");
        drawStatus();
        //WiFi.reconnect();
    }

    socketIO.loop();
    uint64_t now = millis();

    //Serial.println("Sent Cloud Update");
    if (now - messageTimestamp > 5000)
    {

        int32_t enc1= roboclaw.ReadEncM1(0x80);
        Serial.println("Motor Position"+String(enc1));
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
        param1["temperature"] = temperature;
        param1["voltage"] = voltage;

        if (lightState == LIGHTS_ON)
        {
            param1["lights_on"] = true;
        }
        else
        {
            param1["lights_on"] = false;
        }

        if (roverSensorState == BROKEN){
            param1["rover_docked"] = true;
        } else{
            param1["rover_docked"] = false;
        }
        Serial.println(getStateString());

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
    updateRoverSensor();
    updateLimitSwitches();
    updateRoboClaw();
    updateLock();
    updateElevator();  
    delay(100);
}