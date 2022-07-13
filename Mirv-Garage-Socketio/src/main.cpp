
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <HTTPClient.h>
#include <U8x8lib.h>
#include "config.h"

#define USE_SERIAL Serial



WiFiMulti WiFiMulti;
SocketIOclient socketIO;
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
String token = "";

void drawStatus(){

    IPAddress address = WiFi.localIP();
    String addressString = address.toString();

void drawStatus(){

    IPAddress address = WiFi.localIP();
    String addressString = address.toString();



    u8x8.drawString(0, 0, "Connected To:");
    u8x8.drawString(0, 1, ssid);
    u8x8.drawString(0, 3, "Current IP: ");
    u8x8.drawString(0, 4, addressString.c_str());
    if(token != ""){
        u8x8.drawString(0, 6, "Token: Acquired");
    } else{
        u8x8.drawString(0, 6, "Token: Missing");
    }
  


}


    u8x8.drawString(0, 0, "Connected To:");
    u8x8.drawString(0, 1, ssid);
    u8x8.drawString(0, 3, "Current IP: ");
    u8x8.drawString(0, 4, addressString.c_str());
    if(token != ""){
        u8x8.drawString(0, 6, "Token: Acquired");
    } else{
        u8x8.drawString(0, 6, "Token: Missing");
    }
  


}

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            USE_SERIAL.printf("[IOc] Disconnected!\n");
            break; 
        case sIOtype_CONNECT:
            USE_SERIAL.printf("[IOc] Connected to url: %s\n", payload);

            // join default namespace (no auto join in Socket.IO V3)
            socketIO.send(sIOtype_CONNECT, "/");
            break;
        case sIOtype_EVENT:
        {
            char * sptr = NULL;
            int id = strtol((char *)payload, &sptr, 10);
            USE_SERIAL.printf("[IOc] get event: %s id: %d\n", payload, id);
            if(id) {
                payload = (uint8_t *)sptr;
            }
            DynamicJsonDocument doc(1024);
            DeserializationError error = deserializeJson(doc, payload, length);
            if(error) {
                USE_SERIAL.print(F("deserializeJson() failed: "));
                USE_SERIAL.println(error.c_str());
                return;
            }

            String eventName = doc[0];
            USE_SERIAL.printf("[IOc] event name: %s\n", eventName.c_str());

            // Message Includes a ID for a ACK (callback)
            if(id) {
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

void connectToNetwork(){
    u8x8.clearDisplay();
    u8x8.drawString(0, 2, "Connecting to:");
    u8x8.drawString(0, 4, ssid);
    int index = 0;
    while(WiFiMulti.run() != WL_CONNECTED) {
        delay(100);
        
        
        if(index > 16){
            index =0;
            u8x8.clearLine(6);
        }
        u8x8.drawString(index, 6, ssid);

        index +=1;
    }
    String ip = WiFi.localIP().toString();
    USE_SERIAL.printf("[SETUP] WiFi Connected %s\n", ip.c_str());
    u8x8.clearDisplay();
    drawStatus();
}

void getToken(){

    HTTPClient http;
    String endpoint = "http://" +apiHost + ":" + apiPort+"/token";
    
    http.begin(endpoint.c_str());
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String auth = "grant_type=&username="+username+"&password="+apiPassword+"&scope=&client_id=&client_secret=";

    Serial.println("Sending Request for Token");
    int httpResponseCode = http.POST(auth);
    if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();

        DynamicJsonDocument doc(4096);
        DeserializationError error = deserializeJson(doc, payload);

        if(error){
            Serial.println("Encountered Error in JSON Parsing");
            Serial.print(error.f_str());
        }

        if(doc.containsKey("access_token")){
            Serial.println("Found Access Token");
            const char* access_token = doc["access_token"];
            token = String(access_token);
        } else {
            Serial.println("Failed to find access token");
        }
    }
    else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
    }
    http.end();
    drawStatus();
}

void setupScreen(){
    u8x8.begin();
    u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.clearDisplay();
}

void setupSerial(){
    USE_SERIAL.begin(115200);
    USE_SERIAL.setDebugOutput(true);
}

void setup() {
    
    // Setup Screen for Operation
    setupScreen();

    // Setup Serial Bus
    setupSerial();

    for(uint8_t t = 4; t > 0; t--) {
        USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
        USE_SERIAL.flush();
        delay(1000);
    }

    WiFiMulti.addAP(ssid, wifiPassword);

    connectToNetwork();

    getToken();

    String headers = "GARAGEID:" + garageID+"\nAuthorization:Bearer"+token;

    socketIO.setExtraHeaders(headers.c_str());
    
    socketIO.begin(apiHost, apiPort, "/ws/socket.io/?EIO=4");
    socketIO.onEvent(socketIOEvent);
    //socketIO.send(sIOtype_CONNECT, "/");
    
}

unsigned long messageTimestamp = 0;
void loop() {

    socketIO.loop();
    uint64_t now = millis();

    if(now - messageTimestamp > 2000) {
        messageTimestamp = now;

        // creat JSON message for Socket.IO (event)
        DynamicJsonDocument doc(1024);
        JsonArray array = doc.to<JsonArray>();

        // add evnet name
        // Hint: socket.on('event_name', ....
        array.add("event_name");

        // add payload (parameters) for the event
        JsonObject param1 = array.createNestedObject();
        param1["now"] = (uint32_t) now;

        // JSON to String (serializion)
        String output;
        serializeJson(doc, output);

        // Send event
        socketIO.sendEVENT(output);

        // Print JSON for debugging
        USE_SERIAL.println(output);
    }
    
}