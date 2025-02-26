#include <Arduino.h>
#include "components/DistanceSensor/DistanceSensor.h"
#include "components/MotorController/MotorController.h"
#include "components/Motor/Motor.h"
#include "components/LineSensor/LineSensor.h"
#include "components/VelocitySensor/VelocitySensor.h"
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <ArduinoJson.h>

#define SERVICE_UUID "d7aa9e26-3527-416a-aaee-c7b1454642dd"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define DISTANCE_TRIG_PIN 13
#define DISTANCE_ECHO_PIN 18
#define MOTOR_EN1_PIN 14
#define MOTOR_EN2_PIN 19
#define MOTOR_IN1_PIN 23
#define MOTOR_IN2_PIN 22
#define MOTOR_IN3_PIN 12
#define MOTOR_IN4_PIN 21
#define LINE_ENABLE_PIN 27
#define VELOCITY1_PIN 16
#define VELOCITY2_PIN 17
// Global variables
int speed = 50;
int turning = 50;
int sensitivity = 50;

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

// Components initialization
DistanceSensor distanceSensor(13, 18);
MotorController motorController(14, 19, 23, 22, 12, 21);
Motor motor1(motorController, 1);
Motor motor2(motorController, 2);
int sensors[8] = {26, 25, 33, 32, 35, 34, 39, 36};
LineSensor lineSensor(27, sensors);
VelocitySensor velocitySensor1(16);
VelocitySensor velocitySensor2(17);

class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected");
        // Reset motors on new connection
        motor1.setSpeed(0);
        motor2.setSpeed(0);
    }
    
    void onDisconnect(NimBLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected");
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        delay(500);
        NimBLEDevice::startAdvertising();  // This should restart advertising
    }
};

class CharacteristicCallbacks: public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        if (value.length() > 0) {
            // Print the raw received data
            Serial.print("Received command: ");
            Serial.println(value.c_str());
            
            StaticJsonDocument<512> doc;
            DeserializationError error = deserializeJson(doc, value.c_str());

            if (error) {
                Serial.println("Failed to parse JSON");
                return;
            }

            const char* command = doc["command"];
            Serial.print("Command type: ");
            Serial.println(command);
            
            if (strcmp(command, "move") == 0) {
                const char* direction = doc["direction"];
                Serial.print("Movement direction: ");
                Serial.println(direction);
                handleMovement(direction);
            }
            else if (strcmp(command, "set_param") == 0) {
                const char* param = doc["param"];
                int value = doc["value"];
                Serial.print("Parameter update: ");
                Serial.print(param);
                Serial.print(" = ");
                Serial.println(value);
                handleParameter(param, value);
            }
        }
    }

    // Keep your existing handleMovement and handleParameter functions
    void handleMovement(const char* direction) {
        // Your existing code...
    }

    void handleParameter(const char* param, int value) {
        // Your existing code...
    }
};
void setup() {
    Serial.begin(9600);

    // Initialize components
    distanceSensor.init();
    motorController.init();
    motor1.init();
    motor2.init();
    lineSensor.init();
    velocitySensor1.init();
    velocitySensor2.init();

    motor1.setSpeed(100);

    // Initialize NimBLE
    NimBLEDevice::init("ESP32-Robot");
    
    // Configure the server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Create the service
    NimBLEService *pService = pServer->createService(SERVICE_UUID);
    
    // Create the characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::WRITE_NR |  // Use WRITE_NR (Write Without Response)
        NIMBLE_PROPERTY::INDICATE    // Use INDICATE instead of NOTIFY if needed
    );

    pCharacteristic->setCallbacks(new CharacteristicCallbacks());

    // Start the service
    pService->start();

    // Configure advertising (No redeclaration error now)
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setName("ESP32-Robot");
    pAdvertising->setMinInterval(0x20);  // 0x20 * 0.625ms = 20ms
    pAdvertising->setMaxInterval(0x40);  // 0x40 * 0.625ms = 40ms
    
    // Start advertising
    pAdvertising->start();
    
    Serial.println("BLE server ready");
}

void loop() {
    if (deviceConnected) {
        StaticJsonDocument<200> doc;
        doc["distance"] = distanceSensor.getDistance();
        doc["line_value"] = lineSensor.getLinePosition();
        doc["velocity1"] = velocitySensor1.getVelocity();
        doc["velocity2"] = velocitySensor2.getVelocity();
    
        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
        pCharacteristic->indicate(); // Changed from notify() to indicate()
        
        delay(100);
    }

        
        delay(500); // Update rate: 10Hz
    }

