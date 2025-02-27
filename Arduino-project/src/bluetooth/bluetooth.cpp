#include "bluetooth.h"
#include <Arduino.h>

void bluetooth::init() {
    SERVICE_UUID = "d7aa9e26-3527-416a-aaee-c7b1454642dd";
    CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

    NimBLEDevice::init("We're thinking");
    pserver = NimBLEDevice::createServer();
    pserver->setCallbacks(new ServerCallbacks());
    NimBLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::WRITE_NR |  // Use WRITE_NR (Write Without Response)
        NIMBLE_PROPERTY::INDICATE    // Use INDICATE instead of NOTIFY if needed
    );
    pCharacteristic->setCallbacks(new CharacteristicCallbacks());
    pService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setName("ESP32-Robot");
    pAdvertising->setMinInterval(0x20);  // 0x20 * 0.625ms = 20ms
    pAdvertising->setMaxInterval(0x40);  // 0x40 * 0.625ms = 40ms

    pAdvertising->start();

    Serial.println("BLE server ready");
}

void bluetooth::setup() {
    while(!deviceConnected) {
        delay(1000);
        Serial.println("Waiting for device to connect");
    }
}

void bluetooth::update() {
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
}





class ServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected");
    }
    
    void onDisconnect(NimBLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected");
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