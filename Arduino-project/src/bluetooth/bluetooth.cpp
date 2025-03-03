#include "Bluetooth.h"
#include <Arduino.h>

Bluetooth::Bluetooth() {
    deviceConnected = false;
    lastSendTime = 0;

    SERVICE_UUID = "d7aa9e26-3527-416a-aaee-c7b1454642dd";
    CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8";

    NimBLEDevice::init("ESP32-Robot");
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(this);
    NimBLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::WRITE_NR |  // Use WRITE_NR (Write Without Response)
        NIMBLE_PROPERTY::INDICATE    // Use INDICATE instead of NOTIFY if needed
    );
    pCharacteristic->setCallbacks(this);
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

void Bluetooth::setup() {
    while(!deviceConnected) {
        delay(1000);
        Serial.println("Waiting for device to connect");
    }
}

void Bluetooth::send(StaticJsonDocument<200> doc) {
    if (millis() - lastSendTime < 100) return;
    if (deviceConnected) {
        String jsonString;
        serializeJson(doc, jsonString);
        pCharacteristic->setValue(jsonString.c_str());
        pCharacteristic->indicate();
    }
}

void Bluetooth::onConnect(NimBLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
}

void Bluetooth::onDisconnect(NimBLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
    delay(500);
    NimBLEDevice::getAdvertising()->start();  // Correct restart method
}

void Bluetooth::onWrite(NimBLECharacteristic* pCharacteristic) {
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

        // Command handling
        processOrder(doc);
    }
}