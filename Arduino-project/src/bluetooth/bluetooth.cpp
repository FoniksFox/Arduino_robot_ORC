#include "Bluetooth.h"
#include <Arduino.h>

Bluetooth::Bluetooth(const char* name) :
    deviceName(name),
    SERVICE_UUID ("d7aa9e26-3527-416a-aaee-c7b1454642dd"),
    CHARACTERISTIC_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8"),
    deviceConnected(false),
    oldDeviceConnected(false),
    lastUpdateTime(0),
    connectionRetryDelay(500),
    reconnectionAttempts(0),
    commandCallback(nullptr) {
}


bool Bluetooth::begin(){

    // initialize
    NimBLEDevice::init(deviceName);

    // create connection
    pServer = NimBLEDevice::createServer();
    if (!pServer) {
        Serial.println("Failed to create connection");
        return false;
    }

    pServer->setCallbacks(this);

    NimBLEService *pService = pServer->createService(SERVICE_UUID);
    if (!pService) {
        Serial.println("Failed to create BLE service");
        return false;
    }

    // Create characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::WRITE_NR |
        NIMBLE_PROPERTY::INDICATE
    );
   
    if (!pCharacteristic) {
        Serial.println("Failed to create BLE characteristic");
        return false;
    }
   
    pCharacteristic->setCallbacks(this);
    pService->start();
   
    // Configure advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setName(deviceName);
    pAdvertising->setMinInterval(0x20);  // 0x20 * 0.625ms = 20ms
    pAdvertising->setMaxInterval(0x40); // 0x40 * 0.625ms = 40ms
   
   
   
    // Start advertising
    if (!pAdvertising->start()) {
        Serial.println("Failed to start advertising");
        return false;
    }
   
    Serial.print("BLE server ready: ");
    Serial.println(deviceName.c_str());
    return true;

}

bool Bluetooth::waitForConnection(unsigned long timeout_ms) {
    unsigned long startTime = millis();
   
    Serial.print("Waiting for BLE connection");
    while (!deviceConnected) {
        delay(500);
        Serial.print(".");
       
        if (timeout_ms > 0 && (millis() - startTime > timeout_ms)) {
            Serial.println("\nConnection timeout");
            return false;
        }
    }
   
    Serial.println("\nDevice connected!");
    return true;
}


void Bluetooth::setCommandCallback(CommandCallback callback) {
    commandCallback = callback;
}


void Bluetooth::sendResponse(const char* status, const char* message) {
    if (!deviceConnected) return;
   
    StaticJsonDocument<256> response;
    response["status"] = status;
    if (message) {
        response["message"] = message;
    }
   
    String jsonString;
    serializeJson(response, jsonString);
   
    pCharacteristic->setValue(jsonString.c_str());
    pCharacteristic->notify();
}


void Bluetooth::onConnect(NimBLEServer* pServer) {
    deviceConnected = true;
    reconnectionAttempts = 0;
    connectionRetryDelay = 500;
    Serial.println("Device connected");
}


void Bluetooth::onDisconnect(NimBLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
}


void Bluetooth::restartAdvertising() {
    reconnectionAttempts++;
    if (reconnectionAttempts > 5) {
        connectionRetryDelay = min(30000UL, connectionRetryDelay * 2);
    }
   
    // Restart advertising
    NimBLEDevice::getAdvertising()->start();
    Serial.print("Restarted advertising. Retry delay: ");
    Serial.print(connectionRetryDelay);
    Serial.println("ms");
}


void Bluetooth::onWrite(NimBLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
   
    if (value.length() == 0) {
        Serial.println("Received empty command");
        sendResponse("error", "Empty command");
        return;
    }
   
    Serial.print("Queued command: ");
    Serial.println(value.c_str());


    commandQueue.push(value.c_str());
}


void Bluetooth::processQueue() {
    if (!commandQueue.empty()) {
        String commandJson = commandQueue.front();
        commandQueue.pop(); 

        DynamicJsonDocument doc(512);
        DeserializationError error = deserializeJson(doc, commandJson);


        if (error) {
            Serial.print("JSON parse error: ");
            Serial.println(error.c_str());
            return;
        }


        processOrder(doc);

        delay(10);  
    }
}
