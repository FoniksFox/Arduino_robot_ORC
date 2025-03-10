#include "Bluetooth.h"
#include <Arduino.h>
#define BATTERY_PIN 34  // Analog pin to read battery voltage
#define BATTERY_MIN 3200  // Minimum battery voltage in mV (empty)
#define BATTERY_MAX 4200  // Maximum battery voltage in mV (fully charged)

Bluetooth::Bluetooth(const char* deviceName) :
    deviceName(deviceName),
    SERVICE_UUID("d7aa9e26-3527-416a-aaee-c7b1454642dd"),
    CHARACTERISTIC_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a8"),
    CONSOLE_SERVICE_UUID("d7aa9e27-3527-416a-aaee-c7b1454642dd"),
    CONSOLE_TX_CHAR_UUID("beb5483e-36e1-4688-b7f5-ea07361b26a9"),
    CONSOLE_RX_CHAR_UUID("beb5483e-36e1-4688-b7f5-ea07361b26aa"),
    deviceConnected(false),
    oldDeviceConnected(false),
    lastUpdateTime(0),
    connectionRetryDelay(500),
    reconnectionAttempts(0),
    pConsoleTxChar(nullptr),
    pConsoleRxChar(nullptr) {
}

bool Bluetooth::begin() {
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
    
    // Add console service
    NimBLEService *pConsoleService = pServer->createService(CONSOLE_SERVICE_UUID);
    if (!pConsoleService) {
        Serial.println("Failed to create BLE console service");
        return false;
    }
    
    // Create console TX characteristic (robot -> app)
    pConsoleTxChar = pConsoleService->createCharacteristic(
        CONSOLE_TX_CHAR_UUID,
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::NOTIFY
    );
    
    if (!pConsoleTxChar) {
        Serial.println("Failed to create BLE console TX characteristic");
        return false;
    }
    
    // Create console RX characteristic (app -> robot)
    pConsoleRxChar = pConsoleService->createCharacteristic(
        CONSOLE_RX_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE
    );
    
    if (!pConsoleRxChar) {
        Serial.println("Failed to create BLE console RX characteristic");
        return false;
    }
    
    pConsoleRxChar->setCallbacks(this);
    pConsoleService->start();
   
    // Configure advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->addServiceUUID(CONSOLE_SERVICE_UUID);
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
    
    // Check if this is a console command
    if (pCharacteristic->getUUID().equals(CONSOLE_RX_CHAR_UUID)) {
        Serial.print("Received console command: ");
        Serial.println(value.c_str());
        processConsoleCommand(value);
    } else {
        // Regular command processing
        Serial.print("Queued command: ");
        Serial.println(value.c_str());
        commandQueue.push(value.c_str());
    }
}

void Bluetooth::processQueue() {
    if (!commandQueue.empty()) {
        String commandJson = commandQueue.front();
        commandQueue.pop(); 

        StaticJsonDocument<200> doc;
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

void Bluetooth::processConsoleCommand(const std::string& command) {
    sendConsoleMessage(("> " + command).c_str(), "command");
    
    if (command == "help") {
        sendConsoleMessage("Available commands:", "info");
        sendConsoleMessage("  help - Show this help message", "info");
        sendConsoleMessage("  status - Show robot status", "info");
        sendConsoleMessage("  reset - Reset the robot", "info");
    } else if (command == "status") { // test command
        sendConsoleMessage("Robot status:", "Yeh, its aight");
    } else if (command == "reset") {
        sendConsoleMessage("Resetting robot...", "warning");
        // Implement reset (prob not needed, but if i have time)
        delay(1000);
        sendConsoleMessage("Robot reset complete", "info");
    } else {
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, command);
        if (!error) {
            processOrder(doc);
        } else {
            // Not a json
            sendConsoleMessage("Unknown command or invalid format", "error");
        }
    }
}

void Bluetooth::sendConsoleMessage(const char* message, const char* level) {
    if (!deviceConnected || !pConsoleTxChar) return;
    
    StaticJsonDocument<256> consoleMsg;
    consoleMsg["message"] = message;
    consoleMsg["level"] = level;  
    
    String jsonString;
    serializeJson(consoleMsg, jsonString);
    
    // Add message to queue
    consoleQueue.push(jsonString.c_str());
    
    // Limit queue size
    while (consoleQueue.size() > MAX_CONSOLE_QUEUE_SIZE) {
        consoleQueue.pop();
    }
    processConsoleQueue();
}

void Bluetooth::processConsoleQueue() {
    while (!consoleQueue.empty()) {
        std::string msg = consoleQueue.front();
        consoleQueue.pop();
        
        pConsoleTxChar->setValue(msg);
        pConsoleTxChar->notify();
        delay(20);
    }
}

void consoleLog(const char* message) {
    Serial.println(message);  
}

void Bluetooth::sendBatteryLevel(int level) {
    if (!deviceConnected || !pCharacteristic) return;
    
    StaticJsonDocument<64> batteryData;
    batteryData["type"] = "battery";
    batteryData["level"] = level;
    
    String jsonString;
    serializeJson(batteryData, jsonString);
    
    pCharacteristic->setValue(jsonString.c_str());
    pCharacteristic->notify();
    
    // Also send to console for debugging
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "Battery level: %d%%", level);
    sendConsoleMessage(buffer, "info");
}

void Bluetooth::processBatteryReadings() {
    // Check if it's time to update battery reading
    unsigned long currentTime = millis();
    if (currentTime - lastBatteryUpdate < BATTERY_UPDATE_INTERVAL) {
        return;
    }
    
    lastBatteryUpdate = currentTime;
    int rawValue = analogRead(BATTERY_PIN);
    float voltage = rawValue * 3.3 / 4095.0;
    int batteryMv = int(voltage * 1000.0); 
    int batteryPercentage = map(batteryMv, BATTERY_MIN, BATTERY_MAX, 0, 100);
    
    // Constrain to valid range
    batteryPercentage = constrain(batteryPercentage, 0, 100);
    
    // Send the battery level
    if (deviceConnected) {
        sendBatteryLevel(batteryPercentage);
    }
    
    // Debug output
    Serial.print("Battery raw: ");
    Serial.print(rawValue);
    Serial.print(", Voltage: ");
    Serial.print(voltage);
    Serial.print("V, Percentage: ");
    Serial.print(batteryPercentage);
    Serial.println("%");
}