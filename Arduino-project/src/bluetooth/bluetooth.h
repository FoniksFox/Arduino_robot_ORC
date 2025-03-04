#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <ArduinoJson.h>
#include <functional>
#include <queue>

// Command callback function type
typedef std::function<void(const char*, JsonDocument&)> CommandCallback;

class Bluetooth : public NimBLEServerCallbacks, public NimBLECharacteristicCallbacks {
    public:
        Bluetooth(const char* deviceName = "ESP32-Robot");
        
        // Setup and start BLE services
        bool begin();
        
        // Wait for connection with timeout
        bool waitForConnection(unsigned long timeout_ms = 30000);
        
        // Register command handler
        void setCommandCallback(CommandCallback callback);
        
        // Check if device is connected
        bool isConnected() const { return deviceConnected; }
        
        // Send response to client
        void sendResponse(const char* status, const char* message = nullptr);
        
        // NimBLE Callback implementations
        void onConnect(NimBLEServer* pServer) override;
        void onDisconnect(NimBLEServer* pServer) override;
        void onWrite(NimBLECharacteristic* pCharacteristic) override;

        virtual void processOrder(StaticJsonDocument<200> doc) = 0;

        void processQueue();

        std::string deviceName;

        // BLE related variables
        std::string SERVICE_UUID;
        std::string CHARACTERISTIC_UUID;
        std::queue<String> commandQueue;
        NimBLEServer* pServer;
        NimBLECharacteristic* pCharacteristic;
        
        // State variables
        bool deviceConnected;
        bool oldDeviceConnected;
        unsigned long lastUpdateTime;
        unsigned long connectionRetryDelay;
        uint8_t reconnectionAttempts;
        
        // Callback function
        CommandCallback commandCallback;
        
        // Private methods
        void restartAdvertising();
};

#endif // BLUETOOTH_H