#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <ArduinoJson.h>
#include <functional>
#include <queue>
#include <string>

// Command callback function type
typedef std::function<void(const char*, JsonDocument&)> CommandCallback;

class Bluetooth : public NimBLEServerCallbacks, public NimBLECharacteristicCallbacks {
    public:
        Bluetooth(const char* deviceName = "we're thinking");
        
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
        
        // Console-related methods
        void sendConsoleMessage(const char* message, const char* level = "standard");
        void processConsoleCommand(const std::string& command);
        
        // NimBLE Callback implementations
        void onConnect(NimBLEServer* pServer) override;
        void onDisconnect(NimBLEServer* pServer) override;
        void onWrite(NimBLECharacteristic* pCharacteristic) override;

        virtual void processOrder(StaticJsonDocument<200> doc) = 0;

        void processQueue();
        void processConsoleQueue();
        void sendBatteryLevel(int level);
        void processBatteryReadings();
        unsigned long lastBatteryUpdate;
        const unsigned long BATTERY_UPDATE_INTERVAL = 5000;

        std::string deviceName;

    protected:
        // BLE related variables
        std::string SERVICE_UUID;
        std::string CHARACTERISTIC_UUID;
        
        // Console-specific UUIDs
        std::string CONSOLE_SERVICE_UUID;
        std::string CONSOLE_TX_CHAR_UUID;  // For messages from robot to app
        std::string CONSOLE_RX_CHAR_UUID;  // For commands from app to robot
        
        std::queue<String> commandQueue;
        NimBLEServer* pServer;
        NimBLECharacteristic* pCharacteristic;
        NimBLECharacteristic* pConsoleTxChar;
        NimBLECharacteristic* pConsoleRxChar;
        
        // State variables
        bool deviceConnected;
        bool oldDeviceConnected;
        unsigned long lastUpdateTime;
        unsigned long connectionRetryDelay;
        uint8_t reconnectionAttempts;
        
        // Callback function
        CommandCallback commandCallback;
        
        // Console message queue
        std::queue<std::string> consoleQueue;
        const size_t MAX_CONSOLE_QUEUE_SIZE = 50;
        
        // Private methods
        void restartAdvertising();
};

void consoleLog(const char* message);

#endif // BLUETOOTH_H