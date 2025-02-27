#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <ArduinoJson.h>

class bluetooth {
    public:
        void static init();
        void static setup();
        void static update();

    private:
        static std::string SERVICE_UUID;
        static std::string CHARACTERISTIC_UUID;
        static NimBLEServer* pServer;
        static NimBLECharacteristic* pCharacteristic;
        static bool deviceConnected;
};