#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <ArduinoJson.h>

class Bluetooth : public NimBLEServerCallbacks, public NimBLECharacteristicCallbacks {
    public:
        Bluetooth();
        void setup();
        void update(StaticJsonDocument<200> doc);

        void onConnect(NimBLEServer* pServer) override;
        void onDisconnect(NimBLEServer* pServer) override;
        void onWrite(NimBLECharacteristic* pCharacteristic) override;

    private:
        std::string SERVICE_UUID;
        std::string CHARACTERISTIC_UUID;
        NimBLEServer* pServer;
        NimBLECharacteristic* pCharacteristic;
        bool deviceConnected;
        unsigned long lastUpdateTime;

};