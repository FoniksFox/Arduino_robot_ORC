#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEServer.h>
#include <NimBLEUtils.h>
#include <ArduinoJson.h>

class Bluetooth : public NimBLEServerCallbacks, public NimBLECharacteristicCallbacks {
    public:
        Bluetooth();
        void setup();
        void send(StaticJsonDocument<200> doc);

        void onConnect(NimBLEServer* pServer) override;
        void onDisconnect(NimBLEServer* pServer) override;
        void onWrite(NimBLECharacteristic* pCharacteristic) override;
        void virtual processOrder(StaticJsonDocument<200> doc) = 0;

    private:
        std::string SERVICE_UUID;
        std::string CHARACTERISTIC_UUID;
        NimBLEServer* pServer;
        NimBLECharacteristic* pCharacteristic;
        bool deviceConnected;
        unsigned long lastSendTime;

};