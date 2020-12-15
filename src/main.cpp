#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <WiFi.h>
#include <BLE_UUIDs.h>

class CallbacksFrom_STATE_WIFI_CHAR : public BLECharacteristicCallbacks
{
    void onRead(BLECharacteristic *STATE_WIFI_CHAR)
    {
        Serial.println("WiFi state readed");
    };
};

class CallbacksFrom_SSID_WIFI_CHAR : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *SSID_WIFI_CHAR)
    {
        uint8_t *testDing2;
        testDing2 = SSID_WIFI_CHAR->getData();

        char test1[50];
        uint8_t counter1 = 0;
        do
        {
            test1[counter1] = testDing2[counter1];
            counter1++;
        } while (test1[counter1 - 1] != 0);
        Serial.println("");
        Serial.print("Readed SSID_WIFI: ");
        Serial.println(test1);
    };
};

class CallbacksFrom_PASSWORD_WIFI_CHAR : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *PASSWORD_WIFI_CHAR)
    {
        uint8_t *testDing2;
        testDing2 = PASSWORD_WIFI_CHAR->getData();

        char test1[50];
        uint8_t counter1 = 0;
        do
        {
            test1[counter1] = testDing2[counter1];
            counter1++;
        } while (test1[counter1 - 1] != 0);
        Serial.println("");
        Serial.print("Readed password_WIFI: ");
        Serial.println(test1);
    };
};

class CallbacksFrom_UNIQUE_ID_CHAR : public BLECharacteristicCallbacks
{
    void onRead(BLECharacteristic *UNIQUE_ID_CHAR)
    {
        Serial.println("Unique ID readed");
    };
};

class CallbacksFrom_MEASURE_INTERVAL_CHAR : public BLECharacteristicCallbacks
{
    void onRead(BLECharacteristic *MEASURE_INTERVAL_CHAR)
    {
        Serial.println("MEASURE_INTERVAL readed");
    };
    void onWrite(BLECharacteristic *MEASURE_INTERVAL_CHAR)
    {
        uint8_t *testDing2;
        testDing2 = MEASURE_INTERVAL_CHAR->getData();

        char test1[50];
        uint8_t counter1 = 0;
        do
        {
            test1[counter1] = testDing2[counter1];
            counter1++;
        } while (test1[counter1 - 1] != 0);
        Serial.println("");
        Serial.print("Readed MEASURE_INTERVAL: ");
        Serial.println(test1);
    };
};

void setup()
{
    Serial.begin(115200);

    uint64_t chipid = ESP.getEfuseMac();
    // Serial.print("unique id: ");
    // Serial.printf("%llu\n", chipid);

#define numberOfUniqueDigits 14

    char BLE_name[12 + numberOfUniqueDigits] = "Twomes";
    Serial.print("BLE_name: ");
    Serial.println(BLE_name);
    char BLE_unique[numberOfUniqueDigits];
    sprintf(BLE_unique, "%llu", chipid);
    strcat(BLE_name, BLE_unique);
    Serial.print("BLE_name: ");
    Serial.println(BLE_name);

    BLEDevice::init(BLE_name);

    BLEServer *pServer = BLEDevice::createServer();

    BLEService *WIFI_PROVISIONING_SERVICE = pServer->createService(WIFI_PROVISIONING_SERVICE_UUID);

    BLECharacteristic *SSID_WIFI_CHAR = WIFI_PROVISIONING_SERVICE->createCharacteristic(
        SSID_WIFI_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE);
    SSID_WIFI_CHAR->setValue("thisWillBeTheLenght");

    BLECharacteristic *PASSWORD_WIFI_CHAR = WIFI_PROVISIONING_SERVICE->createCharacteristic(
        PASSWORD_WIFI_UUID,
        BLECharacteristic::PROPERTY_WRITE);
    PASSWORD_WIFI_CHAR->setValue("thisWillBeTheLenght");

    BLECharacteristic *STATE_WIFI_CHAR = WIFI_PROVISIONING_SERVICE->createCharacteristic(
        STATE_WIFI_UUID,
        BLECharacteristic::PROPERTY_READ);
    STATE_WIFI_CHAR->setValue("thisWillBeTheLenght");

    BLEService *CONFIG_PROVISIONING_SERVICE = pServer->createService(CONFIG_PROVISIONING_SERVICE_UUID);

    BLECharacteristic *UNIQUE_ID_CHAR = CONFIG_PROVISIONING_SERVICE->createCharacteristic(
        UNIQUE_ID_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ);
    //UNIQUE_ID_CHAR->setValue((WiFi.macAddress());
    UNIQUE_ID_CHAR->setValue("4e:43:a8:f6:73:32"); //see above

    BLECharacteristic *MEASURE_INTERVAL_CHAR = CONFIG_PROVISIONING_SERVICE->createCharacteristic(
        MEASURE_INTERVAL_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);
    MEASURE_INTERVAL_CHAR->setValue("thisWillBeTheLenght");

    WIFI_PROVISIONING_SERVICE->start();
    CONFIG_PROVISIONING_SERVICE->start();

    BLEAdvertising *pAdvertising = pServer->getAdvertising(); // this still is working for backward compatibility
    //BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(WIFI_PROVISIONING_SERVICE_UUID);
    pAdvertising->addServiceUUID(CONFIG_PROVISIONING_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    BLEDevice::setPower(ESP_PWR_LVL_N12);

    //BLECharacteristicCallbacks pCharacteristicsCallback = new BLECharacteristicCallbacks();

    SSID_WIFI_CHAR->setCallbacks(new CallbacksFrom_SSID_WIFI_CHAR());
    PASSWORD_WIFI_CHAR->setCallbacks(new CallbacksFrom_PASSWORD_WIFI_CHAR());
    STATE_WIFI_CHAR->setCallbacks(new CallbacksFrom_STATE_WIFI_CHAR());

    UNIQUE_ID_CHAR->setCallbacks(new CallbacksFrom_UNIQUE_ID_CHAR());
    MEASURE_INTERVAL_CHAR->setCallbacks(new CallbacksFrom_MEASURE_INTERVAL_CHAR());
}

void loop()
{
}