#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

HardwareSerial Propeller(1);  // UART1 for Propeller communication

void setup() {
  Serial.begin(115200);
  Propeller.begin(115200, SERIAL_8N1, 16, 17);

  BLEDevice::init("Two-Wheeled Bot");
  pServer = BLEDevice::createServer();
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  
  Serial.println("BLE Ready");
}

void loop() {
  if (deviceConnected) {
    // Read Propeller data
    if (Propeller.available()) {
      String data = Propeller.readStringUntil(' ');
      Serial.println("Received from Propeller: " + data);

      // Send data over BLE
      pCharacteristic->setValue(data.c_str());
      pCharacteristic->notify();
    }
    
    // Receive BLE commands
    if (pCharacteristic->getValue().length() > 0) {
      String command = pCharacteristic->getValue().c_str();
      Serial.println("Command received: " + command);

      // Forward command to Propeller
      Propeller.println(command);
    }
  }
}
