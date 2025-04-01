#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicRX = NULL;
BLECharacteristic *pCharacteristicTX = NULL;
bool deviceConnected = false;

// UART1 for Propeller communication
HardwareSerial Propeller(1);

// BLE Server Callbacks to detect connection status
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE Client Connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE Client Disconnected");
    pServer->startAdvertising(); // Restart advertising after disconnect
  }
};

// BLE Characteristic Callbacks to detect incoming data
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.print("Received via BLE: ");
      Serial.println(rxValue.c_str());

      // Forward received BLE data to Propeller via UART
      Propeller.println(rxValue.c_str());
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize UART1 (GPIO16 RX, GPIO17 TX)
  Propeller.begin(115200, SERIAL_8N1, 16, 17);

  // Initialize BLE
  BLEDevice::init("Two-Wheeled Bot");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // RX Characteristic (ESP32 receives data from BLE client)
  pCharacteristicRX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  pCharacteristicRX->setCallbacks(new MyCallbacks());

  // TX Characteristic (ESP32 sends data to BLE client)
  pCharacteristicTX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE Ready");
}

void loop() {
  // Forward data from Propeller (UART) to BLE client
  if (deviceConnected && Propeller.available()) {
    String data = Propeller.readStringUntil('\n'); // Adjust delimiter as needed
    data.trim(); // Remove any trailing newline or spaces

    if (data.length() > 0) {
      Serial.print("Received from Propeller: ");
      Serial.println(data);

      // Send data over BLE
      pCharacteristicTX->setValue(data.c_str());
      pCharacteristicTX->notify();
    }
  }

  delay(10); // Small delay to avoid overwhelming the loop
}