#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define BLE_SERVER_NAME "Robot-BLE-IMU"

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define POSE_CHAR_UUID      "19B10003-E8F2-537E-4F6C-D104768A1214"
#define WASD_CHAR_UUID      "19B10002-E8F2-537E-4F6C-D104768A1214"

#define ROT_TIME_UINT_UUID  "19B10004-E8F2-537E-4F6C-D104768A1214"
#define LIN_TIME_UINT_UUID  "19B10005-E8F2-537E-4F6C-D104768A1214"
#define MOTORSPEED_UINT_UUID "19B10006-E8F2-537E-4F6C-D104768A1214"

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicWASD = NULL; // For receiving commands
BLECharacteristic *pCharacteristicPOSE = NULL; // For sending pose updates
BLECharacteristic *pCharacteristicRotTime = NULL; // For receiving rotation time
BLECharacteristic *pCharacteristicLinTime = NULL; // For receiving linear time
BLECharacteristic *pCharacteristicMotorSpeed = NULL; // For receiving motor speed

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

// BLE Characteristic Callbacks to handle incoming data
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      char command = rxValue[0]; // Assume single-character commands
      Serial.print("Received via BLE: ");
      Serial.println(command);

      // Forward the command to the Propeller via UART
      Propeller.println(command);

      // Handle special case for 'Q' (reset pose)
      if (command == 'Q') {
        String resetPose = "Theta: 0.00; X: 0.00; Y: 0.00";
        pCharacteristicPOSE->setValue(resetPose.c_str());
        pCharacteristicPOSE->notify();
        Serial.println("Pose reset and sent via BLE");
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize UART1 (GPIO16 RX, GPIO17 TX)
  Propeller.begin(115200, SERIAL_8N1, 16, 17);

  // Initialize BLE
  BLEDevice::init(BLE_SERVER_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // RX Characteristic (ESP32 receives data from BLE client)
  pCharacteristicWASD = pService->createCharacteristic(
                        WASD_CHAR_UUID,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  pCharacteristicWASD->setCallbacks(new MyCallbacks());

  // TX Characteristic (ESP32 sends data to BLE client)
  pCharacteristicPOSE = pService->createCharacteristic(
                        POSE_CHAR_UUID,
                        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
                      );

  pCharacteristicLinTime = pService->createCharacteristic(
                        LIN_TIME_UINT_UUID,
                        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
                      );

  pCharacteristicRotTime = pService->createCharacteristic(
                        ROT_TIME_UINT_UUID,
                        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
                      );

  pCharacteristicMotorSpeed = pService->createCharacteristic(
                        MOTORSPEED_UINT_UUID,
                        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ
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
      pCharacteristicPOSE->setValue(data.c_str());
      pCharacteristicPOSE->notify();
    }
  }

  delay(10); // Small delay to avoid overwhelming the loop
}