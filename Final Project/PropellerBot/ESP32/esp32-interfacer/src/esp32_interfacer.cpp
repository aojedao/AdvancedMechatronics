#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// BLE configuration (match your ROS bridge)
#define BLE_SERVER_NAME     "Propellerbot-BLE-ROS"
#define SERVICE_UUID        "0000ffe1-0000-1000-8000-00805f9b34fb"
#define CMD_CHAR_UUID       "0000ffe1-0000-1000-8000-00805f9b34fb" // RX

// UART1 for Propeller communication
HardwareSerial Propeller(1);

// --- MAC address print function ---
void printMacToSerial() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.print("ESP32 MAC: ");
  for (int i = 0; i < 6; ++i) {
    if (i > 0) Serial.print(":");
    Serial.printf("%02X", mac[i]);
  }
  Serial.println();
}

// BLE objects
BLEServer *pServer = nullptr;
BLECharacteristic *pCmdChar = nullptr;

bool deviceConnected = false;

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("BLE Client Connected");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("BLE Client Disconnected");
    pServer->startAdvertising();
  }
};

// BLE Characteristic Callbacks
class CmdCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
       // Forward to Propeller as-is (already in tag:value format)
      Propeller.print("cmdvel:"); // Tag for propeller parsing
      Propeller.println(rxValue.c_str());

      Serial.print("Received BLE Command: ");
      Serial.println(rxValue.c_str());
     
      
    }
  }
};

void setup() {
  Serial.begin(115200);
  Propeller.begin(115200, SERIAL_8N1, 14, 27); // RX=14, TX=27 (adjust as needed)
  Serial.println("ESP32-ROS-Propeller Bridge Starting...");

  printMacToSerial(); // <-- Print MAC address at startup

  // BLE setup
  BLEDevice::init(BLE_SERVER_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  // RX Characteristic (receives commands from ROS/PC)
  pCmdChar = pService->createCharacteristic(
    CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pCmdChar->setCallbacks(new CmdCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("BLE Ready, waiting for ROS commands...");
}

void loop() {
  // Forward data from Propeller to BLE client (optional, for odometry/IMU feedback)
  if (deviceConnected && Propeller.available()) {
    String data = Propeller.readStringUntil('\n');
    data.trim();
    if (data.length() > 0) {
      Serial.print("From Propeller: ");
      Serial.println(data);
      // You could add a BLE notify characteristic for feedback if needed
      // For now, just print to Serial
    }
  }
  delay(10);
}
