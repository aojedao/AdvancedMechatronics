
/*
 * ESP32 BLE Differential Drive Robot Controller
 * ---------------------------------------------
 * Receives velocity commands (linear, angular) via BLE from a ROS2 node,
 * and controls two motors (differential drive).
 *
 * Author: Santiago Bernheim (NYU Tandon, Fulbright)
 * Date: May 2025
 */

 #include <Arduino.h>
 #include <BLEDevice.h>
 #include <BLEServer.h>
 #include <BLEUtils.h>
 #include <BLE2902.h>
 
 // ==================== PARAMETRIC CONFIGURATION ====================
 #define BOT_NAME            "BLE_Device_1"   // or "BLE_Device_2" for the second bot
 #define BLE_SERVICE_UUID    "0000ffe1-0000-1000-8000-00805f9b34fb"
 #define BLE_CHAR_UUID_RX    "0000ffe1-0000-1000-8000-00805f9b34fb"
 #define BLE_CHAR_UUID_TX "0000ffe2-0000-1000-8000-00805f9b34fb"

 // TX characteristic can be omitted or set to another UUID if needed
 
 #define SERIAL_BAUD         115200
 #define DEBUG_PRINT         1               // Set to 0 to disable Serial debug
 // ===================================================================
 
 // Forward declarations
 void processCommand(const String& cmd);
 
 // BLE globals
 BLEServer* pServer = nullptr;
 BLECharacteristic* pTxCharacteristic = nullptr;
 bool deviceConnected = false;
 bool oldDeviceConnected = false;
 unsigned long lastConnectionAttempt = 0;
 const unsigned long RECONNECT_INTERVAL = 2000; // ms
 
 // ==================== BLE CALLBACKS ====================
 class ServerCallbacks : public BLEServerCallbacks {
     void onConnect(BLEServer* pServer) override {
         deviceConnected = true;
 #if DEBUG_PRINT
         Serial.println("[BLE] Client connected");
 #endif
     }
     void onDisconnect(BLEServer* pServer) override {
         deviceConnected = false;
 #if DEBUG_PRINT
         Serial.println("[BLE] Client disconnected");
 #endif
     }
 };
 
 class RxCallbacks : public BLECharacteristicCallbacks {
     void onWrite(BLECharacteristic* pCharacteristic) override {
         std::string rxValue = pCharacteristic->getValue();
         if (rxValue.length() > 0) {
             String cmd = String(rxValue.c_str());
 #if DEBUG_PRINT
             Serial.print("[BLE] Received: ");
             Serial.println(cmd);
 #endif
             processCommand(cmd);
         }
     }
 };
 
 // ==================== COMMAND PROCESSING ====================
 void processCommand(const String& cmd) {
     // Expected format: "x.xx,y.yy,z.zz\n"
     float x = 0, y = 0, z = 0;
     int num = sscanf(cmd.c_str(), "%f,%f,%f", &x, &y, &z);
     if (num == 3) {
 #if DEBUG_PRINT
         Serial.printf("[CMD] Parsed: x=%.2f, y=%.2f, z=%.2f\n", x, y, z);
 #endif
         // TODO: Actuate motors or perform actions here
     } else {
 #if DEBUG_PRINT
         Serial.println("[CMD] Invalid command format");
 #endif
     }
 }
 
 // ==================== BLE SETUP ====================
 void setupBLE() {
     BLEDevice::init(BOT_NAME);
     pServer = BLEDevice::createServer();
     pServer->setCallbacks(new ServerCallbacks());
 
     BLEService* pService = pServer->createService(BLE_SERVICE_UUID);
 
     // RX Characteristic (Write)
     BLECharacteristic* pRxCharacteristic = pService->createCharacteristic(
         BLE_CHAR_UUID_RX,
         BLECharacteristic::PROPERTY_WRITE
     );
     pRxCharacteristic->setCallbacks(new RxCallbacks());
 
     // TX Characteristic (Notify)
     pTxCharacteristic = pService->createCharacteristic(
         BLE_CHAR_UUID_TX,
         BLECharacteristic::PROPERTY_NOTIFY
     );
     pTxCharacteristic->addDescriptor(new BLE2902());
 
     pService->start();
 
     // Start advertising
     BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
     pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
     pAdvertising->setScanResponse(false);
     pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
     pAdvertising->setMinPreferred(0x12);
     BLEDevice::startAdvertising();
 
 #if DEBUG_PRINT
     Serial.println("[BLE] Advertising started");
 #endif
 }
 
 // ==================== ARDUINO SETUP/LOOP ====================
 void setup() {
 #if DEBUG_PRINT
     Serial.begin(SERIAL_BAUD);
     Serial.println("\n[BOOT] ESP32 BLE Bot Receiver Starting...");
 #endif
     setupBLE();
 }
 
 void loop() {
     // Handle BLE reconnection
     if (!deviceConnected && oldDeviceConnected) {
         // Client disconnected, restart advertising after a delay
         delay(500);
         BLEDevice::startAdvertising();
 #if DEBUG_PRINT
         Serial.println("[BLE] Restarted advertising");
 #endif
         oldDeviceConnected = deviceConnected;
     }
     if (deviceConnected && !oldDeviceConnected) {
         // Just connected
         oldDeviceConnected = deviceConnected;
     }
 
     // Optionally, add periodic tasks or motor control here
 
     delay(10); // Small delay to avoid busy loop
 }
 