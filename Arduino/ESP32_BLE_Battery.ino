#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- CONFIGURATION ---
// CHANGE THIS NUMBER FOR EACH ESP32 BEFORE UPLOADING
#define DEVICE_NUM 5 // Set this to 4, 5, or 6
// --------------------

// UUIDs for the Service and Characteristic
#define SERVICE_UUID        "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189daa0a16d8"

// Global variables
BLEServer* pServer = NULL;
BLECharacteristic* pDataCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
char bleDeviceName[32]; // Buffer to hold the dynamic device name

// Callback class for handling server connection events
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Device disconnected");
    }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Multi-Parameter Sensor!");

  // Seed the random number generator
  randomSeed(analogRead(A0));

  // Create the dynamic BLE device name
  snprintf(bleDeviceName, sizeof(bleDeviceName), "BATT-Mon_%d", DEVICE_NUM);
  Serial.print("BLE Device Name: ");
  Serial.println(bleDeviceName);

  // Create the BLE Device
  BLEDevice::init(bleDeviceName);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic for our data string
  pDataCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Add the descriptor for notifications
  pDataCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {
  // Check connection status
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Restarted advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // If a device is connected, read sensors and send data
  if (deviceConnected) {
    // 1. Get the real CPU temperature
    float temp_celsius = temperatureRead();

    // 2. Generate random current value (e.g., 0.50A to 2.50A)
    float current_amps = (float)random(50, 250) / 100.0;

    // 3. Generate random voltage value (e.g., 11.0V to 13.0V)
    float voltage_volts = (float)random(1100, 1300) / 100.0;

    // 4. Format the data into a single key-value string
    char dataBuffer[48];
    snprintf(dataBuffer, sizeof(dataBuffer), "t=%.2f,a=%.2f,v=%.2f", temp_celsius, current_amps, voltage_volts);

    // 5. Set the characteristic's value and notify the client
    pDataCharacteristic->setValue(dataBuffer);
    pDataCharacteristic->notify();

    // 6. Print to Serial Monitor for debugging
    Serial.print("Sent data: ");
    Serial.println(dataBuffer);
  }
  
  delay(2000); // Wait for 2 seconds
}