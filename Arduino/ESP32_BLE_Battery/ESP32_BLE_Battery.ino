#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <INA226.h>           // Rob Tillaart's library
#include <esp_sleep.h>
#include "driver/rtc_io.h"    // for rtc_gpio_* pull configuration

// ===================== CONFIG =====================
#define DEVICE_NUM              4          // 4 / 5 / 6
#define WAKE_UP_BUTTON_PIN      33         // RTC IO PIN! (32–39, 0,2,4,12–15,25–27)
#define USE_EXT0_WAKEUP         1          // 1 = EXT0 (single pin), 0 = EXT1 (mask)
#define SLEEP_TIMEOUT_MS        60000      // 1 min idle (no BLE connection) -> sleep
#define WAKE_INTERVAL_MS        120000     // timer wake every 2 minutes (optional)
#define COUNTDOWN_SECONDS       10

// ---- INA226 / SHUNT CONFIG ----
// Your physical shunt is nominally 0.005 Ω, BUT DMM says 0.475A while chip showed 0.59A.
// Refined shunt = 0.005 * (0.59 / 0.475) ≈ 0.00621 Ω
#define SHUNT_NOMINAL_OHMS      0.00500f
#define SHUNT_FOR_CAL           0.00621f   // <-- use this for configure() so current matches DMM
#define MAX_CURRENT_A           20.0f      // target full-scale (not used by configure path)

// Manual calibration choices (safe defaults)
#define INA_LSB_mA_PRIMARY      0.625f     // try 0.625 mA/bit first (supports up to ~20A)
#define INA_LSB_mA_FALLBACK     1.000f     // fallback if library complains
#define INA_I_OFFSET_mA         0.0f       // zero-current offset (refine later from no-load average)
#define INA_V_SCALE_E4          10000      // bus voltage scale factor (10000 = 1.0000x)
// =================================================

// I2C pins (classic ESP32 defaults)
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Sensors
Adafruit_AHTX0 aht;
INA226 ina226(0x40, &Wire);   // will be reassigned internally if another addr is detected

// BLE UUIDs (match Flutter)
#define SERVICE_UUID        "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define CHARACTERISTIC_UUID "cba1d466-344c-4be3-ab3f-189daa0a16d8"

// BLE globals
BLEServer* pServer = nullptr;
BLECharacteristic* pDataCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;
String bleDeviceName;

// State
unsigned long lastConnectionTime = 0;

struct SensorReadings {
  float temperature;
  float current;   // A
  float voltage;   // V
  float humidity;   // V
  bool valid;
};

// --- BLE Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
    Serial.println("Device connected");
  }
  void onDisconnect(BLEServer*) override {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

// Prototypes
bool initializeHardware();
void initializeBLE();
SensorReadings getSensorReadings();
void updateOLEDDisplay(SensorReadings readings, const String& status);
void print_wakeup_reason();
void enterDeepSleep();
void scanI2C();

void setup() {
  Serial.begin(115200);
  delay(300);

  print_wakeup_reason();

  if (!initializeHardware()) {
    Serial.println("Failed to initialize hardware. Halting.");
    while (true) delay(1000);
  }

  initializeBLE();

  pinMode(WAKE_UP_BUTTON_PIN, INPUT);  // external pulls recommended
  lastConnectionTime = millis();

  Serial.println("Setup complete. Advertising...");
}

void loop() {
  // Restart advertising after disconnect
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    BLEDevice::startAdvertising();
    Serial.println("Restarted advertising");
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected) {
    lastConnectionTime = millis();

    SensorReadings readings = getSensorReadings();
    if (readings.valid) {
      updateOLEDDisplay(readings, "Connected");

      char dataBuffer[64];
      snprintf(dataBuffer, sizeof(dataBuffer),
               "t=%.2f,a=%.2f,v=%.2f,h=%.2f",
               readings.temperature, readings.current, readings.voltage, readings.humidity);
      pDataCharacteristic->setValue(dataBuffer);
      pDataCharacteristic->notify();
    }
  } else {
    // Not connected
    if (millis() - lastConnectionTime > SLEEP_TIMEOUT_MS) {
      // countdown on OLED before sleeping
      for (int i = COUNTDOWN_SECONDS; i > 0; --i) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.println("Sleeping in...");
        display.setTextSize(2);
        display.setCursor(40, 28);
        display.println(i);
        display.display();
        delay(1000);
        if (deviceConnected) break;
      }
      if (!deviceConnected) {
        enterDeepSleep();
      }
    } else {
      SensorReadings readings = getSensorReadings();
      if (readings.valid) {
        updateOLEDDisplay(readings, "Advertising...");
      }
    }
  }

  delay(2000);
}

// ================== IMPLEMENTATION ==================

void scanI2C()
{
  Serial.println("I2C scan start...");
  byte count = 0;
  for (uint8_t address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  - Found I2C 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      count++;
    }
  }
  if (count == 0) Serial.println("  (no devices found)");
  Serial.println("I2C scan done.");
}

bool initializeHardware() {
  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000); // 100 kHz for stability on long wires / multiple devices
  delay(20);

  // OLED @ 0x3C
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    return false;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.display();

  // I2C scan to confirm all addresses
  scanI2C();  // expect 0x38 (AHT), 0x3C (OLED), 0x40 (INA226)

  // AHT10/AHT20 @ 0x38
  if (!aht.begin()) {
    Serial.println("Failed to find AHT10/AHT20 at 0x38 (check wiring/power).");
    return false;
  }

  // Probe INA226 0x40..0x45 (A0/A1 strap)
  uint8_t inaAddr = 0;
  for (uint8_t addr = 0x40; addr <= 0x45; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { inaAddr = addr; break; }
  }
  if (inaAddr == 0) {
    Serial.println("INA226 not found on 0x40..0x45. Check A0/A1 jumpers, power, and SDA/SCL.");
    return false;
  }

  // Recreate INA226 instance with detected address (keeps your symbol name)
  static INA226 _inaTemp(0x40, &Wire);
  *(INA226*)&ina226 = INA226(inaAddr, &Wire);

  if (!ina226.begin()) {
    Serial.println("INA226 begin() failed after address detection.");
    return false;
  }
  delay(10);

#ifdef INA226_LIB_VERSION
  Serial.print("INA226_LIB_VERSION: ");
  Serial.println(INA226_LIB_VERSION);
#endif
  Serial.print("MAN: 0x"); Serial.println(ina226.getManufacturerID(), HEX);
  Serial.print("DIE: 0x"); Serial.println(ina226.getDieID(), HEX);

  // ---------- MANUAL CALIBRATION (robust) ----------
  // Use refined shunt so current matches DMM.
  int rc = ina226.configure(
    SHUNT_FOR_CAL,        // refined shunt (Ohm)
    INA_LSB_mA_PRIMARY,   // 0.625 mA/bit
    INA_I_OFFSET_mA,      // zero-current offset (mA)
    INA_V_SCALE_E4        // bus voltage scaling (x / 10000)
  );

  if (rc != 0) {
    Serial.print("INA226.configure(shunt=");
    Serial.print(SHUNT_FOR_CAL, 6);
    Serial.print(", LSB=");
    Serial.print(INA_LSB_mA_PRIMARY, 3);
    Serial.print(" mA) failed, rc=");
    Serial.println(rc);

    // Fallback to 1.0 mA/bit (coarser but very tolerant)
    rc = ina226.configure(
      SHUNT_FOR_CAL,
      INA_LSB_mA_FALLBACK,
      INA_I_OFFSET_mA,
      INA_V_SCALE_E4
    );

    if (rc != 0) {
      Serial.print("INA226.configure(L=1.000 mA) failed, rc=");
      Serial.println(rc);
      Serial.println(">> Re-check wiring; try LSB 1.0–2.5 mA or adjust SHUNT_FOR_CAL slightly.");
      return false;
    } else {
      Serial.println("Configured INA226 with LSB = 1.000 mA/bit (fallback).");
    }
  } else {
    Serial.println("Configured INA226 with LSB = 0.625 mA/bit (refined shunt).");
  }

  // ---- Make INA226 actually measure continuously ----
  // These APIs exist in recent Rob Tillaart INA226 versions
  ina226.setAverage(64);                     // 1,4,16,64,128,... balance noise/speed
  ina226.setBusVoltageConversionTime(1100);  // µs (140..8244). 1100 is good middle ground
  ina226.setShuntVoltageConversionTime(1100);
  ina226.setMode(7); // continuous shunt + bus (AKA INA226_MODE_SHUNT_BUS_CONT)
  delay(5);

  // Print calibration summary
  Serial.print("isCalibrated: ");
  Serial.println(ina226.isCalibrated() ? "YES" : "NO");
  Serial.print("CurrentLSB (A): ");
  Serial.println(ina226.getCurrentLSB(), 6);
  Serial.print("CurrentLSB (mA): ");
  Serial.println(ina226.getCurrentLSB_mA(), 3);
  Serial.print("Shunt (Ohm): ");
  Serial.println(ina226.getShunt(), 6);
  Serial.print("MaxCurrent (A): ");
  Serial.println(ina226.getMaxCurrent(), 3);

  // Quick sanity prints (optional)
  for (int i = 0; i < 5; i++) {
    float bv = ina226.getBusVoltage();
    float sv = ina226.getShuntVoltage_mV();
    float im = ina226.getCurrent_mA();
    Serial.print("Bus="); Serial.print(bv, 3); Serial.print(" V  ");
    Serial.print("Shunt="); Serial.print(sv, 3); Serial.print(" mV  ");
    Serial.print("I="); Serial.print(im, 1); Serial.println(" mA");
    delay(300);
  }

  Serial.print("INA226 OK at 0x");
  Serial.println(inaAddr, HEX);
  Serial.println("Hardware initialized OK.");
  return true;
}

void initializeBLE() {
  bleDeviceName = String("BATT-Mon_") + String(DEVICE_NUM);
  BLEDevice::init(bleDeviceName.c_str());

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());
  pServer = server;

  BLEService* svc = pServer->createService(SERVICE_UUID);
  pDataCharacteristic = svc->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pDataCharacteristic->addDescriptor(new BLE2902());
  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  Serial.print("Advertising as: ");
  Serial.println(bleDeviceName);
}

SensorReadings getSensorReadings() {
  SensorReadings r;
  r.valid = false;

  sensors_event_t humidity, temp;
  if (aht.getEvent(&humidity, &temp)) {
    r.temperature = temp.temperature;
    r.humidity    = humidity.relative_humidity;
    r.voltage     = ina226.getBusVoltage();          // Volts
    r.current     = ina226.getCurrent_mA() / 1000.0; // Amps (library returns mA)
    r.valid       = true;
  } else {
    Serial.println("Failed to read AHT sensor");
  }
  return r;
}

void updateOLEDDisplay(SensorReadings readings, const String& status) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(bleDeviceName);
  display.print("Status: ");
  display.println(status);

  display.setCursor(0, 20);
  display.printf("V: %.2f V\n", readings.voltage);
  display.printf("A: %.2f A\n", readings.current);
  display.printf("T: %.2f C\n", readings.temperature);
  display.printf("H: %.2f RH\n", readings.humidity);
  display.display();
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO (EXT0)");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL (EXT1)");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}

void enterDeepSleep() {
  Serial.println("Entering deep sleep...");

  // Turn off OLED
  display.clearDisplay();
  display.display();
  display.ssd1306_command(SSD1306_DISPLAYOFF);

  // Timer wake (comment out if you only want button wake)
  esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_MS * 1000ULL);

#if USE_EXT0_WAKEUP
  // EXT0 (single pin, level wake)
  gpio_num_t WAKEUP_GPIO = (gpio_num_t)WAKE_UP_BUTTON_PIN;
  // Wake on HIGH level; keep pin LOW during sleep with pulldown.
  esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, 1); // 1 = HIGH, 0 = LOW
  rtc_gpio_pullup_dis(WAKEUP_GPIO);
  rtc_gpio_pulldown_en(WAKEUP_GPIO);
#else
  // EXT1 (mask, any-high)
  uint64_t mask = (1ULL << WAKE_UP_BUTTON_PIN);
  esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  rtc_gpio_pulldown_en((gpio_num_t)WAKE_UP_BUTTON_PIN);
  rtc_gpio_pullup_dis((gpio_num_t)WAKE_UP_BUTTON_PIN);
#endif

  esp_deep_sleep_start();
  // never returns
}
