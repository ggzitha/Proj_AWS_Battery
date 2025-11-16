#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <INA226.h>
#include <esp_sleep.h>
#include "driver/rtc_io.h"

// ===== NimBLE (multi-central) =====
#include <NimBLEDevice.h>

// ===== WiFi / OTA (ESP32) =====
#include <WiFi.h>
#include <ArduinoOTA.h>

// ===================== CONFIG =====================
#define DEVICE_NUM 6          // 4 / 5 / 6 -> used in names
#define WAKE_UP_BUTTON_PIN 2  // Button to GND; we use pull-up and wake on LOW
#define USE_EXT0_WAKEUP 0     // Keep 0 (EXT1/GPIO wake) on C3

#define SLEEP_TIMEOUT_MS 60000   // 1 min idle -> sleep
#define WAKE_INTERVAL_MS 300000  // timer wake every 5 minutes
#define DOT_PERIOD_MS 500
#define OLED_REFRESH_RATE 500  // ms (2 Hz)

// BMS estimate for OLED bars (app does precise logic)
#define SERIES_CELLS 4
#define PARALLEL_CELLS 3
#define PCELL_VMIN 2.70f
#define PCELL_VNOM 3.60f
#define PCELL_VMAX 4.20f

// Kapasitas per sel (default 3400 mAh, sesuaikan dengan pack kamu)
#define PCELL_CAPACITY_mAh 4000.0f

// Kapasitas pack total (Ah) = kapasitas per sel * parallel
static float PACK_CAPACITY_Ah() {
  return (PCELL_CAPACITY_mAh * PARALLEL_CELLS) / 1000.0f;
}

// ---- INA226 / SHUNT CONFIG ----
bool DO_CAL = false;
// shunt (Ohm). Lower shunt = higher measurement range, lower resolution.
#define SHUNT_FOR_CAL 0.00621f
// #define SHUNT_FOR_CAL 0.0055f  // real shunt value: pararel 10 mΩ so it mustbe arround  ≈ 5 mΩ, But this use the test Calibrate use voltsRead = Volts Power SUpply & Multimeter, B \
                               // Because i "GAK ada Multimeter presisi buat ohm", Ini dulu di cari, Baru Cari INA_LSB_mA, pake hitungan !!!!
// current_LSB_mA recommendations: 0.050, 0.100, 0.250, 0.500, 1, 2, 2.5
#define INA_LSB_mA_PRIMARY 0.625f  // Ini I_max(Targeted) nya 0.625 mA×32768≈ 20.48 A,
// #define INA_LSB_mA_PRIMARY 0.625f   // Even if you “calibrate” it for 20 A or 30 A, ADC will clip at 16.38 A (Hardware Limit ?). Calc By :
/* 16-Bit res = 65535 data ≈ –32768 to +32767 === I_max(Targeted) / 32768 
                                              16.384 A / 32768 = 0.0004883 A ≈ 0.0005 A 
                                              satuan mA jadi Current_LSB_A * 1000.0 = 0.0005 * 1000.0 = 0.5
                                            */
#define INA_LSB_mA_FALLBACK 1.000f  // Back-up LSB

// current_zero_offset_mA (in driver). Keep 0; we do user offsets after read.
#define INA_I_OFFSET_mA_CFG 0.0f

// bus_V_scaling_e4 (default 10000 => volts)
#define INA_V_SCALE_E4 10000

// ===== User calibration offsets (apply AFTER sensor read) =====
// B4 = -99.5f | B5 = -79.5f | B6 = -74.5f
#define V_OFFSET_mV -74.5f  // e.g. +25.0f adds +25 mV to pack voltage
// B4 = -39.5f | B5 = -31.0f | B6 = -19.0f
#define I_OFFSET_mA -19.0f  // e.g. -6.0f subtracts 6 mA Paling terakhir, abis ubah DO_CAL = true, ntar dapat ofset, habis dimasukan Ubah lagi DO_CAL = false

// ===== Load compensation + smoothing (SoC under load) =====
// LiitoKala spec: internal resistance <17 mΩ / cell
#define R_CELL_mOHM 18.0f    // per cell
#define R_EXTRA_mOHM 60.0f   // BMS FETs + shunt + wiring (tune)
#define SOC_EMA_ALPHA 0.30f  // 0..1; higher = snappier, lower = smoother

static float R_PACK_OHM() {
  float groupResistance_mOhm = R_CELL_mOHM / PARALLEL_CELLS;         // mΩ per parallel group
  float seriesStack_mOhm = groupResistance_mOhm * SERIES_CELLS;      // mΩ for 4S stack
  float totalPackResistance_mOhm = seriesStack_mOhm + R_EXTRA_mOHM;  // mΩ incl. wiring/BMS
  return totalPackResistance_mOhm / 1000.0f;                         // Ω
}

// I2C pins (ESP32-C3)
#define I2C_SDA_PIN 3
#define I2C_SCL_PIN 4

// OTA AP credentials
#define OTA_PASSWORD "123456789"
#define OTA_PORT 3232
#define OTA_HOLD_MS 5000  // hold time to toggle OTA

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Sensors
Adafruit_AHTX0 aht;
INA226 ina226(0x40, &Wire);

// === BLE UUIDs (match Flutter) ===
static NimBLEUUID SERVICE_UUID("91bad492-b950-4226-aa2b-4ede9fa42f59");
// IMPORTANT: restored original characteristic UUID (ab3f)
static NimBLEUUID CHAR_UUID("cba1d466-344c-4be3-ab3f-189daa0a16d8");

// BLE globals
NimBLEServer* bleServer = nullptr;
NimBLECharacteristic* bleDataCharacteristic = nullptr;
String bleDeviceName;
unsigned long lastConnectionTimeMs = 0;
unsigned long lastNotifyMillis = 0;
unsigned long notifyIntervalMs = 2000;  // Kirim Data tiap 2 detik

// Adv / status animation
uint8_t dotPhase = 0;
unsigned long lastDotTickMs = 0;

// OTA state
bool otaMode = false;
String otaSSID;
IPAddress otaIP(0, 0, 0, 0);

// Button/hold detection (shared for enter/exit OTA)
bool holdInProgress = false;
unsigned long holdStartMs = 0;
int overlayCountdown = -1;      // -1 = no overlay
String headerOverlayText = "";  // "ROM-(4)" ... "ROM-(1)"

// ======== Define SensorReadings BEFORE prototypes ========
struct SensorReadings {
  float temperatureC;     // °C
  float currentA;         // A
  float voltageV;         // V
  float powerW;           // W
  float humidityPercent;  // %RH (still measured, unused on OLED now)
  bool valid;
};

// ===== Averaging state =====
struct AccumulatedSums {
  double sumVoltageV = 0;
  double sumCurrentA = 0;
  double sumTemperatureC = 0;
  double sumPowerW = 0;
  double sumHumidityPct = 0;
  int sampleCount = 0;
} acc1s;

SensorReadings averagedLast = { 0, 0, 0, 0, 0, false };
SensorReadings averagedPrev = { 0, 0, 0, 0, 0, false };
bool have1sWindow = false;
bool have2sWindow = false;
unsigned long lastOLEDms = 0;

// ===== Coulomb Counting state (integrasi arus) =====
double coulombAccumulatedAh = 0.0;  // Ah netto sejak anchor
unsigned long lastCoulombMs = 0;    // timestamp terakhir update CC
bool coulombInitialized = false;    // sudah pernah di-anchor?
float coulombBaseSocPct = 50.0f;    // SoC anchor dari OCV pertama kali

// ======== Prototypes ========
bool initializeHardware();
void initializeBLE();
SensorReadings getInstantReading();
void notifyAll(const SensorReadings& readings);
void updateOLEDDisplay(const SensorReadings& readings);
void drawBatteryBars(float percent);
// ---- SoC helpers ----
float estimateSOCpct(float measuredPackVoltageV);
static float pack_ocv_from_vi(float measuredPackVoltageV, float packCurrentA);
static float soc_from_cell_ocv(float cellVoltageV);
void updateCoulombCounting(const SensorReadings& readings);
float soc_from_coulomb(float socOcv);

void drawCenteredBottom(const String& text);
void print_wakeup_reason();
void enterDeepSleep();
void scanI2C();

void checkOTAToggle();
void startOTA_AP_Mode();
void stopOTA_AP_Mode();
bool runSleepCountdownCancelable(int seconds);

// ================== BLE CALLBACKS ==================
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    Serial.printf("Central connected (handle=%d), total=%d\n",
                  info.getConnHandle(), server->getConnectedCount());
    NimBLEDevice::startAdvertising();
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& info, int reason) override {
    Serial.printf("Central disconnected (handle=%d, reason=%d), total=%d\n",
                  info.getConnHandle(), reason, server->getConnectedCount());
    NimBLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(200);

  print_wakeup_reason();

  if (!initializeHardware()) {
    Serial.println("Failed to initialize hardware. Halting.");
    while (true) delay(1000);
  }

  initializeBLE();

  // Wake pin idle HIGH for wake-on-LOW
  pinMode(WAKE_UP_BUTTON_PIN, INPUT_PULLUP);

  lastConnectionTimeMs = millis();
  lastCoulombMs = millis();  // inisialisasi timestamp CC

  // Optionally allow entering OTA right after boot if held
  checkOTAToggle();

  Serial.println("Setup complete. Advertising...");
}

void loop() {
  // Always watch for 5s hold to enter/exit OTA and drive overlay text
  checkOTAToggle();

  if (otaMode) {
    ArduinoOTA.handle();

    // OTA page on OLED
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("OTA MODE");

    // Show overlay countdown if user is holding to EXIT
    if (overlayCountdown > 0) {
      String exitOverlay = "Exit-(" + String(overlayCountdown) + ")";
      int overlayX = SCREEN_WIDTH - (exitOverlay.length() * 6);
      if (overlayX < 0) overlayX = 0;
      display.setCursor(overlayX, 0);
      display.print(exitOverlay);
    }

    display.setCursor(0, 12);
    display.print("SSID: ");
    display.print(otaSSID);
    display.setCursor(0, 24);
    display.print("PWD : ");
    display.print(OTA_PASSWORD);
    display.setCursor(0, 36);
    display.print("IP  : ");
    display.print(otaIP == IPAddress(0, 0, 0, 0) ? "192.168.4.1" : otaIP.toString());
    display.setCursor(0, 46);
    display.print("Update Via ArduinoIDE");
    display.setCursor(0, 56);
    display.print("Net-PORT or espota.py");
    display.display();

    delay(20);
    return;  // no deep sleep in OTA
  }

  // ---- 1) Take an instantaneous sample (corrected with offsets) ----
  SensorReadings instantReading = getInstantReading();
  if (instantReading.valid) {
    acc1s.sumVoltageV += instantReading.voltageV;
    acc1s.sumCurrentA += instantReading.currentA;
    acc1s.sumPowerW += instantReading.powerW;
    acc1s.sumTemperatureC += instantReading.temperatureC;
    acc1s.sumHumidityPct += instantReading.humidityPercent;
    acc1s.sampleCount += 1;
  }

  // ---- 2) On each OLED_REFRESH_RATE boundary, finalize OLED average ----
  unsigned long nowMs = millis();
  if (nowMs - lastOLEDms >= OLED_REFRESH_RATE) {
    lastOLEDms = nowMs;

    if (acc1s.sampleCount > 0) {
      averagedPrev = averagedLast;

      averagedLast.temperatureC = (float)(acc1s.sumTemperatureC / acc1s.sampleCount);
      averagedLast.currentA = (float)(acc1s.sumCurrentA / acc1s.sampleCount);
      averagedLast.powerW = (float)(acc1s.sumPowerW / acc1s.sampleCount);
      averagedLast.voltageV = (float)(acc1s.sumVoltageV / acc1s.sampleCount);
      averagedLast.humidityPercent = (float)(acc1s.sumHumidityPct / acc1s.sampleCount);
      averagedLast.valid = true;

      have2sWindow = have1sWindow;  // becomes true after second window
      have1sWindow = true;

      // ====== UPDATE COULOMB COUNTING (pakai rata-rata window) ======
      updateCoulombCounting(averagedLast);

      // reset accumulator for next window
      acc1s = AccumulatedSums{};
    }

    // Update OLED with the averaged data
    if (averagedLast.valid) {
      updateOLEDDisplay(averagedLast);
    }
  }

  // ---- 3) BLE notify every 2s with mean of last two windows ----
  const int connectionCount = bleServer ? bleServer->getConnectedCount() : 0;
  if (connectionCount > 0) {
    lastConnectionTimeMs = nowMs;
    if (have2sWindow && (nowMs - lastNotifyMillis >= notifyIntervalMs)) {
      lastNotifyMillis = nowMs;

      SensorReadings bleAverage;
      bleAverage.valid = true;
      bleAverage.voltageV = (averagedLast.voltageV + averagedPrev.voltageV) * 0.5f;
      bleAverage.currentA = (averagedLast.currentA + averagedPrev.currentA) * 0.5f;
      bleAverage.temperatureC = (averagedLast.temperatureC + averagedPrev.temperatureC) * 0.5f;
      bleAverage.humidityPercent = (averagedLast.humidityPercent + averagedPrev.humidityPercent) * 0.5f;
      bleAverage.powerW = (averagedLast.powerW + averagedPrev.powerW) * 0.5f;

      notifyAll(bleAverage);
    }
  } else {
    // No connections: consider deep sleep after timeout (unless user is holding)
    if (!holdInProgress && (nowMs - lastConnectionTimeMs > SLEEP_TIMEOUT_MS)) {
      bool userCanceled = runSleepCountdownCancelable(5);

      if (userCanceled) {
        // refresh idle timer so we postpone sleeping
        lastConnectionTimeMs = millis();

        // brief "Canceled" toast
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print(bleDeviceName);
        display.setCursor(0, 12);
        display.print("Sleep canceled");
        display.display();
        delay(600);

      } else {
        // only sleep if still no connection and not in OTA hold
        if ((bleServer ? bleServer->getConnectedCount() : 0) == 0 && !holdInProgress) {
          enterDeepSleep();
        }
      }
    }
  }

  delay(200);  // modest loop rate (~5 Hz sampling into avg)
}

// ================== IMPLEMENTATION ==================

void scanI2C() {
  Serial.println("I2C scan start...");
  byte foundCount = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  - Found I2C 0x%02X\n", address);
      foundCount++;
    }
  }
  if (foundCount == 0) Serial.println("  (no devices found)");
  Serial.println("I2C scan done.");
}

bool initializeHardware() {
  // I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
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

  scanI2C();  // expect 0x38 (AHT), 0x3C (OLED), 0x40..0x45 (INA226)

  // AHT10/AHT20 @ 0x38
  if (!aht.begin()) {
    Serial.println("Failed to find AHT10/AHT20 at 0x38.");
    return false;
  }

  // Probe INA226 0x40..0x45
  uint8_t inaAddress = 0;
  for (uint8_t addr = 0x40; addr <= 0x45; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      inaAddress = addr;
      break;
    }
  }
  if (inaAddress == 0) {
    Serial.println("INA226 not found on 0x40..0x45.");
    return false;
  }

  // Rebind INA226 instance with detected address
  *(INA226*)&ina226 = INA226(inaAddress, &Wire);

  if (!ina226.begin()) {
    Serial.println("INA226 begin() failed.");
    return false;
  }

  // Configure INA226 (keep driver current offset = 0; we add manual offset later)
  int rc = ina226.configure(
    SHUNT_FOR_CAL,
    INA_LSB_mA_PRIMARY,
    INA_I_OFFSET_mA_CFG,
    INA_V_SCALE_E4);
  if (rc != 0) {
    Serial.println("\n***** Config Error! Chosen values outside range *****\n");
    Serial.printf("INA226.configure(0.625 mA) rc=%d, fallback 1.000 mA\n", rc);
    rc = ina226.configure(
      SHUNT_FOR_CAL,
      INA_LSB_mA_FALLBACK,
      INA_I_OFFSET_mA_CFG,
      INA_V_SCALE_E4);
    if (rc != 0) {
      Serial.printf("INA226.configure(1.000 mA) rc=%d\n", rc);
      return false;
    }
  } else {
    if (DO_CAL) {
      Serial.println("\n***** INA 226 CONFIGURE *****");
      Serial.print("Shunt:\t");
      Serial.print(SHUNT_FOR_CAL, 4);
      Serial.println(" Ohm");
      Serial.print("current_LSB_mA:\t");
      Serial.print(INA_LSB_mA_PRIMARY * 1e+3, 1);
      Serial.println(" uA / bit");
      Serial.print("\nMax Measurable Current:\t");
      Serial.print(ina226.getMaxCurrent(), 3);
      Serial.println(" A");

      /* CALIBRATION */
      float bv = 0, cu = 0;
      for (int i = 0; i < 10; i++) {
        bv += ina226.getBusVoltage();
        cu += ina226.getCurrent_mA();
        delay(150);
      }
      bv /= 10;
      cu /= 10;
      Serial.println("\nAverage Bus and Current values for use in Shunt Resistance, Bus Voltage and Current Zero Offset calibration:");
      bv = 0;
      for (int i = 0; i < 10; i++) {
        bv += ina226.getBusVoltage();
        delay(100);
      }
      bv /= 10;
      Serial.print("\nAverage of 10 Bus Voltage values = ");
      Serial.print(bv, 3);
      Serial.println("V");
      cu = 0;
      for (int i = 0; i < 10; i++) {
        cu += ina226.getCurrent_mA();
        delay(100);
      }
      cu /= 10;
      Serial.print("Average of 10 Current values = ");
      Serial.print(cu, 3);
      Serial.println("mA");

      Serial.println("\nCALIBRATION VALUES TO USE:\t(DMM = Digital MultiMeter)");
      Serial.println("Step 5. Attach a power supply with voltage 5-10V to INA226 on VBUS/IN+ and GND pins, without any load.");
      Serial.print("\tcurrent_zero_offset_mA = ");
      Serial.print(INA_I_OFFSET_mA_CFG + cu, 3);
      Serial.println("mA");
      if (cu > 5)
        Serial.println("********** NOTE: No resistive load needs to be present during current_zero_offset_mA calibration. **********");
      Serial.print("\tbus_V_scaling_e4 = ");
      Serial.print(INA_V_SCALE_E4);
      Serial.print(" / ");
      Serial.print(bv, 3);
      Serial.println(" * (DMM Measured Bus Voltage)");
      Serial.println("Step 8. Set DMM in current measurement mode. Use a resistor that will generate around 50-100mA IOUT measurement between IN- and GND pins with DMM in series with load. Note current measured on DMM.");
      Serial.print("\tshunt = ");
      Serial.print(SHUNT_FOR_CAL);
      Serial.print(" * ");
      Serial.print(cu, 3);
      Serial.println(" / (DMM Measured IOUT)");
      if (cu < 40)
        Serial.println("********** NOTE: IOUT needs to be more than 50mA for better shunt resistance calibration. **********");
      delay(1000);

      /* MEASUREMENTS */
      Serial.println("\nBUS(V) SHUNT(mV) CURRENT(mA) POWER(mW)");
      for (int i = 0; i < 5; i++) {
        bv = ina226.getBusVoltage();
        float sv = ina226.getShuntVoltage_mV();
        cu = ina226.getCurrent_mA();
        float po = (bv - sv / 1000) * cu;
        Serial.print(bv, 3);
        Serial.print("\t");
        Serial.print(sv, 3);
        Serial.print("\t\t");
        Serial.print(cu, 1);
        Serial.print("\t");
        Serial.print(po, 1);
        Serial.println();
        delay(1000);
      }
    }
  }
  ina226.setAverage(INA226_128_SAMPLES);
  ina226.setBusVoltageConversionTime(INA226_2100_us);
  ina226.setShuntVoltageConversionTime(INA226_2100_us);
  ina226.setMode(7);  // continuous shunt+bus

  // BLE name (also used in header)
  bleDeviceName = String("BATT-Mon_") + String(DEVICE_NUM);

  Serial.printf("INA226 @0x%02X ready\n", inaAddress);
  return true;
}

void initializeBLE() {
  NimBLEDevice::init(bleDeviceName.c_str());
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // optional
  NimBLEDevice::setMTU(256);               // optional

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  NimBLEService* bleService = bleServer->createService(SERVICE_UUID);

  bleDataCharacteristic = bleService->createCharacteristic(
    CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Portable CCCD (0x2902)
  NimBLEDescriptor* cccd = new NimBLEDescriptor(
    (uint16_t)0x2902,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE,
    2  // 2-byte CCCD
  );
  // Default notifications ON (0x0001, little-endian)
  uint8_t cccdInitialValue[2] = { 0x01, 0x00 };
  cccd->setValue(cccdInitialValue, sizeof(cccdInitialValue));
  bleDataCharacteristic->addDescriptor(cccd);

  bleService->start();

  // --- Advertising & Scan Response (NimBLE 2.x style) ---
  NimBLEAdvertisementData advData;
  advData.setName(bleDeviceName.c_str());
  advData.setCompleteServices(SERVICE_UUID);

  NimBLEAdvertisementData scanRespData;
  scanRespData.setName(bleDeviceName.c_str());

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->setAdvertisementData(advData);
  advertising->setScanResponseData(scanRespData);

  advertising->start();  // continue advertising even when connected
  Serial.printf("Advertising as %s\n", bleDeviceName.c_str());
}

// ---- Take one corrected sample (offsets applied) ----
SensorReadings getInstantReading() {
  SensorReadings readings;
  readings.valid = false;

  sensors_event_t humidityEvent, temperatureEvent;
  if (aht.getEvent(&humidityEvent, &temperatureEvent)) {
    float busVoltageV = ina226.getBusVoltage();      // V
    float shuntCurrent_mA = ina226.getCurrent_mA();  // mA
    float busPower_mW = ina226.getPower_mW();        // mW

    // Apply user offsets (post-read)
    busVoltageV += (V_OFFSET_mV / 1000.0f);
    shuntCurrent_mA += I_OFFSET_mA;

    readings.temperatureC = temperatureEvent.temperature;
    readings.humidityPercent = humidityEvent.relative_humidity;
    readings.voltageV = busVoltageV;
    readings.currentA = shuntCurrent_mA / 1000.0f;  // A
    readings.powerW = busPower_mW / 1000.0f;        // W
    readings.valid = true;
  }
  return readings;
}

void notifyAll(const SensorReadings& readings) {
  if (!bleDataCharacteristic || !readings.valid) return;

  // BLE payload kept same for app: v%.2f a%.2f t%.2f h%.1f
  char payload[128];
  snprintf(payload, sizeof(payload), "v%.2fa%.2ft%.2fh%.1f",
           readings.voltageV,
           readings.currentA,
           readings.temperatureC,
           readings.humidityPercent);

  bleDataCharacteristic->setValue((uint8_t*)payload, strlen(payload));
  bleDataCharacteristic->notify();
}

// ===================== Coulomb Counting =====================
// ASSUMPTION: currentA > 0 = CHARGING (adds energy to battery)
//             currentA < 0 = DISCHARGING (removes energy from battery)

void updateCoulombCounting(const SensorReadings& readings) {
  if (!readings.valid) return;

  unsigned long nowMs = millis();
  if (lastCoulombMs == 0) {
    lastCoulombMs = nowMs;
    return;
  }

  float dtSec = (nowMs - lastCoulombMs) / 1000.0f;
  lastCoulombMs = nowMs;

  if (dtSec <= 0.0f) return;

  float packCurrentA = readings.currentA;

  // Positive current = charging = adds to accumulated Ah
  // Negative current = discharging = subtracts from accumulated Ah
  float deltaAh = (packCurrentA * dtSec) / 3600.0f;  // Ah in time dt
  coulombAccumulatedAh += deltaAh;
}


// SoC relatif dari Coulomb Counting, anchored ke SoC OCV saat pertama dipakai
float soc_from_coulomb(float socOcv) {
  float capAh = PACK_CAPACITY_Ah();
  if (capAh <= 0.01f) return socOcv;  // fallback ke OCV kalau kapasitas aneh

  if (!coulombInitialized) {
    // Pertama kali dipanggil: anchor coulomb ke SoC OCV saat itu
    coulombBaseSocPct = socOcv;
    coulombAccumulatedAh = 0.0;
    coulombInitialized = true;
  }

  // Delta SOC from accumulated charge/discharge
  float deltaSocPct = (float)(coulombAccumulatedAh / capAh) * 100.0f;
  float soc = coulombBaseSocPct + deltaSocPct;

  if (soc < 0.0f) soc = 0.0f;
  if (soc > 100.0f) soc = 100.0f;
  return soc;
}


// ===================== SoC (OCV-based, sama konsep dengan app) =====================

// Correct measured voltage to OCV (open-circuit voltage) with pack resistance.
// ASSUMPTION: currentA > 0 = CHARGING, currentA < 0 = DISCHARGING
static float pack_ocv_from_vi(float measuredPackVoltageV, float packCurrentA) {
  // Under CHARGING (I > 0):
  //   - Measured voltage is HIGH (V_meas = V_ocv + I*R)
  //   - To get OCV: V_ocv = V_meas - I*R
  //
  // Under DISCHARGING (I < 0):
  //   - Measured voltage is LOW (V_meas = V_ocv + I*R, but I is negative)
  //   - To get OCV: V_ocv = V_meas - I*R (subtracting negative = adding)
  //
  // Formula is same for both: V_ocv = V_meas - I*R
  
  return measuredPackVoltageV - (packCurrentA * R_PACK_OHM());
}

// Mapping tegangan per-cell -> SoC% dengan 3 titik:
//   PCELL_VMIN -> 0%
//   PCELL_VNOM -> 50%
//   PCELL_VMAX -> 100%
static float soc_from_cell_ocv(float cellVoltageV) {
  const float vmin = PCELL_VMIN;
  const float vnom = PCELL_VNOM;
  const float vmax = PCELL_VMAX;

  float socPct = 0.0f;

  if (cellVoltageV <= vmin) {
    socPct = 0.0f;
  } else if (cellVoltageV >= vmax) {
    socPct = 100.0f;
  } else if (cellVoltageV <= vnom) {
    // 0–50% (vmin → vnom)
    socPct = 50.0f * (cellVoltageV - vmin) / (vnom - vmin);
  } else {
    // 50–100% (vnom → vmax)
    socPct = 50.0f + 50.0f * (cellVoltageV - vnom) / (vmax - vnom);
  }

  if (socPct < 0.0f) socPct = 0.0f;
  if (socPct > 100.0f) socPct = 100.0f;
  return socPct;
}

// Estimasi SoC pack (OCV + Coulomb + EMA; tanpa absolute Ah kalibrasi penuh)
float estimateSOCpct(float measuredPackVoltageV) {
  // 1) Arus pack rata2 terakhir (untuk koreksi IR)
  float packCurrentA = averagedLast.valid ? averagedLast.currentA : 0.0f;

  // 2) Correct measured voltage to OCV
  //    This accounts for 4S configuration (series voltage drop)
  float packVoltageOCV = pack_ocv_from_vi(measuredPackVoltageV, packCurrentA);  
  // 3) Convert pack OCV to per-cell voltage
  //    (parallel config doesn't affect voltage, only capacity)
  float cellVoltageV = packVoltageOCV / SERIES_CELLS;
  // 4) Get SOC from per-cell OCV
  float socOcv = soc_from_cell_ocv(cellVoltageV);

  // 4) SoC dari Coulomb Counting (anchored ke SoC OCV)
  float socCc = soc_from_coulomb(socOcv);

  // 6) Blend: OCV is more reliable (85%), CC for dynamics (15%)
  const float weightOcv = 0.85f;
  const float weightCc = 0.15f;
  float socMix = weightOcv * socOcv + weightCc * socCc;

  // 7) Exponential moving average for smoothing
  static bool emaInitialized = false;
  static float socEma = 0.0f;
  if (!emaInitialized) {
    socEma = socMix;
    emaInitialized = true;
  } else {
    socEma = SOC_EMA_ALPHA * socMix + (1.0f - SOC_EMA_ALPHA) * socEma;
  }

  if (socEma < 0.0f) socEma = 0.0f;
  if (socEma > 100.0f) socEma = 100.0f;
  return socEma;
}

// === Single long battery bar with % text ===
void drawBatteryBars(float percent) {
  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;

  const int barX = 0;
  const int barY = 9;
  const int barWidth = 100;
  const int barHeight = 8;

  display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);

  int filledWidth = (int)((barWidth - 2) * (percent / 100.0f) + 0.5f);
  if (filledWidth > 0) {
    display.fillRect(barX + 1, barY + 1, filledWidth, barHeight - 2, SSD1306_WHITE);
  }

  // use floor instead of rounding up
  int percentInt = (int)(percent);  // 99.4 -> 99, not 100
  char percentText[8];
  snprintf(percentText, sizeof(percentText), "%d%%", percentInt);

  int textX = barX + barWidth + 4;
  int textY = barY;

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(textX, textY);
  display.print(percentText);
}

void drawCenteredBottom(const String& text) {
  int textWidth = text.length() * 6;
  int x = (SCREEN_WIDTH - textWidth) / 2;
  int y = 55;
  if (x < 0) x = 0;
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.print(text);
}

void updateOLEDDisplay(const SensorReadings& readings) {
  display.clearDisplay();
  display.setTextWrap(false);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Header name (left)
  display.setCursor(0, 0);
  display.print(bleDeviceName);

  // Header overlay (right)
  if (overlayCountdown > 0) {
    headerOverlayText = "ROM-(" + String(overlayCountdown) + ")";
    int overlayX = SCREEN_WIDTH - (headerOverlayText.length() * 6);
    if (overlayX < 0) overlayX = 0;
    display.setCursor(overlayX, 0);
    display.print(headerOverlayText);
  }

  // Battery bar with OCV + Coulomb SoC
  float socPercent = readings.valid ? estimateSOCpct(readings.voltageV) : 0.0f;
  drawBatteryBars(socPercent);

  // Separators
  display.drawFastHLine(0, 18, 128, SSD1306_WHITE);
  display.drawFastVLine(64, 18, 33, SSD1306_WHITE);
  display.drawFastHLine(0, 53, 128, SSD1306_WHITE);

  // Left: V / A (averaged values)
  display.setCursor(0, 25);
  if (readings.valid) display.printf("V:%.2f V", readings.voltageV);
  else display.print("V:--.--");

  display.setCursor(0, 37);
  if (readings.valid) display.printf("A:%.2f A", readings.currentA);
  else display.print("A:--.--");

  // Right: T / P
  display.setCursor(67, 25);
  if (readings.valid) {
    display.printf("T:%.2f ", readings.temperatureC);
    display.print((char)247);  // °
    display.print("C");
  } else {
    display.print("T:--.-");
  }

  display.setCursor(68, 37);
  if (readings.valid) {
    // float powerWatt = readings.powerW;
    float powerWatt = readings.voltageV * readings.currentA;
    display.printf("P:%.2f W", powerWatt);
  } else {
    display.print("P:--.--");
  }

  // Bottom centered status (animated)
  const int connectionCount = bleServer ? bleServer->getConnectedCount() : 0;
  String statusText;
  if (connectionCount > 0) {
    String leftDots, rightDots;
    for (uint8_t phaseIndex = 0; phaseIndex < dotPhase; ++phaseIndex) {
      leftDots += ". ";
      rightDots += " .";
    }
    statusText = leftDots + String("Connected (") + String(connectionCount) + String(")") + rightDots;
  } else {
    unsigned long nowMs = millis();
    if (nowMs - lastDotTickMs >= DOT_PERIOD_MS) {
      lastDotTickMs = nowMs;
      dotPhase = (dotPhase + 1) % 4;
    }
    String leftDots, rightDots;
    for (uint8_t phaseIndex = 0; phaseIndex < dotPhase; ++phaseIndex) {
      leftDots += ". ";
      rightDots += " .";
    }
    statusText = leftDots + "Adv" + rightDots;
  }
  int statusWidth = statusText.length() * 6;
  int statusX = (SCREEN_WIDTH - statusWidth) / 2;
  if (statusX < 0) statusX = 0;
  display.setCursor(statusX, 56);
  display.print(statusText);

  display.display();
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeupReason = esp_sleep_get_wakeup_cause();
  switch (wakeupReason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup: EXT0"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup: EXT1"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup: TIMER"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup: TOUCH"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup: ULP"); break;
    case ESP_SLEEP_WAKEUP_GPIO:
      {
        uint64_t gpioMask = esp_sleep_get_gpio_wakeup_status();
        Serial.printf("Wakeup: GPIO (mask=0x%llX)\n", (unsigned long long)gpioMask);
        break;
      }
    default:
      Serial.printf("Wakeup not from deep sleep: %d\n", wakeupReason);
      break;
  }
}

// Cancelable "sleeping in N..." countdown that polls the button frequently.
// Returns true if user canceled, false if countdown completed.
bool runSleepCountdownCancelable(int seconds) {
  for (int remaining = seconds; remaining > 0; --remaining) {
    // draw one frame
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print(bleDeviceName);
    display.setCursor(0, 12);
    display.print("Sleeping in ");
    display.print(remaining);
    display.print("s");
    display.display();

    // poll for up to 1000 ms with debounce and OTA suppression
    unsigned long deadlineMs = millis() + 1000;
    bool debouncedLow = false;
    unsigned long lowSinceMs = 0;

    while ((long)(deadlineMs - millis()) > 0) {
      // If user presses the wake button (active-LOW), cancel
      int pinState = digitalRead(WAKE_UP_BUTTON_PIN);
      if (pinState == LOW) {
        if (!debouncedLow) {
          debouncedLow = true;
          lowSinceMs = millis();
        }
        // simple 30 ms debounce
        if (millis() - lowSinceMs >= 30) {
          // Cancel the countdown
          // (do NOT trigger OTA toggle here; short press just cancels)
          return true;
        }
      } else {
        debouncedLow = false;
      }

      // If a central connected during countdown, abort too
      if (bleServer && bleServer->getConnectedCount() > 0) return true;

      delay(20);
    }
  }
  return false;  // countdown reached 0
}

void enterDeepSleep() {
  Serial.println("Entering deep sleep...");

  // Turn off OLED
  display.clearDisplay();
  display.display();
  display.ssd1306_command(SSD1306_DISPLAYOFF);

  // Timer wake
  esp_sleep_enable_timer_wakeup((uint64_t)WAKE_INTERVAL_MS * 1000ULL);

#if USE_EXT0_WAKEUP
  // EXT0 (if you really need it)
  esp_deep_sleep_enable_ext0_wakeup((gpio_num_t)WAKE_UP_BUTTON_PIN, 1);
  gpio_pullup_dis((gpio_num_t)WAKE_UP_BUTTON_PIN);
  gpio_pulldown_en((gpio_num_t)WAKE_UP_BUTTON_PIN);
#else
  // *** C3 GPIO wake: wake when button pulls pin LOW ***
  uint64_t wakeMask = (1ULL << WAKE_UP_BUTTON_PIN);
  esp_deep_sleep_enable_gpio_wakeup(wakeMask, ESP_GPIO_WAKEUP_GPIO_LOW);
  // Hold pin HIGH during sleep so it doesn't instantly wake
  gpio_pullup_en((gpio_num_t)WAKE_UP_BUTTON_PIN);
  gpio_pulldown_dis((gpio_num_t)WAKE_UP_BUTTON_PIN);
#endif

  esp_deep_sleep_start();
}

// ================== OTA HELPERS ==================

void startOTA_AP_Mode() {
  if (otaMode) return;
  otaMode = true;

  otaSSID = "ROM_BATT-Mon_" + String(DEVICE_NUM);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(otaSSID.c_str(), OTA_PASSWORD);
  otaIP = WiFi.softAPIP();

  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(otaSSID.c_str());
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint8_t lastPercent = 255;
    uint8_t percent = (progress * 100) / total;
    if (percent != lastPercent) {
      lastPercent = percent;
      Serial.printf("OTA %u%%\n", percent);
    }
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });

  ArduinoOTA.begin();
  Serial.printf("OTA AP started: SSID=%s PWD=%s IP=%s\n",
                otaSSID.c_str(), OTA_PASSWORD, otaIP.toString().c_str());
}

void stopOTA_AP_Mode() {
  if (!otaMode) return;
  ArduinoOTA.end();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  otaMode = false;
  otaIP = IPAddress(0, 0, 0, 0);
  Serial.println("OTA AP stopped");
}

void checkOTAToggle() {
  // Button is active-LOW (to GND). Show overlay "ROM-(4..1)" for last 4s of hold.
  int pinState = digitalRead(WAKE_UP_BUTTON_PIN);
  unsigned long nowMs = millis();

  if (pinState == LOW) {
    if (!holdInProgress) {
      holdInProgress = true;
      holdStartMs = nowMs;
      overlayCountdown = 4;  // start at 4
    } else {
      unsigned long heldMs = nowMs - holdStartMs;
      long remainMs = (long)OTA_HOLD_MS - (long)heldMs;
      int showSeconds = (remainMs > 0) ? ((remainMs + 999) / 1000) : 0;  // ceil
      if (showSeconds > 4) showSeconds = 4;                              // clamp to 4..1
      if (showSeconds < 0) showSeconds = 0;
      overlayCountdown = showSeconds;

      if (heldMs >= OTA_HOLD_MS) {
        // Toggle OTA mode
        if (otaMode) stopOTA_AP_Mode();
        else startOTA_AP_Mode();

        // prevent rapid retrigger; wait for release
        while (digitalRead(WAKE_UP_BUTTON_PIN) == LOW) {
          delay(10);
        }
        holdInProgress = false;
        overlayCountdown = -1;
      }
    }
  } else {
    // Released
    holdInProgress = false;
    overlayCountdown = -1;
  }
}
