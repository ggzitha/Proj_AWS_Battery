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
#define DEVICE_NUM              6          // 4 / 5 / 6 -> used in names
#define WAKE_UP_BUTTON_PIN      2          // Button to GND; we use pull-up and wake on LOW
#define USE_EXT0_WAKEUP         0          // Keep 0 (EXT1/GPIO wake) on C3

#define SLEEP_TIMEOUT_MS        60000      // 1 min idle -> sleep
#define WAKE_INTERVAL_MS        300000     // timer wake every 5 minutes
#define DOT_PERIOD_MS           500

// BMS estimate for OLED bars (app does precise logic)
#define SERIES_CELLS            4
#define PCELL_VMIN              2.75f
#define PCELL_VNOM              3.60f
#define PCELL_VMAX              4.20f

// ---- INA226 / SHUNT CONFIG ----
#define SHUNT_FOR_CAL           0.00621f   // refined shunt (Ohm)
#define INA_LSB_mA_PRIMARY      0.625f
#define INA_LSB_mA_FALLBACK     1.000f
#define INA_I_OFFSET_mA_CFG     0.0f       // keep 0; we apply our own offset below
#define INA_V_SCALE_E4          10000

// ===== User calibration offsets (apply AFTER sensor read) =====
#define V_OFFSET_mV             0.0f       // add/subtract voltage in millivolts (e.g. +25.0f)
#define I_OFFSET_mA             0.0f       // add/subtract current in milliamps (e.g. -5.0f)

// I2C pins (ESP32-C3)
#define I2C_SDA_PIN 3
#define I2C_SCL_PIN 4

// OTA AP credentials
#define OTA_PASSWORD            "123456789"
#define OTA_PORT                3232
#define OTA_HOLD_MS             5000       // hold time to toggle OTA

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Sensors
Adafruit_AHTX0 aht;
INA226 ina226(0x40, &Wire);

// === BLE UUIDs (match Flutter) ===
static NimBLEUUID SERVICE_UUID("91bad492-b950-4226-aa2b-4ede9fa42f59");
static NimBLEUUID CHAR_UUID   ("cba1d466-344c-4be3-ab3f-189daa0a16d8");

// BLE globals
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pDataChar = nullptr;
String bleDeviceName;
unsigned long lastConnectionTime = 0;
unsigned long lastNotifyMillis = 0;
unsigned long SendedDataRoutine = 2000; // 2 seconds

// Adv / status animation
uint8_t dotPhase = 0;
unsigned long lastDotTick = 0;

// OTA state
bool otaMode = false;
String otaSSID;
IPAddress otaIP(0,0,0,0);

// Button/hold detection (shared for enter/exit OTA)
bool holdInProgress = false;
unsigned long holdStartMs = 0;
int overlayCountdown = -1;          // -1 = no overlay
String headerOverlay = "";          // "ROM-(4)" ... "ROM-(1)"

// ======== Define SensorReadings BEFORE prototypes ========
struct SensorReadings {
  float temperature;  // C
  float current;      // A
  float voltage;      // V
  float humidity;     // %RH
  bool  valid;
};

// ===== Averaging state =====
// accumulate every loop; finalize each 1s; BLE sends avg of last two 1s windows every 2s
struct Accum {
  double v=0, i=0, t=0, h=0;
  int n=0;
} acc1s;

SensorReadings avg1s_last = {0,0,0,0,false};
SensorReadings avg1s_prev = {0,0,0,0,false};
bool have1s = false;
bool have2s = false;
unsigned long lastOLEDms = 0;

// ======== Prototypes ========
bool initializeHardware();
void initializeBLE();
SensorReadings getInstantReading();                 // returns ONE corrected sample
void notifyAll(const SensorReadings& rd);
void updateOLEDDisplay(const SensorReadings& rd);
void drawBatteryBars(float percent);
float estimateSOCpct(float packVolt);
void drawCenteredBottom(const String& s);
void print_wakeup_reason();
void enterDeepSleep();
void scanI2C();

void checkOTAToggle();
void startOTA_AP_Mode();
void stopOTA_AP_Mode();
String twoDigit(int n);

// ================== BLE CALLBACKS ==================
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    Serial.printf("Central connected (handle=%d), total=%d\n",
                  info.getConnHandle(), s->getConnectedCount());
    NimBLEDevice::startAdvertising();
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    Serial.printf("Central disconnected (handle=%d, reason=%d), total=%d\n",
                  info.getConnHandle(), s->getConnectedCount());
    NimBLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(300);

  print_wakeup_reason();

  if (!initializeHardware()) {
    Serial.println("Failed to initialize hardware. Halting.");
    while (true) delay(1000);
  }

  initializeBLE();

  // Wake pin idle HIGH for wake-on-LOW
  pinMode(WAKE_UP_BUTTON_PIN, INPUT_PULLUP);

  lastConnectionTime = millis();
  checkOTAToggle();

  Serial.println("Setup complete. Advertising...");
}

void loop() {
  // Always watch for 5s hold to enter/exit OTA and drive header overlay
  checkOTAToggle();

  if (otaMode) {
    ArduinoOTA.handle();

    // OTA page on OLED
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("OTA MODE");

    if (overlayCountdown > 0) {
      String ov = "Exit-(" + String(overlayCountdown) + ")";
      int ox = SCREEN_WIDTH - (ov.length() * 6);
      if (ox < 0) ox = 0;
      display.setCursor(ox, 0);
      display.print(ov);
    }

    display.setCursor(0, 12); display.print("SSID: "); display.print(otaSSID);
    display.setCursor(0, 24); display.print("PWD : "); display.print(OTA_PASSWORD);
    display.setCursor(0, 36); display.print("IP  : "); display.print(otaIP == IPAddress(0,0,0,0) ? "192.168.4.1" : otaIP.toString());
    display.setCursor(0, 48); display.print("Update via Arduino-IDE");
    display.setCursor(0, 56); display.print("Net-PORT or espota.py");
    display.display();

    delay(20);
    return;  // stay awake in OTA
  }

  // ---- 1) Take an instantaneous sample (corrected with offsets) ----
  SensorReadings samp = getInstantReading();
  if (samp.valid) {
    acc1s.v += samp.voltage;
    acc1s.i += samp.current;
    acc1s.t += samp.temperature;
    acc1s.h += samp.humidity;
    acc1s.n += 1;
  }

  // ---- 2) On each 1-second boundary, finalize OLED average ----
  unsigned long now = millis();
  if (now - lastOLEDms >= 1000) {
    lastOLEDms = now;

    if (acc1s.n > 0) {
      avg1s_prev = avg1s_last;
      avg1s_last.temperature = (float)(acc1s.t / acc1s.n);
      avg1s_last.current     = (float)(acc1s.i / acc1s.n);
      avg1s_last.voltage     = (float)(acc1s.v / acc1s.n);
      avg1s_last.humidity    = (float)(acc1s.h / acc1s.n);
      avg1s_last.valid       = true;
      have2s = have1s;    // becomes true after second 1s window
      have1s = true;

      // reset accumulator for next second
      acc1s = Accum{};
    }

    // Update OLED once per second with the 1s average
    if (avg1s_last.valid) updateOLEDDisplay(avg1s_last);
  }

  // ---- 3) BLE notify every 2s with mean of last two 1s windows ----
  const int connCount = pServer ? pServer->getConnectedCount() : 0;
  if (connCount > 0) {
    lastConnectionTime = now;
    if (have2s && (now - lastNotifyMillis >= SendedDataRoutine)) {
      lastNotifyMillis = now;

      SensorReadings bleAvg;
      bleAvg.valid       = true;
      bleAvg.voltage     = (avg1s_last.voltage + avg1s_prev.voltage) * 0.5f;
      bleAvg.current     = (avg1s_last.current + avg1s_prev.current) * 0.5f;
      bleAvg.temperature = (avg1s_last.temperature + avg1s_prev.temperature) * 0.5f;
      bleAvg.humidity    = (avg1s_last.humidity + avg1s_prev.humidity) * 0.5f;

      notifyAll(bleAvg);
    }
  } else {
    // No connections: consider deep sleep after timeout (unless user is holding)
    if (!holdInProgress && (now - lastConnectionTime > SLEEP_TIMEOUT_MS)) {
      for (int i = 5; i > 0; --i) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        display.setCursor(0, 0);
        display.print(bleDeviceName);
        display.setCursor(0, 12);
        display.print("Sleeping in ");
        display.print(i);
        display.print("s");
        display.display();
        delay(1000);
        if (pServer->getConnectedCount() > 0) break;
      }
      if (pServer->getConnectedCount() == 0) {
        enterDeepSleep();
      }
    }
  }

  delay(200);  // modest loop rate (~5 Hz sampling into 1s avg)
}

// ================== IMPLEMENTATION ==================

void scanI2C() {
  Serial.println("I2C scan start...");
  byte count = 0;
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("  - Found I2C 0x%02X\n", address);
      count++;
    }
  }
  if (count == 0) Serial.println("  (no devices found)");
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
  uint8_t inaAddr = 0;
  for (uint8_t addr = 0x40; addr <= 0x45; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { inaAddr = addr; break; }
  }
  if (inaAddr == 0) {
    Serial.println("INA226 not found on 0x40..0x45.");
    return false;
  }

  // Rebind INA226 instance with detected address
  *(INA226*)&ina226 = INA226(inaAddr, &Wire);

  if (!ina226.begin()) {
    Serial.println("INA226 begin() failed.");
    return false;
  }

  // Configure INA226 (keep library offset 0; we apply our own in code)
  int rc = ina226.configure(
    SHUNT_FOR_CAL,
    INA_LSB_mA_PRIMARY,
    INA_I_OFFSET_mA_CFG,
    INA_V_SCALE_E4
  );
  if (rc != 0) {
    Serial.printf("INA226.configure(0.625 mA) rc=%d, fallback 1.000 mA\n", rc);
    rc = ina226.configure(
      SHUNT_FOR_CAL,
      INA_LSB_mA_FALLBACK,
      INA_I_OFFSET_mA_CFG,
      INA_V_SCALE_E4
    );
    if (rc != 0) {
      Serial.printf("INA226.configure(1.000 mA) rc=%d\n", rc);
      return false;
    }
  }

  ina226.setAverage(64);
  ina226.setBusVoltageConversionTime(1100);
  ina226.setShuntVoltageConversionTime(1100);
  ina226.setMode(7); // continuous shunt+bus

  // BLE name (also used in header)
  bleDeviceName = String("BATT-Mon_") + String(DEVICE_NUM);

  Serial.printf("INA226 @0x%02X ready\n", inaAddr);
  return true;
}

void initializeBLE() {
  NimBLEDevice::init(bleDeviceName.c_str());
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);   // optional
  NimBLEDevice::setMTU(185);                // optional

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* svc = pServer->createService(SERVICE_UUID);

  pDataChar = svc->createCharacteristic(
      CHAR_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  // Portable CCCD (0x2902)
  NimBLEDescriptor* cccd = new NimBLEDescriptor(
      (uint16_t)0x2902,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE,
      2 // 2-byte CCCD
  );
  // Default notifications ON (0x0001, little-endian)
  uint8_t cccdVal[2] = {0x01, 0x00};
  cccd->setValue(cccdVal, sizeof(cccdVal));
  pDataChar->addDescriptor(cccd);

  svc->start();

  // --- Advertising & Scan Response (NimBLE 2.x style) ---
  NimBLEAdvertisementData advData;
  advData.setName(bleDeviceName.c_str());
  advData.setCompleteServices(SERVICE_UUID);

  NimBLEAdvertisementData scanResp;
  scanResp.setName(bleDeviceName.c_str());

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanResp);

  adv->start();  // continue advertising even when connected
  Serial.printf("Advertising as %s\n", bleDeviceName.c_str());
}

// ---- Take one corrected sample (offsets applied) ----
SensorReadings getInstantReading() {
  SensorReadings r;
  r.valid = false;

  sensors_event_t humidity, temp;
  if (aht.getEvent(&humidity, &temp)) {
    float v  = ina226.getBusVoltage();          // V
    float im = ina226.getCurrent_mA();          // mA

    // Apply user offsets (post-read)
    v  += (V_OFFSET_mV / 1000.0f);
    im += I_OFFSET_mA;

    r.temperature = temp.temperature;
    r.humidity    = humidity.relative_humidity;
    r.voltage     = v;
    r.current     = im / 1000.0f;               // A
    r.valid       = true;
  }
  return r;
}

void notifyAll(const SensorReadings& rd) {
  if (!pDataChar || !rd.valid) return;

  // EXACT format for the phone parser: v%.2fa%.2ft%.2fh%.1f
  char buf[128];
  snprintf(buf, sizeof(buf),
           "v%.2fa%.2ft%.2fh%.1f",
           rd.voltage, rd.current, rd.temperature, rd.humidity);

  pDataChar->setValue((uint8_t*)buf, strlen(buf));
  pDataChar->notify(); // to all subscribed centrals
}

float estimateSOCpct(float packVolt) {
  float vcell = packVolt / SERIES_CELLS;
  if (vcell <= PCELL_VMIN) return 0.0f;
  if (vcell >= PCELL_VMAX) return 100.0f;
  if (vcell <= PCELL_VNOM) {
    return 50.0f * (vcell - PCELL_VMIN) / (PCELL_VNOM - PCELL_VMIN);
  }
  return 50.0f + 50.0f * (vcell - PCELL_VNOM) / (PCELL_VMAX - PCELL_VNOM);
}

// === Single long battery bar with % text ===
void drawBatteryBars(float percent) {
  if (percent < 0.0f)  percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;

  const int barX = 0;
  const int barY = 9;
  const int barW = 100;
  const int barH = 8;

  display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);

  int fillW = (int)((barW - 2) * (percent / 100.0f) + 0.5f);
  if (fillW > 0) display.fillRect(barX + 1, barY + 1, fillW, barH - 2, SSD1306_WHITE);

  int ipct = (int)(percent + 0.5f);
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", ipct);

  int textX = barX + barW + 4;
  int textY = barY;

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(textX, textY);
  display.print(buf);
}

void drawCenteredBottom(const String& s) {
  int w = s.length() * 6;
  int x = (SCREEN_WIDTH - w) / 2;
  int y = 55;
  if (x < 0) x = 0;
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.print(s);
}

void updateOLEDDisplay(const SensorReadings& rd) {
  display.clearDisplay();
  display.setTextWrap(false);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Header name (left)
  display.setCursor(0, 0);
  display.print(bleDeviceName);

  // Header overlay (right)
  if (overlayCountdown > 0) {
    headerOverlay = "ROM-(" + String(overlayCountdown) + ")";
    int ox = SCREEN_WIDTH - (headerOverlay.length() * 6);
    if (ox < 0) ox = 0;
    display.setCursor(ox, 0);
    display.print(headerOverlay);
  }

  // Battery bar
  float pct = rd.valid ? estimateSOCpct(rd.voltage) : 0.0f;
  drawBatteryBars(pct);

  // Separators
  display.drawFastHLine(0, 18, 128, SSD1306_WHITE);
  display.drawFastVLine(64, 18, 33, SSD1306_WHITE);
  display.drawFastHLine(0, 53, 128, SSD1306_WHITE);

  // Left: V / A (use averaged values)
  display.setCursor(0, 25);
  if (rd.valid) display.printf("V= %.2f V", rd.voltage); else display.print("V= --.--");
  display.setCursor(0, 37);
  if (rd.valid) display.printf("A= %.2f A", rd.current); else display.print("A= --.--");

  // Right: T / H
  display.setCursor(67, 25);
  if (rd.valid) display.printf("T= %.2f C", rd.temperature); else display.print("T= --.--");
  display.setCursor(68, 37);
  if (rd.valid) display.printf("H= %.1f%%RH", rd.humidity); else display.print("H= --.--");

  // Bottom centered status (animated)
  const int connCount = pServer ? pServer->getConnectedCount() : 0;
  String status;
  if (connCount > 0) {
    String leftDots, rightDots;
    for (uint8_t i = 0; i < dotPhase; ++i) { leftDots += ". "; rightDots += " ."; }
    status = leftDots + String("Connected (") + String(connCount) + String(")") + rightDots;
  } else {
    unsigned long now = millis();
    if (now - lastDotTick >= DOT_PERIOD_MS) {
      lastDotTick = now;
      dotPhase = (dotPhase + 1) % 4;
    }
    String leftDots, rightDots;
    for (uint8_t i = 0; i < dotPhase; ++i) { leftDots += ". "; rightDots += " ."; }
    status = leftDots + "Adv" + rightDots;
  }
  int w = status.length() * 6;
  int x = (SCREEN_WIDTH - w) / 2; if (x < 0) x = 0;
  display.setCursor(x, 56);
  display.print(status);

  display.display();
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup: EXT0"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup: EXT1"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup: TIMER"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("Wakeup: TOUCH"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup: ULP"); break;
    case ESP_SLEEP_WAKEUP_GPIO: {
      uint64_t m = esp_sleep_get_gpio_wakeup_status();
      Serial.printf("Wakeup: GPIO (mask=0x%llX)\n", (unsigned long long)m);
      break;
    }
    default: Serial.printf("Wakeup not from deep sleep: %d\n", wakeup_reason); break;
  }
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
  esp_deep_sleep_enable_ext0_wakeup((gpio_num_t)WAKE_UP_BUTTON_PIN, 1);
  gpio_pullup_dis((gpio_num_t)WAKE_UP_BUTTON_PIN);
  gpio_pulldown_en((gpio_num_t)WAKE_UP_BUTTON_PIN);
#else
  uint64_t mask = (1ULL << WAKE_UP_BUTTON_PIN);
  esp_deep_sleep_enable_gpio_wakeup(mask, ESP_GPIO_WAKEUP_GPIO_LOW);
  gpio_pullup_en((gpio_num_t)WAKE_UP_BUTTON_PIN);
  gpio_pulldown_dis((gpio_num_t)WAKE_UP_BUTTON_PIN);
#endif

  esp_deep_sleep_start();
}

// ================== OTA HELPERS ==================

String twoDigit(int n) {
  if (n < 10) return "0" + String(n);
  return String(n);
}

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

  ArduinoOTA.onStart([]() { Serial.println("OTA Start"); });
  ArduinoOTA.onEnd([]()   { Serial.println("\nOTA End"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint8_t last = 255;
    uint8_t pct = (progress * 100) / total;
    if (pct != last) { last = pct; Serial.printf("OTA %u%%\n", pct); }
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
  otaIP = IPAddress(0,0,0,0);
  Serial.println("OTA AP stopped");
}

void checkOTAToggle() {
  int pin = digitalRead(WAKE_UP_BUTTON_PIN);
  unsigned long now = millis();

  if (pin == LOW) {
    if (!holdInProgress) {
      holdInProgress = true;
      holdStartMs = now;
      overlayCountdown = 4;   // start at 4
    } else {
      unsigned long held = now - holdStartMs;
      long remain = (long)OTA_HOLD_MS - (long)held;
      int show = (remain > 0) ? ((remain + 999) / 1000) : 0; // ceil
      if (show > 4) show = 4; if (show < 0) show = 0;
      overlayCountdown = show;

      if (held >= OTA_HOLD_MS) {
        if (otaMode) stopOTA_AP_Mode();
        else startOTA_AP_Mode();

        while (digitalRead(WAKE_UP_BUTTON_PIN) == LOW) { delay(10); }
        holdInProgress = false;
        overlayCountdown = -1;
      }
    }
  } else {
    holdInProgress = false;
    overlayCountdown = -1;
  }
}
