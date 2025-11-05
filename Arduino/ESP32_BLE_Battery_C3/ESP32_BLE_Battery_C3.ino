#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_AHTX0.h>
#include <INA226.h>
#include <esp_sleep.h>
#include "driver/rtc_io.h"

// ===== NimBLE (multi-central) =====
#include <NimBLEDevice.h>

// ===================== CONFIG =====================
#define DEVICE_NUM              5          // 4 / 5 / 6
#define WAKE_UP_BUTTON_PIN      2          // Button to GND; we use pull-up and wake on LOW
#define USE_EXT0_WAKEUP         0          // C3: keep 0 (EXT1) unless you know your pin is valid for EXT0

#define SLEEP_TIMEOUT_MS        60000      // 1 min idle -> sleep
#define WAKE_INTERVAL_MS        300000     // timer wake every 5 minutes
#define DOT_PERIOD_MS           500

// BMS estimate for OLED bars (app does precise logic)
#define SERIES_CELLS            3
#define PCELL_VMIN              2.80f
#define PCELL_VNOM              3.60f
#define PCELL_VMAX              4.20f

// ---- INA226 / SHUNT CONFIG ----
#define SHUNT_FOR_CAL           0.00621f   // refined shunt (Ohm)
#define INA_LSB_mA_PRIMARY      0.625f
#define INA_LSB_mA_FALLBACK     1.000f
#define INA_I_OFFSET_mA         0.0f
#define INA_V_SCALE_E4          10000

// I2C pins (ESP32-C3)
#define I2C_SDA_PIN 3
#define I2C_SCL_PIN 4

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
unsigned long SendedDataRoutine = 2000; // Kirim Data tiap 2 detik

// Adv / status animation
uint8_t dotPhase = 0;
unsigned long lastDotTick = 0;

struct SensorReadings {
  float temperature;
  float current;   // A
  float voltage;   // V
  float humidity;  // %RH
  bool valid;
};

// === Prototypes ===
bool initializeHardware();
void initializeBLE();
SensorReadings getSensorReadings();
void notifyAll(const SensorReadings& rd);
void updateOLEDDisplay(const SensorReadings& rd);
void drawBatteryBars(float percent);
float estimateSOCpct(float packVolt);
void drawCenteredBottom(const String& s);
void print_wakeup_reason();
void enterDeepSleep();
void scanI2C();

// ================== BLE CALLBACKS ==================
// NimBLE 2.3.6 callback signatures
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    Serial.printf("Central connected (handle=%d), total=%d\n",
                  info.getConnHandle(), s->getConnectedCount());
    // keep advertising so more centrals can join
    NimBLEDevice::startAdvertising();
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    Serial.printf("Central disconnected (handle=%d, reason=%d), total=%d\n",
                  info.getConnHandle(), reason, s->getConnectedCount());
    // restart advertising
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

  // IMPORTANT for wake-on-LOW: keep pin HIGH when idle
  pinMode(WAKE_UP_BUTTON_PIN, INPUT_PULLUP);

  lastConnectionTime = millis();

  Serial.println("Setup complete. Advertising...");
}

void loop() {
  const int connCount = pServer ? pServer->getConnectedCount() : 0;

  // Read sensors periodically for OLED + notifications
  SensorReadings readings = getSensorReadings();

  if (connCount > 0) {
    lastConnectionTime = millis();

    // Throttle BLE notify: kirim tiap SendedDataRoutine ms (default 2000 ms)
    if (millis() - lastNotifyMillis >= SendedDataRoutine) {
      lastNotifyMillis = millis();
      if (readings.valid) notifyAll(readings);
    }
  } else {
    // No connections: consider deep sleep after timeout
    if (millis() - lastConnectionTime > SLEEP_TIMEOUT_MS) {
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

  // Update OLED with current state (smooth UI refresh)
  updateOLEDDisplay(readings);

  delay(200);  // modest loop rate
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

  int rc = ina226.configure(
    SHUNT_FOR_CAL,
    INA_LSB_mA_PRIMARY,
    INA_I_OFFSET_mA,
    INA_V_SCALE_E4
  );
  if (rc != 0) {
    Serial.printf("INA226.configure(0.625 mA) rc=%d, fallback 1.000 mA\n", rc);
    rc = ina226.configure(
      SHUNT_FOR_CAL,
      INA_LSB_mA_FALLBACK,
      INA_I_OFFSET_mA,
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

  Serial.printf("INA226 @0x%02X ready\n", inaAddr);
  return true;
}

void initializeBLE() {
  bleDeviceName = String("BATT-Mon_") + String(DEVICE_NUM);

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

  // Optional: tweak intervals
  // adv->setMinInterval(0x20); // ~20 ms
  // adv->setMaxInterval(0x40); // ~40 ms

  adv->start();  // continue advertising even when connected
  Serial.printf("Advertising as %s\n", bleDeviceName.c_str());
}

SensorReadings getSensorReadings() {
  SensorReadings r;
  r.valid = false;

  sensors_event_t humidity, temp;
  if (aht.getEvent(&humidity, &temp)) {
    r.temperature = temp.temperature;
    r.humidity    = humidity.relative_humidity;
    r.voltage     = ina226.getBusVoltage();          // V
    r.current     = ina226.getCurrent_mA() / 1000.0; // A
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
  // Simple piecewise Min -> Nom -> Max (OLED only)
  float vcell = packVolt / SERIES_CELLS;
  if (vcell <= PCELL_VMIN) return 0.0f;
  if (vcell >= PCELL_VMAX) return 100.0f;
  if (vcell <= PCELL_VNOM) {
    return 50.0f * (vcell - PCELL_VMIN) / (PCELL_VNOM - PCELL_VMIN);
  }
  return 50.0f + 50.0f * (vcell - PCELL_VNOM) / (PCELL_VMAX - PCELL_VNOM);
}

void drawBatteryBars(float percent) {
  // 4 segments at 25/50/75/100 like your mock
  const int y = 9, h = 8;
  struct Seg { int x,w; float th; };
  Seg segs[4] = {
    {  0,26, 25.0f },
    { 29,34, 50.0f },
    { 67,32, 75.0f },
    {102,26,100.0f }
  };

  for (int i=0;i<3;i++) {
    if (percent >= segs[i].th) display.fillRect(segs[i].x, y, segs[i].w, h, SSD1306_WHITE);
    else display.drawRect(segs[i].x, y, segs[i].w, h, SSD1306_WHITE);
  }
  // Last segment: outline + fill at >=100%
  display.drawRect(segs[3].x, y, segs[3].w, h, SSD1306_WHITE);
  if (percent >= 100.0f) {
    display.fillRect(segs[3].x, y, segs[3].w, h, SSD1306_WHITE);
  }
}

void drawCenteredBottom(const String& s) {
  // Approx width ~6 px per char with default font
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
  display.setTextWrap(false);                 // prevent wrapping into bottom line
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // Header name
  display.setCursor(0, 0);
  display.print(bleDeviceName);

  // Battery bars (top)
  float pct = rd.valid ? estimateSOCpct(rd.voltage) : 0.0f;
  drawBatteryBars(pct);                        // uses y=9, h=8

  // Separators
  display.drawFastHLine(0, 18, 128, SSD1306_WHITE);  // top of metric area
  display.drawFastVLine(64, 18, 33, SSD1306_WHITE);  // vertical divider
  display.drawFastHLine(0, 53, 128, SSD1306_WHITE);  // bottom line

  // Left: V / A
  display.setCursor(0, 25);
  if (rd.valid) display.printf("V= %.2f V", rd.voltage); else display.print("V= --.--");
  display.setCursor(0, 37);
  if (rd.valid) display.printf("A= %.2f A", rd.current); else display.print("A= --.--");

  // Right: T / H
  display.setCursor(67, 25);
  if (rd.valid) display.printf("T= %.2f C", rd.temperature); else display.print("T= --.--");
  display.setCursor(68, 37);
  if (rd.valid) display.printf("H= %.1f%%RH", rd.humidity); else display.print("H= --.--");

  // Bottom centered status
  const int connCount = pServer ? pServer->getConnectedCount() : 0;
  String status;
  if (connCount > 0) {
    status = String("Connected (") + String(connCount) + String(")");
  } else {
    unsigned long now = millis();
    if (now - lastDotTick >= DOT_PERIOD_MS) {
      lastDotTick = now;
      dotPhase = (dotPhase + 1) % 4;
    }
    status = "Adv";
    for (uint8_t i = 0; i < dotPhase; ++i) status += " .";
  }
  // Center on y=56 (clear of the 53px line)
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
  gpio_num_t WAKEUP_GPIO = (gpio_num_t)WAKE_UP_BUTTON_PIN;
  // If needed for EXT0 on your setup:
  esp_deep_sleep_enable_ext0_wakeup((gpio_num_t)WAKE_UP_BUTTON_PIN, 1);
  gpio_pullup_dis((gpio_num_t)WAKE_UP_BUTTON_PIN);
  gpio_pulldown_en((gpio_num_t)WAKE_UP_BUTTON_PIN);
#else
  // *** C3 GPIO wake: wake when button pulls pin LOW ***
  uint64_t mask = (1ULL << WAKE_UP_BUTTON_PIN);
  esp_deep_sleep_enable_gpio_wakeup(mask, ESP_GPIO_WAKEUP_GPIO_LOW);
  // Hold pin HIGH during sleep so it doesn't instantly wake
  gpio_pullup_en((gpio_num_t)WAKE_UP_BUTTON_PIN);
  gpio_pulldown_dis((gpio_num_t)WAKE_UP_BUTTON_PIN);
#endif

  esp_deep_sleep_start();
}
