// ============================================================
//  CPRPal — ESP32 Arduino Firmware
//  A wearable CPR coaching device that monitors compression
//  rate (CPM), compression depth/force, body tilt (IMU), and
//  rescuer heart rate (PPG). Feedback is delivered via a TFT
//  display, NeoPixel LED bars, and a buzzer metronome.
//
//  FreeRTOS Task layout:
//    Core 0 — taskHX711   : reads the load cell + computes CPM
//    Core 0 — taskIMU     : reads BNO055 orientation
//    Core 0 — taskPPG     : pulse-oximetry / heart-rate detection
//    Core 0 — taskBuzzer  : 110 BPM audible + visual metronome
//    Core 1 — loop()      : TFT display + NeoPixel LED bar refresh
// ============================================================

#include "SPI.h"
#include "HX711.h"                   // Load cell amplifier library
#include "Adafruit_GFX.h"            // Core graphics primitives for TFT
#include "Adafruit_ILI9341.h"        // ILI9341 TFT driver
#include <Wire.h>                    // I2C bus (used by BNO055)
#include <Adafruit_Sensor.h>         // Adafruit unified sensor abstraction
#include <Adafruit_BNO055.h>         // BNO055 9-DOF IMU driver
#include <utility/imumaths.h>        // IMU math helpers
#include <Adafruit_NeoPixel.h>       // WS2812B RGB LED strip driver
#include <WiFi.h>                    // ESP32 Wi-Fi stack
#include <HTTPClient.h>              // HTTP POST to cloud endpoint
#include "esp_sleep.h"               // Light/deep sleep APIs
#include "esp_task_wdt.h"            // Task watchdog timer
#include "esp_system.h"              // esp_reset_reason()
#include "driver/gpio.h"             // Low-level GPIO (pull-ups etc.)
#include "driver/rtc_io.h"           // RTC-domain GPIO (survives sleep)
#include "soc/rtc_cntl_reg.h"        // RTC control registers (brownout)

// --- SPLASH / BOOT ANIMATION IMAGE SUPPORT ---
#include <SdFat_Adafruit_Fork.h>     // SD card FAT filesystem
#include <Adafruit_SPIFlash.h>       // Alternatively, SPI flash storage
#include <Adafruit_ImageReader.h>    // BMP image reader for TFT

// ── Forward declarations ──────────────────────────────────────────────────────
// Declared here so functions defined later can be referenced before they appear.
void taskHX711(void* pvParameters);
void taskIMU(void* pvParameters);
void taskPPG(void* pvParameters);
void taskBuzzer(void* pvParameters);
void prepareWiFiForTraining();
void showModeSelectScreen();
void resetLiveSessionMetrics();
void reinitializeHX711();

// ──────────────────────────────────────────────────────────────────────────────
//  START BUTTON
//  D6 is used as the single physical button: short press = Emergency mode,
//  hold 3 s = Training mode, hold 2 s from sleep = wake.
// ──────────────────────────────────────────────────────────────────────────────
#define START_BUTTON D6

// ──────────────────────────────────────────────────────────────────────────────
//  STORAGE SELECTION
//  Define USE_SD_CARD to load boot BMP files from an SD card;
//  comment it out to use the onboard SPI flash instead.
// ──────────────────────────────────────────────────────────────────────────────
#define USE_SD_CARD
#define SD_CS 4   // SD card chip-select pin

#if defined(USE_SD_CARD)
  SdFat SD;                          // SD FAT filesystem instance
  Adafruit_ImageReader reader(SD);   // BMP reader backed by SD
#else
  // SPI Flash fallback (e.g., Adafruit QSPIFlash on SAMD)
  Adafruit_FlashTransport_SPI flashTransport(SS, &SPI);
  Adafruit_SPIFlash flash(&flashTransport);
  FatVolume filesys;
  Adafruit_ImageReader reader(filesys);
#endif

// ──────────────────────────────────────────────────────────────────────────────
//  HX711 LOAD CELL AMPLIFIER
//  DOUT = serial data out from HX711
//  CLK  = clock pin driven by MCU
// ──────────────────────────────────────────────────────────────────────────────
#define DOUT 3
#define CLK  2

HX711 scale;                         // HX711 driver instance
float calibration_factor = 6500;     // Scale factor (raw counts → lbs);
                                     // adjust via serial '+'/'-' commands

// ──────────────────────────────────────────────────────────────────────────────
//  ILI9341 TFT (240×320 px, SPI)
//  TFT_DC = data/command select
//  TFT_CS = chip select
// ──────────────────────────────────────────────────────────────────────────────
#define TFT_DC 9
#define TFT_CS 10
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// ──────────────────────────────────────────────────────────────────────────────
//  BNO055 9-DOF IMU
//  Connected via I2C at address 0x28 on the Wire bus (pins A4/A5).
// ──────────────────────────────────────────────────────────────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// ──────────────────────────────────────────────────────────────────────────────
//  NEOPIXEL LED STRIPS
//
//  strip       — 15-pixel force/depth bar graph on the main housing
//  rhythmStrip — 13-pixel visual metronome strip (flashes at 110 BPM)
//
//  NOTE: LED_PIN (2) is shared with HX711 CLK — this works because
//  NeoPixel bit-banging only happens on Core 1 while HX711 runs on Core 0,
//  but ideally these should be separate pins in a hardware revision.
// ──────────────────────────────────────────────────────────────────────────────
#define LED_PIN    2
#define LED_COUNT  15
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define RHYTHM_LED_PIN   4
#define RHYTHM_LED_COUNT 13
Adafruit_NeoPixel rhythmStrip(RHYTHM_LED_COUNT, RHYTHM_LED_PIN, NEO_GRB + NEO_KHZ800);

// ──────────────────────────────────────────────────────────────────────────────
//  PULSE OXIMETRY (PPG)
//  PULSE_PIN   — analog input connected to the PulseSensor / photodiode
//  PPG_LED_PIN — indicator LED that flashes on each detected heartbeat
// ──────────────────────────────────────────────────────────────────────────────
#define PULSE_PIN    23
#define PPG_LED_PIN  21

// ──────────────────────────────────────────────────────────────────────────────
//  BUZZER (110 BPM METRONOME)
//  Uses the ESP32 LEDC (PWM) peripheral so the tone is hardware-generated
//  and does not block the CPU.
//  BUZZER_PERIOD_MS = 60 000 / 100 BPM ≈ 600 ms per beat
// ──────────────────────────────────────────────────────────────────────────────
#define BUZZER_PIN       19
#define BUZZER_CHANNEL   0           // LEDC channel 0
#define BUZZER_FREQ_HZ   2000        // 2 kHz tone — clearly audible
#define BUZZER_BEEP_MS   30          // Beep on-time in milliseconds
#define BUZZER_PERIOD_MS 600         // Full beat period (~100 BPM)

// ──────────────────────────────────────────────────────────────────────────────
//  IDLE / AUTO-SLEEP
//  If no significant force change is detected for IDLE_TIMEOUT_MS the device
//  enters low-power mode. IDLE_DELTA_LB is the minimum weight change (lbs)
//  that counts as "activity". WAKE_HOLD_MS is how long the button must be
//  held to wake the device back up.
// ──────────────────────────────────────────────────────────────────────────────
#define IDLE_TIMEOUT_MS   20000UL    // 20 seconds without activity → sleep
#define IDLE_DELTA_LB     10.0f      // ±10 lb change resets the idle timer
#define WAKE_HOLD_MS      2000UL     // Hold button 2 s to wake from sleep

// ──────────────────────────────────────────────────────────────────────────────
//  PPG SIGNAL CONSTANTS
//  FINGER_ON         — ADC threshold below which no finger is detected
//  PPG_AVG_COUNT     — rolling average window (smooths noise)
//  MIN/MAX_BEAT_INTERVAL — clamp valid inter-beat intervals (ms)
//                         equates to ~20–200 BPM
// ──────────────────────────────────────────────────────────────────────────────
const int          FINGER_ON         = 1700;
const int          PPG_AVG_COUNT     = 10;
const int          MIN_BEAT_INTERVAL = 300;   // ≈ 200 BPM max
const int          MAX_BEAT_INTERVAL = 3000;  // ≈  20 BPM min

// ──────────────────────────────────────────────────────────────────────────────
//  CPM (COMPRESSIONS PER MINUTE) DETECTION
//  FORCE_THRESHOLD_LB — minimum force (lbs) to count as a compression
//  CPM_TIMEOUT_MS     — reset CPM if no compression for this long
//  CPM_DEBOUNCE_MS    — ignore edges faster than this (anti-bounce)
//  CPM_INTERVAL_COUNT — number of recent intervals averaged for CPM
// ──────────────────────────────────────────────────────────────────────────────
const float        FORCE_THRESHOLD_LB   = 30.0f;
const unsigned long CPM_TIMEOUT_MS    = 2000UL;
const unsigned long CPM_DEBOUNCE_MS   = 180UL;
const int          CPM_INTERVAL_COUNT = 6;

// ──────────────────────────────────────────────────────────────────────────────
//  WI-FI / CLOUD ENDPOINT
//  Wi-Fi is only enabled during Training mode; it is fully shut down for
//  Emergency mode to conserve power and reduce latency.
// ──────────────────────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "GalaxyS24FE99E2";
const char* WIFI_PASSWORD = "chickenbutt";
const char* SENSOR_ENDPOINT =
  "https://gentle-stone-0c20e761e.6.azurestaticapps.net/api/sensor";

// ──────────────────────────────────────────────────────────────────────────────
//  DEVICE MODES
//  MODE_WAITING   — idle / mode-selection screen
//  MODE_EMERGENCY — real cardiac event; no Wi-Fi, minimal overhead
//  MODE_TRAINING  — 2-minute timed practice session with cloud upload
// ──────────────────────────────────────────────────────────────────────────────
enum DeviceMode {
  MODE_WAITING = 0,
  MODE_EMERGENCY,
  MODE_TRAINING
};

// Training session timing constants
const unsigned long TRAINING_HOLD_MS    = 3000UL;    // Hold time (ms) to select training
const unsigned long TRAINING_SESSION_MS = 120000UL;  // Session length: 2 minutes
const unsigned long TRAINING_SAMPLE_MS  = 3000UL;    // Sample metrics every 3 s

// ──────────────────────────────────────────────────────────────────────────────
//  RTC MEMORY — persists across soft resets and deep sleep cycles.
//  Used to keep a rolling log of recent reset reasons for diagnostics.
// ──────────────────────────────────────────────────────────────────────────────
RTC_DATA_ATTR int     rtc_resetCount    = 0;
RTC_DATA_ATTR uint8_t rtc_resetLog[10]  = {0};  // Circular log of last 10 resets

// ──────────────────────────────────────────────────────────────────────────────
//  RTOS SYNCHRONISATION PRIMITIVES
//  xDataMutex — protects all shared sensor/state variables (g_xxx)
//  xWireMutex — protects the I2C bus (shared between IMU and potentially PPG)
// ──────────────────────────────────────────────────────────────────────────────
SemaphoreHandle_t xDataMutex;
SemaphoreHandle_t xWireMutex;

// ──────────────────────────────────────────────────────────────────────────────
//  SHARED SENSOR STATE
//  All variables prefixed g_ are written by sensor tasks (Core 0) and read
//  by the display loop (Core 1). Every access must be wrapped with
//  xSemaphoreTake(xDataMutex) / xSemaphoreGive(xDataMutex).
// ──────────────────────────────────────────────────────────────────────────────

// — Load cell / force —
volatile float  g_currentWeight    = 0.0f;  // Current filtered force (lbs)
volatile float  g_filteredWeight   = 0.0f;  // Same as above (kept for clarity)
volatile float  g_currentRawWeight = 0.0f;  // Un-filtered raw reading (lbs)
volatile float  g_maxWeight        = 0.0f;  // Peak force seen this session (lbs)
volatile float  g_compressionRate  = 0.0f;  // Rolling-average CPM

// — IMU orientation (relative to calibration baseline) —
volatile float  g_currentHeading   = 0.0f;  // Yaw  delta from baseline (°)
volatile float  g_currentRoll      = 0.0f;  // Roll delta from baseline (°)
volatile float  g_currentPitch     = 0.0f;  // Pitch delta from baseline (°)
volatile float  g_baseHeading      = 0.0f;  // Baseline yaw  captured at startup
volatile float  g_baseRoll         = 0.0f;  // Baseline roll
volatile float  g_basePitch        = 0.0f;  // Baseline pitch
volatile bool   g_imuBaselineReady = false; // True once baseline is captured
char            g_imuPostureStatus[16] = "Waiting";  // "Good" / "Adjust" / "Lean"

// — PPG / heart rate —
volatile int    g_currentSignal = 0;          // Latest PPG ADC reading
volatile int    g_currentBPM    = 0;          // Estimated BPM (0 = no reading)
char            g_ppgStatus[16] = "No heartbeat"; // Human-readable status

// — Control flags (set by UI / idle logic, consumed by tasks) —
volatile bool   g_doTare      = false;  // When true, taskHX711 re-tares the scale
volatile bool   g_resetMax    = false;  // When true, clears g_maxWeight
volatile bool   g_goToSleep   = false;  // When true, loop() triggers low-power mode

// — Idle tracking —
volatile unsigned long g_lastActivityTime = 0;  // millis() of last detected activity

// — Session / mode state —
volatile DeviceMode g_mode           = MODE_WAITING;
volatile bool       g_sessionRunning = false;  // Buzzer/metronome active only when true

// ──────────────────────────────────────────────────────────────────────────────
//  TRAINING SESSION ACCUMULATORS
//  Sampled every TRAINING_SAMPLE_MS; averaged and uploaded at session end.
// ──────────────────────────────────────────────────────────────────────────────
unsigned long g_trainingStartTime   = 0;
unsigned long g_lastTrainingSample  = 0;
bool          g_trainingResultsSent = false;

float g_sumAvgWeight    = 0.0f;
float g_sumMaxWeight    = 0.0f;
float g_sumCompRate     = 0.0f;
float g_sumPPGSignal    = 0.0f;
float g_sumBPM          = 0.0f;
float g_sumPitch        = 0.0f;
float g_sumRoll         = 0.0f;
int   g_trainingSamples = 0;

// ──────────────────────────────────────────────────────────────────────────────
//  TASK HANDLES — used to suspend, resume, or delete tasks dynamically.
// ──────────────────────────────────────────────────────────────────────────────
TaskHandle_t hHX711  = NULL;
TaskHandle_t hIMU    = NULL;
TaskHandle_t hPPG    = NULL;
TaskHandle_t hBuzzer = NULL;

// ──────────────────────────────────────────────────────────────────────────────
//  SENSOR DATA STRUCT — packed before serialising to JSON for cloud upload.
// ──────────────────────────────────────────────────────────────────────────────
struct SensorData {
  String deviceId;
  String timestamp;
  String mode;
  String sessionType;
  int ppg;                   // Raw PPG signal value
  float imuX;                // IMU X (heading) — currently unused in uploads
  float imuY;                // IMU Y (roll)
  float imuZ;                // IMU Z (pitch)
  int compressionRate;       // CPM rounded to int
  float compressionDepth;    // Average force in lbs (proxy for depth)
  int recoil;                // 1 = full recoil detected, 0 = incomplete
  String status;             // Human-readable quality summary
  int sampleCount;           // Number of 3-second samples in session
  unsigned long sessionDurationMs; // Total session duration in ms
};

// ──────────────────────────────────────────────────────────────────────────────
//  UTILITY HELPERS
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Wrap an angle (degrees) into the range [-180, +180].
 * Used when computing relative IMU orientation deltas.
 */
static float wrapAngle(float a) {
  while (a >  180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

/**
 * Convert an esp_reset_reason_t enum value to a human-readable string.
 * Used in the boot diagnostic printout.
 */
static const char* resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXTERNAL";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "OTHER_WDT";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    default:                return "UNKNOWN";
  }
}

/**
 * Returns the current uptime as a string (milliseconds since boot).
 * Used as a lightweight timestamp for cloud payloads when NTP is unavailable.
 */
String getTimestampString() {
  return String(millis());
}

/**
 * Build a brief quality-assessment string for the CPR compressions.
 * @param rate    CPM value
 * @param depth   Force in lbs (used as depth proxy)
 * @param recoil  1 if full recoil was detected
 */
String determineStatus(int rate, float depth, int recoil) {
  if (rate > 125 || depth > 6.2f) return "Bad: too fast/deep";
  if (rate < 100 || depth < 5.0f || recoil == 0) return "Warn: shallow/recoil";
  return "Good compressions";
}

/**
 * Reset all live-session metrics and trigger a scale tare.
 * Called when switching modes or waking from sleep.
 * Must be called from Core 1 (acquires xDataMutex internally).
 */
void resetLiveSessionMetrics() {
  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  g_currentWeight    = 0.0f;
  g_filteredWeight   = 0.0f;
  g_currentRawWeight = 0.0f;
  g_maxWeight        = 0.0f;
  g_compressionRate  = 0.0f;
  g_lastActivityTime = millis();
  g_doTare           = true;   // Signal taskHX711 to re-tare
  g_resetMax         = true;   // Signal taskHX711 to clear peak force
  xSemaphoreGive(xDataMutex);
}

/**
 * Power-cycle the HX711 and re-initialise it with the current
 * calibration factor. Also performs a tare (zero offset).
 * Safe to call from any context that is not already mid-read.
 */
void reinitializeHX711() {
  scale.power_down();
  delay(50);
  scale.begin(DOUT, CLK);
  scale.set_scale(calibration_factor);
  scale.power_up();
  delay(100);
  scale.tare();  // Zero out any resting load
}

// ──────────────────────────────────────────────────────────────────────────────
//  WI-FI HELPERS
//  Wi-Fi is only used in Training mode; these functions manage the lifecycle.
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Try to connect to Wi-Fi, showing progress on the TFT.
 * On failure, shows an "offline" message and waits for button press.
 * @return true if connected, false if timed out.
 */
bool connectToWiFiWithScreen() {
  prepareWiFiForTraining();

  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(20, 80);
  tft.println("Connecting to");
  tft.setCursor(20, 105);
  tft.println("WiFi");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int tries = 0;
  const int MAX_TRIES = 20;  // 20 × 500 ms = 10 s timeout

  while (WiFi.status() != WL_CONNECTED && tries < MAX_TRIES) {
    delay(500);
    tries++;
    // Animate dots so user knows the device is working
    tft.fillRect(20, 140, 200, 24, ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(20, 140);
    for (int d = 0; d < (tries % 4); d++) tft.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    // Connection failed — show offline warning and let user proceed
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
    tft.setCursor(20, 80);
    tft.println("No WiFi");
    tft.setCursor(20, 105);
    tft.println("detected.");
    tft.setTextColor(ILI9341_WHITE);
    tft.setCursor(20, 150);
    tft.println("Training will");
    tft.setCursor(20, 175);
    tft.println("run offline.");
    tft.setCursor(20, 200);
    tft.println("Results will");
    tft.setCursor(20, 225);
    tft.println("not be saved.");
    tft.setTextColor(ILI9341_YELLOW);
    tft.setCursor(20, 275);
    tft.println("Press button");
    tft.setCursor(20, 300);
    tft.println("to continue.");

    // Block until the user acknowledges the offline state
    while (digitalRead(START_BUTTON) == HIGH) delay(10);
    delay(200);
    while (digitalRead(START_BUTTON) == LOW)  delay(10);
    delay(200);
    return false;
  }

  // Success — display IP and enable Wi-Fi sleep to save power between uploads
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);
  tft.setCursor(20, 100);
  tft.println("WiFi connected!");
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20, 130);
  tft.println(WiFi.localIP().toString());
  WiFi.setSleep(true);  // Modem sleep reduces idle current ~50 mA
  delay(1000);
  return true;
}

/**
 * Re-connect if Wi-Fi dropped mid-session (e.g., modem sleep disconnected).
 * Tries for up to 10 seconds before giving up silently.
 */
void ensureWiFiConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost. Reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
      delay(500);
      tries++;
    }
  }
}

/**
 * Fully shut down Wi-Fi (radio off, auto-reconnect disabled).
 * Called when entering Emergency mode or after Training upload completes.
 */
void disableWiFiCompletely() {
  Serial.println("Shutting WiFi down...");
  WiFi.setAutoReconnect(false);
  WiFi.disconnect(true, true);  // Disconnect and erase stored SSID/PW from RAM
  delay(100);
  WiFi.mode(WIFI_OFF);
  delay(100);
}

/**
 * Configure the Wi-Fi driver for a clean STA-mode connection.
 * Disables persistence so no SSID is written to NVS flash.
 */
void prepareWiFiForTraining() {
  WiFi.persistent(false);      // Don't save credentials to flash
  WiFi.setAutoReconnect(false); // We manage reconnection manually
  WiFi.mode(WIFI_STA);
  delay(50);
}

// ──────────────────────────────────────────────────────────────────────────────
//  CLOUD DATA UPLOAD
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Serialise a SensorData struct to a JSON string suitable for the Azure
 * Static Web Apps Function endpoint. Manual serialisation is used to avoid
 * pulling in a large JSON library.
 */
String sensorDataToJson(const SensorData &data) {
  String json = "{";
  json += "\"deviceId\":\"" + data.deviceId + "\",";
  json += "\"timestamp\":\"" + data.timestamp + "\",";
  json += "\"mode\":\"" + data.mode + "\",";
  json += "\"sessionType\":\"" + data.sessionType + "\",";
  json += "\"ppg\":" + String(data.ppg) + ",";
  json += "\"imuX\":" + String(data.imuX, 2) + ",";
  json += "\"imuY\":" + String(data.imuY, 2) + ",";
  json += "\"imuZ\":" + String(data.imuZ, 2) + ",";
  json += "\"compressionRate\":" + String(data.compressionRate) + ",";
  json += "\"compressionDepth\":" + String(data.compressionDepth, 2) + ",";
  json += "\"recoil\":" + String(data.recoil) + ",";
  json += "\"status\":\"" + data.status + "\",";
  json += "\"sampleCount\":" + String(data.sampleCount) + ",";
  json += "\"sessionDurationMs\":" + String(data.sessionDurationMs);
  json += "}";
  return json;
}

/**
 * POST a SensorData payload to the Azure endpoint.
 * Attempts to reconnect Wi-Fi if the link has dropped.
 * @return true on HTTP 2xx response, false otherwise.
 */
bool sendSensorDataToAzure(const SensorData &data) {
  if (WiFi.status() != WL_CONNECTED) {
    ensureWiFiConnected();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("No WiFi — skipping upload.");
      return false;
    }
  }

  HTTPClient http;
  http.begin(SENSOR_ENDPOINT);
  http.addHeader("Content-Type", "application/json");

  String jsonPayload = sensorDataToJson(data);
  Serial.println("Sending JSON:");
  Serial.println(jsonPayload);

  int httpResponseCode = http.POST(jsonPayload);
  String responseBody  = http.getString();

  Serial.print("HTTP Response Code: ");
  Serial.println(httpResponseCode);
  Serial.print("Response Body: ");
  Serial.println(responseBody);

  http.end();
  return (httpResponseCode >= 200 && httpResponseCode < 300);
}

// ──────────────────────────────────────────────────────────────────────────────
//  LED HELPERS  (called from Core 1 only)
// ──────────────────────────────────────────────────────────────────────────────

/** Set all pixels on the main force-bar strip to white (startup/idle state). */
void setStripWhite() {
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(255, 255, 255));
  }
  strip.show();
}

/** Set all pixels on the rhythm strip to a warm amber (flash-on state). */
void setRhythmStripWhite() {
  for (int i = 0; i < rhythmStrip.numPixels(); i++) {
    rhythmStrip.setPixelColor(i, rhythmStrip.Color(255, 180, 80));
  }
  rhythmStrip.show();
}

/** Turn off all rhythm-strip pixels. */
void setRhythmStripOff() {
  rhythmStrip.clear();
  rhythmStrip.show();
}

/**
 * Update the main NeoPixel bar graph to reflect the current compression force.
 *  < 5 lb   → all off  (no meaningful force)
 *  5–75 lb  → yellow   (too light for AHA-guideline compressions)
 *  75–105 lb→ green    (good depth / force range)
 *  > 105 lb → red      (too strong)
 * Force is clamped to 120 lb; the full bar represents 120 lb.
 */
void updateBarGraph(float force) {
  strip.clear();

  uint32_t yellow = strip.Color(255, 165, 0);
  uint32_t green  = strip.Color(0, 255, 0);
  uint32_t red    = strip.Color(255, 0, 0);

  const int barCount = LED_COUNT;

  if (force < 5.0f) {
    strip.show();
    return;  // Nothing to display at near-zero force
  }

  float clampedForce = force;
  if (clampedForce > 120.0f) clampedForce = 120.0f;

  // Map 0–120 lb range onto 0–LED_COUNT pixels
  int ledsToLight = map((int)clampedForce, 0, 120, 0, barCount);
  if (ledsToLight < 0) ledsToLight = 0;
  if (ledsToLight > barCount) ledsToLight = barCount;

  for (int i = 0; i < ledsToLight; i++) {
    if (force < 75.0f) {
      strip.setPixelColor(i, yellow);
    } else if (force <= 105.0f) {
      strip.setPixelColor(i, green);
    } else {
      strip.setPixelColor(i, red);
    }
  }

  strip.show();
}

/**
 * Flash the rhythm strip in sync with the 110 BPM metronome by checking the
 * current phase of the beat period. The first 120 ms of each 600 ms period
 * is the "on" phase. Called every loop() iteration from Core 1.
 */
void updateRhythmStrip() {
  unsigned long now = millis();
  unsigned long phase = now % BUZZER_PERIOD_MS;

  if (phase < 120) {
    setRhythmStripWhite();
  } else {
    setRhythmStripOff();
  }
}

// ──────────────────────────────────────────────────────────────────────────────
//  IMU BASELINE CALIBRATION
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Capture the device's resting orientation as a baseline.
 * All subsequent IMU readings are reported as deltas from this baseline,
 * so tilt detection is relative to how the device was placed on the manikin.
 * Discards 50 warm-up samples, then averages 100 samples over ~1 second.
 */
void captureIMUBaseline() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(10, 100);
  tft.println("Calibrating...");
  tft.setCursor(10, 130);
  tft.println("Keep device still");

  // Warm-up: allow sensor fusion algorithm to settle
  for (int i = 0; i < 50; i++) {
    sensors_event_t ev;
    bno.getEvent(&ev);
    delay(10);
  }

  // Averaging pass
  float sumH = 0, sumR = 0, sumP = 0;
  const int samples = 100;
  for (int i = 0; i < samples; i++) {
    sensors_event_t ev;
    bno.getEvent(&ev);
    sumH += ev.orientation.x;  // Heading (yaw)
    sumR += ev.orientation.y;  // Roll
    sumP += ev.orientation.z;  // Pitch
    delay(10);
  }

  g_baseHeading      = sumH / samples;
  g_baseRoll         = sumR / samples;
  g_basePitch        = sumP / samples;
  g_imuBaselineReady = true;
}

// ──────────────────────────────────────────────────────────────────────────────
//  STATUS HELPERS — translate raw values to human-readable feedback strings
// ──────────────────────────────────────────────────────────────────────────────

// CPM target range per AHA guidelines: 100–120 compressions per minute
const float CPM_LOW_OK  = 100.0f;
const float CPM_HIGH_OK = 120.0f;

// Force (depth proxy) target range in lbs
const float FORCE_LOW_OK  = 75.0f;
const float FORCE_HIGH_OK = 110.0f;

// Tilt magnitude threshold: > 8° from baseline triggers "ADJUST"
const float TILT_WARN_DEG = 8.0f;

/** Return a compression-rate status string based on CPM value. */
const char* getCPMStatus(float cpm) {
  if (cpm < 1.0f)         return "WAITING";   // No compressions detected yet
  if (cpm < CPM_LOW_OK)   return "TOO SLOW";
  if (cpm > CPM_HIGH_OK)  return "TOO FAST";
  return "GOOD";
}

/** Return a force/depth status string. */
const char* getForceStatus(float forceLb) {
  if (forceLb < FORCE_LOW_OK)   return "TOO LIGHT";
  if (forceLb > FORCE_HIGH_OK)  return "TOO STRONG";
  return "GOOD";
}

/**
 * Compute the tilt magnitude from roll and pitch deltas (degrees) and
 * return a posture status string.
 */
const char* getTiltStatus(float roll, float pitch) {
  float tiltMag = sqrtf(roll * roll + pitch * pitch);  // Euclidean tilt angle
  if (tiltMag >= TILT_WARN_DEG) return "ADJUST";
  return "GOOD";
}

/**
 * Return heartbeat detection status. "Heartbeat detect" is shown only when
 * a valid BPM is available AND the finger-on sensor confirms contact.
 */
const char* getHeartbeatStatus(int bpm, const char* ppgStatus) {
  if (bpm > 0 && strcmp(ppgStatus, "No Finger") != 0) {
    return "Heartbeat detect";
  }
  return "Missing heartbeat";
}

// ──────────────────────────────────────────────────────────────────────────────
//  TFT DISPLAY LAYOUT
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Draw the static label text for the Emergency mode display.
 * Dynamic values are overwritten by the loop() update function, so the
 * labels only need to be drawn once per mode entry.
 *
 * Layout (y positions):
 *   y=  0 — "CPM"
 *   y= 70 — "FORCE"
 *   y=150 — "TILT"
 *   y=230 — "PPG"
 */
void drawStaticLabels() {
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);

  tft.setCursor(20, 0);
  tft.println("CPM");

  tft.setCursor(20, 70);
  tft.println("FORCE");

  tft.setCursor(20, 150);
  tft.println("TILT");

  tft.setCursor(20, 230);
  tft.println("PPG");
}

/**
 * Draw the static labels for Training mode display.
 * Layout is slightly tighter (labels at y=0, 60, 140, 220) to leave room
 * for the extra numeric data shown in training.
 */
void drawStaticLabelsTraining() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);

  tft.setCursor(20, 0);
  tft.println("CPM");
  tft.setCursor(20, 60);
  tft.println("FORCE");
  tft.setCursor(20, 140);
  tft.println("TILT");
  tft.setCursor(20, 220);
  tft.println("PPG");
}

// ──────────────────────────────────────────────────────────────────────────────
//  LOW POWER / SLEEP MODE
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Enter a low-power state when the device has been idle for IDLE_TIMEOUT_MS.
 *
 * Steps:
 *  1. Shut down Wi-Fi radio.
 *  2. Suspend all sensor tasks (they remain in RAM but don't execute).
 *  3. Clear LEDs and buzzer.
 *  4. Display "No activity" message and instructions.
 *  5. Enter FreeRTOS light sleep (750 ms wake timer) in a loop.
 *  6. On each light-sleep wakeup, poll the button; require a 2-second hold.
 *  7. On confirmed 2-second hold, re-initialise all peripherals and restart.
 *
 * The watchdog is de-initialised before sleeping to prevent spurious resets,
 * and the brownout detector is disabled to tolerate the power rail droop
 * that sometimes occurs when Wi-Fi shuts down.
 */
void enterLowPowerMode() {
  disableWiFiCompletely();

  // Suspend sensor tasks — they keep their stack but stop being scheduled
  if (hHX711)  vTaskSuspend(hHX711);
  if (hIMU)    vTaskSuspend(hIMU);
  if (hPPG)    vTaskSuspend(hPPG);
  if (hBuzzer) vTaskSuspend(hBuzzer);

  // Clear all outputs
  strip.clear();
  strip.show();
  setRhythmStripOff();
  ledcWrite(BUZZER_CHANNEL, 0);
  ledcDetachPin(BUZZER_PIN);
  digitalWrite(PPG_LED_PIN, LOW);

  // Show sleep screen
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20, 100);
  tft.println("No activity");
  tft.setCursor(20, 125);
  tft.println("detected.");
  tft.setTextSize(2);
  tft.setCursor(20, 165);
  tft.setTextColor(ILI9341_YELLOW);
  tft.println("Hold button");
  tft.setCursor(20, 190);
  tft.println("to wake.");

  // Disable watchdog — the task will not service it while sleeping
  esp_task_wdt_deinit();
  // Disable brownout detector to prevent spurious resets during power transitions
  REG_CLR_BIT(RTC_CNTL_BROWN_OUT_REG, RTC_CNTL_BROWN_OUT_ENA);

  // Ensure wake button has an internal pull-up (button pulls to GND when pressed)
  gpio_pullup_en(GPIO_NUM_6);
  gpio_pulldown_dis(GPIO_NUM_6);

  // Wait for button to be released before entering sleep loop
  while (digitalRead(START_BUTTON) == LOW) delay(10);
  delay(200);

  Serial.println("Low power mode. Hold D6 for 2s to wake.");
  Serial.flush();

  // ── Sleep / poll loop ──────────────────────────────────────────────────────
  while (true) {
    // Light sleep for 750 ms; wakes automatically via timer
    esp_sleep_enable_timer_wakeup(750000ULL);
    esp_light_sleep_start();

    // Button not pressed — go back to sleep
    if (digitalRead(START_BUTTON) == HIGH) continue;

    // Button is pressed — measure how long it has been held
    unsigned long holdStart = millis();
    while (digitalRead(START_BUTTON) == LOW) {
      if (millis() - holdStart >= WAKE_HOLD_MS) {
        // ── WAKE-UP SEQUENCE ──────────────────────────────────────────────
        Serial.println("Button held 2s — waking.");
        Serial.flush();

        // Reset shared state
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
        g_goToSleep        = false;
        g_lastActivityTime = millis();
        g_mode             = MODE_WAITING;
        g_sessionRunning   = false;
        xSemaphoreGive(xDataMutex);

        // Delete old task instances (they were suspended, not running)
        if (hHX711)  { vTaskDelete(hHX711);  hHX711  = NULL; }
        if (hIMU)    { vTaskDelete(hIMU);    hIMU    = NULL; }
        if (hPPG)    { vTaskDelete(hPPG);    hPPG    = NULL; }
        if (hBuzzer) { vTaskDelete(hBuzzer); hBuzzer = NULL; }

        // Re-initialise SPI bus (TFT)
        SPI.end();
        delay(50);
        SPI.begin();

        // Re-initialise I2C bus and BNO055
        Wire.end();
        delay(50);
        Wire.begin(A4, A5);
        Wire.setClock(400000);
        bno.begin();
        delay(500);
        bno.setExtCrystalUse(true);
        delay(200);

        // Re-initialise load cell
        reinitializeHX711();
        resetLiveSessionMetrics();

        // Re-initialise buzzer PWM
        pinMode(BUZZER_PIN, OUTPUT);
        digitalWrite(BUZZER_PIN, LOW);
        ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ_HZ, 8);
        ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
        ledcWrite(BUZZER_CHANNEL, 0);

        // Re-initialise NeoPixel strips
        strip.begin();
        strip.setBrightness(40);
        strip.show();
        setStripWhite();

        rhythmStrip.begin();
        rhythmStrip.setBrightness(40);
        rhythmStrip.show();
        setRhythmStripOff();

        // Restart all sensor tasks
        xTaskCreatePinnedToCore(taskHX711,  "HX711",  4096, NULL, 2, &hHX711,  0);
        xTaskCreatePinnedToCore(taskIMU,    "IMU",    4096, NULL, 2, &hIMU,    0);
        xTaskCreatePinnedToCore(taskPPG,    "PPG",    4096, NULL, 2, &hPPG,    0);
        xTaskCreatePinnedToCore(taskBuzzer, "Buzzer", 2048, NULL, 3, &hBuzzer, 0);

        delay(500);
        captureIMUBaseline();  // Re-capture baseline in current position
        showModeSelectScreen();
        return;  // Exit enterLowPowerMode(); loop() takes over
      }
      delay(10);
    }
    // Button released before 2 s — go back to sleep
  }
}

// ============================================================
//  TASK: HX711  —  Load Cell + CPM Detection  (Core 0)
// ============================================================
/**
 * Continuously reads the HX711 load cell at ~40 Hz (25 ms delay).
 *
 * Responsibilities:
 *  1. Read raw weight, apply exponential moving average (α = 0.25).
 *  2. Detect rising edge through FORCE_THRESHOLD_LB → count compressions.
 *  3. Compute rolling CPM from the last CPM_INTERVAL_COUNT intervals.
 *  4. Reset CPM if no compression for CPM_TIMEOUT_MS.
 *  5. Trigger idle sleep if no weight change for IDLE_TIMEOUT_MS.
 *  6. Honour g_doTare and g_resetMax flags from the UI.
 */
void taskHX711(void* pvParameters) {
  const float weightAlpha = 0.25f;  // EMA smoothing factor (0 = no update, 1 = raw)
  float localFiltered = 0.0f;       // Local copy of filtered weight
  float idleReference = 0.0f;       // Reference weight for idle detection
  bool idleRefSet = false;          // True once idleReference has been set

  // CPM detection state
  bool aboveThreshold = false;               // True while force > threshold
  unsigned long lastCompressionTime = 0;     // Timestamp of most recent compression
  unsigned long lastEdgeTime = 0;            // Timestamp of most recent rising edge
  float intervalHistory[CPM_INTERVAL_COUNT] = {0}; // Circular buffer of CPM values
  int intervalIndex = 0;                     // Write pointer into intervalHistory
  int intervalCount = 0;                     // Valid entries currently in history
  float localRate = 0.0f;                    // Current CPM estimate

  const TickType_t xDelay = pdMS_TO_TICKS(25);  // 40 Hz sample rate

  // Stamp activity at task start to prevent premature sleep
  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  g_lastActivityTime = millis();
  xSemaphoreGive(xDataMutex);

  for (;;) {
    unsigned long now = millis();

    // ── 1. Read load cell ────────────────────────────────────────────────────
    float rawW = 0.0f;
    if (scale.is_ready()) {
      rawW = -scale.get_units(1);  // Negate: weight is applied to top of sensor
    } else {
      // HX711 not ready — reuse last valid reading to avoid stale zero
      xSemaphoreTake(xDataMutex, portMAX_DELAY);
      rawW = g_currentRawWeight;
      xSemaphoreGive(xDataMutex);
    }

    if (rawW < 0.0f) rawW = 0.0f;  // Clamp: negative force is unphysical here

    // ── 2. Check for tare / max-reset flags from UI ──────────────────────────
    bool doTareNow = false;
    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    if (g_doTare)   { g_doTare = false; doTareNow = true; }
    if (g_resetMax) { g_maxWeight = 0.0f; g_resetMax = false; }
    DeviceMode modeNow = g_mode;
    xSemaphoreGive(xDataMutex);

    if (doTareNow) {
      // Full re-initialise: power-cycle HX711, tare, reset CPM state
      reinitializeHX711();
      localFiltered = 0.0f;
      rawW = 0.0f;
      aboveThreshold = false;
      lastCompressionTime = 0;
      lastEdgeTime = 0;
      localRate = 0.0f;
      for (int i = 0; i < CPM_INTERVAL_COUNT; i++) intervalHistory[i] = 0.0f;
      intervalIndex = 0;
      intervalCount = 0;
      idleRefSet = false;

      xSemaphoreTake(xDataMutex, portMAX_DELAY);
      g_currentWeight    = 0.0f;
      g_filteredWeight   = 0.0f;
      g_currentRawWeight = 0.0f;
      g_compressionRate  = 0.0f;
      g_lastActivityTime = now;
      xSemaphoreGive(xDataMutex);
    }

    // ── 3. Exponential moving average ────────────────────────────────────────
    localFiltered = (1.0f - weightAlpha) * localFiltered + weightAlpha * rawW;
    float filtW = localFiltered;

    // ── 4. Idle detection ────────────────────────────────────────────────────
    // Reset idle reference on first sample
    if (!idleRefSet) {
      idleReference = filtW;
      idleRefSet = true;
    }

    // If the filtered weight has changed by ≥ IDLE_DELTA_LB, count as activity
    if (fabsf(filtW - idleReference) >= IDLE_DELTA_LB) {
      idleReference = filtW;  // Slide reference to new weight
      xSemaphoreTake(xDataMutex, portMAX_DELAY);
      g_lastActivityTime = now;
      xSemaphoreGive(xDataMutex);
    }

    // Check if we should go to sleep (only in non-training modes)
    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    unsigned long lastAct = g_lastActivityTime;
    xSemaphoreGive(xDataMutex);

    if ((modeNow == MODE_WAITING || modeNow == MODE_EMERGENCY) &&
        (now - lastAct) >= IDLE_TIMEOUT_MS) {
      xSemaphoreTake(xDataMutex, portMAX_DELAY);
      g_goToSleep = true;
      xSemaphoreGive(xDataMutex);
    }

    // ── 5. CPM: rising-edge detection ────────────────────────────────────────
    bool currentlyAbove = (filtW >= FORCE_THRESHOLD_LB);

    if (!aboveThreshold && currentlyAbove) {
      // Rising edge: force just crossed above the threshold
      if ((now - lastEdgeTime) >= CPM_DEBOUNCE_MS) {
        lastEdgeTime = now;

        if (lastCompressionTime > 0) {
          unsigned long intervalMs = now - lastCompressionTime;
          // Only count intervals that correspond to plausible compression rates
          if (intervalMs >= CPM_DEBOUNCE_MS && intervalMs <= 2000UL) {
            float instCPM = 60000.0f / intervalMs;  // Instantaneous CPM

            // Store in circular buffer and recompute rolling average
            intervalHistory[intervalIndex] = instCPM;
            intervalIndex = (intervalIndex + 1) % CPM_INTERVAL_COUNT;
            if (intervalCount < CPM_INTERVAL_COUNT) intervalCount++;

            float sum = 0.0f;
            for (int i = 0; i < intervalCount; i++) sum += intervalHistory[i];
            localRate = sum / intervalCount;
          }
        }

        lastCompressionTime = now;
      }
    }

    aboveThreshold = currentlyAbove;

    // ── 6. CPM timeout: clear if no compression for 2 s ─────────────────────
    if (lastCompressionTime > 0 && (now - lastCompressionTime) >= CPM_TIMEOUT_MS) {
      localRate = 0.0f;
      intervalCount = 0;
      intervalIndex = 0;
      for (int i = 0; i < CPM_INTERVAL_COUNT; i++) intervalHistory[i] = 0.0f;
      lastCompressionTime = 0;
    }

    // ── 7. Publish to shared state ───────────────────────────────────────────
    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    g_currentRawWeight = rawW;
    g_filteredWeight   = filtW;
    g_currentWeight    = filtW;
    if (filtW > g_maxWeight) g_maxWeight = filtW;  // Track session peak
    g_compressionRate  = localRate;
    xSemaphoreGive(xDataMutex);

    vTaskDelay(xDelay);
  }
}

// ============================================================
//  TASK: IMU / BNO055  —  Orientation Tracking  (Core 0)
// ============================================================
/**
 * Reads BNO055 orientation at ~40 Hz and publishes heading/roll/pitch
 * deltas relative to the calibration baseline. The xWireMutex prevents
 * I2C bus conflicts with other tasks.
 */
void taskIMU(void* pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(25);

  for (;;) {
    sensors_event_t event;

    // Acquire I2C bus, read orientation event, release bus
    xSemaphoreTake(xWireMutex, portMAX_DELAY);
    bno.getEvent(&event);
    xSemaphoreGive(xWireMutex);

    // Read baseline (captured once after mode selection)
    float bH, bR, bP;
    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    bH = g_baseHeading; bR = g_baseRoll; bP = g_basePitch;
    xSemaphoreGive(xDataMutex);

    // Compute deltas, wrapping to [-180, +180]
    float heading = wrapAngle(event.orientation.x - bH);
    float roll    = wrapAngle(event.orientation.y - bR);
    float pitch   = wrapAngle(event.orientation.z - bP);

    // Compute tilt magnitude and translate to status string
    float tiltMag = sqrtf(roll * roll + pitch * pitch);
    const char* status;
    if      (tiltMag < 5.0f)  status = "Good";
    else if (tiltMag < 10.0f) status = "Adjust";
    else                      status = "Lean";

    // Publish
    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    g_currentHeading = heading;
    g_currentRoll    = roll;
    g_currentPitch   = pitch;
    strncpy(g_imuPostureStatus, status, sizeof(g_imuPostureStatus) - 1);
    g_imuPostureStatus[sizeof(g_imuPostureStatus) - 1] = '\0';
    xSemaphoreGive(xDataMutex);

    vTaskDelay(xDelay);
  }
}

// ============================================================
//  TASK: PPG  —  Pulse Oximetry / Heart Rate  (Core 0)
// ============================================================
/**
 * Samples the PPG (photoplethysmography) analog input at ~50 Hz,
 * detects heartbeat peaks, and computes a rolling-average BPM.
 *
 * Algorithm:
 *  1. Read raw ADC and maintain a 10-sample moving average.
 *  2. If signal < FINGER_ON threshold → no finger → zero out BPM.
 *  3. Track adaptive baseline (slow EMA) and peak envelope (fast decay EMA).
 *  4. Compute amplitude = peak - baseline.
 *  5. If amplitude ≥ 30: apply hysteresis thresholds for rise/fall detection.
 *  6. On rising edge: compute instantaneous BPM, add to 4-beat history.
 *  7. If no beat for 3.5 s: declare "Missing heartbeat" and zero BPM.
 */
void taskPPG(void* pvParameters) {
  const TickType_t xDelay = pdMS_TO_TICKS(20);  // ~50 Hz

  // Rolling average buffer for noise reduction
  int   ppgSampleBuffer[PPG_AVG_COUNT] = {};
  int   ppgSampleIndex  = 0;
  bool  ppgBufferFilled = false;

  // Adaptive peak/baseline tracking
  float ppgBaseline  = 0.0f;  // Slow-following baseline (removes DC offset)
  float ppgPeak      = 0.0f;  // Peak envelope (follows signal peaks)
  bool  ppgPulseHigh = false; // True between rise threshold crossing and fall

  // Beat history for averaged BPM
  int   beatHistory[4] = {};
  int   historyIndex   = 0;
  int   validBeatCount = 0;
  unsigned long lastBeatTime = 0;

  for (;;) {
    unsigned long now = millis();
    int raw = analogRead(PULSE_PIN);

    // Accumulate samples into circular buffer
    ppgSampleBuffer[ppgSampleIndex++] = raw;
    if (ppgSampleIndex >= PPG_AVG_COUNT) {
      ppgSampleIndex  = 0;
      ppgBufferFilled = true;
    }

    // Wait until buffer is full before processing
    if (!ppgBufferFilled) { vTaskDelay(xDelay); continue; }

    // Compute moving average
    long sum = 0;
    for (int i = 0; i < PPG_AVG_COUNT; i++) sum += ppgSampleBuffer[i];
    int sig = (int)(sum / PPG_AVG_COUNT);

    // ── Finger detection ─────────────────────────────────────────────────────
    if (sig < FINGER_ON) {
      // No finger on sensor — reset all state
      ppgBaseline    = (float)sig;
      ppgPeak        = (float)sig;
      ppgPulseHigh   = false;
      validBeatCount = 0;
      lastBeatTime   = 0;
      digitalWrite(PPG_LED_PIN, LOW);

      xSemaphoreTake(xDataMutex, portMAX_DELAY);
      g_currentSignal = sig;
      g_currentBPM    = 0;
      strncpy(g_ppgStatus, "No Finger", sizeof(g_ppgStatus) - 1);
      g_ppgStatus[sizeof(g_ppgStatus) - 1] = '\0';
      xSemaphoreGive(xDataMutex);

      vTaskDelay(xDelay);
      continue;
    }

    // Initialise tracking variables on first valid sample
    if (ppgBaseline == 0.0f) ppgBaseline = (float)sig;
    if (ppgPeak     == 0.0f) ppgPeak     = (float)sig;

    // Update adaptive baseline (15% toward current sample)
    ppgBaseline = 0.85f * ppgBaseline + 0.15f * (float)sig;
    // Update peak envelope: rises instantly, decays slowly
    if (sig > ppgPeak) ppgPeak = (float)sig;
    else               ppgPeak = 0.94f * ppgPeak + 0.06f * (float)sig;

    float amplitude = ppgPeak - ppgBaseline;

    const char* status = "Scanning";
    int         bpm    = 0;

    if (amplitude < 30.0f) {
      // Amplitude too small — poor contact or no pulse
      ppgPulseHigh = false;
      digitalWrite(PPG_LED_PIN, LOW);
      status = "Weak Signal";
      bpm = 0;
    } else {
      // Hysteresis: rise at 55% of amplitude above baseline, fall at 35%
      float riseThresh = ppgBaseline + 0.55f * amplitude;
      float fallThresh = ppgBaseline + 0.35f * amplitude;

      if (!ppgPulseHigh && sig >= riseThresh) {
        // ── Rising edge (systolic peak) ────────────────────────────────────
        ppgPulseHigh = true;
        if (lastBeatTime != 0) {
          unsigned long interval = now - lastBeatTime;
          if (interval >= MIN_BEAT_INTERVAL && interval <= MAX_BEAT_INTERVAL) {
            int newBPM = (int)(60000UL / interval);
            beatHistory[historyIndex] = newBPM;
            historyIndex = (historyIndex + 1) % 4;
            if (validBeatCount < 4) validBeatCount++;

            // Average of up to 4 most recent beats
            int total = 0;
            for (int i = 0; i < validBeatCount; i++) total += beatHistory[i];
            bpm    = total / validBeatCount;
            status = (validBeatCount < 4) ? "Scanning" : "Stable";
            digitalWrite(PPG_LED_PIN, HIGH);  // Flash LED on beat
          } else {
            status = "Keep Still";  // Interval out of range → motion artefact
          }
        } else {
          status = "First Beat";  // No previous beat reference yet
          digitalWrite(PPG_LED_PIN, HIGH);
        }
        lastBeatTime = now;

      } else if (ppgPulseHigh && sig <= fallThresh) {
        // ── Falling edge — reset pulse flag ───────────────────────────────
        ppgPulseHigh = false;
        digitalWrite(PPG_LED_PIN, LOW);
      }

      // Preserve last known BPM if no new beat this cycle
      xSemaphoreTake(xDataMutex, portMAX_DELAY);
      if (bpm == 0) bpm = g_currentBPM;
      xSemaphoreGive(xDataMutex);
    }

    // ── Beat timeout: 3.5 s without a beat → zero BPM ──────────────────────
    if (lastBeatTime != 0 && (now - lastBeatTime > 3500)) {
      bpm = 0;
      validBeatCount = 0;
      status = "Missing heartbeat";
    }

    // Publish
    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    g_currentSignal = sig;
    g_currentBPM    = bpm;
    strncpy(g_ppgStatus, status, sizeof(g_ppgStatus) - 1);
    g_ppgStatus[sizeof(g_ppgStatus) - 1] = '\0';
    xSemaphoreGive(xDataMutex);

    vTaskDelay(xDelay);
  }
}

// ============================================================
//  TASK: BUZZER  —  110 BPM Metronome  (Core 0)
// ============================================================
/**
 * Produces a short 2 kHz beep every BUZZER_PERIOD_MS milliseconds when a
 * session is running. Also drives the rhythm NeoPixel strip in sync.
 * The task checks the sleep and session flags each cycle so it can stop
 * cleanly without needing to be explicitly notified.
 *
 * Timing note: The beep and silence are implemented as two sequential
 * vTaskDelay calls rather than one, which gives better timing accuracy
 * than a single delay with an internal on/off toggle.
 */
void taskBuzzer(void* pvParameters) {
  ledcSetup(BUZZER_CHANNEL, BUZZER_FREQ_HZ, 8);  // 8-bit resolution
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcWrite(BUZZER_CHANNEL, 0);  // Start silent

  const TickType_t xBeepDuration  = pdMS_TO_TICKS(BUZZER_BEEP_MS);
  const TickType_t xSilencePeriod = pdMS_TO_TICKS(BUZZER_PERIOD_MS - BUZZER_BEEP_MS);

  for (;;) {
    // Read control flags
    bool sleeping       = false;
    bool sessionRunning = false;
    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    sleeping       = g_goToSleep;
    sessionRunning = g_sessionRunning;
    xSemaphoreGive(xDataMutex);

    if (sleeping || !sessionRunning) {
      // Not active — keep outputs off and poll at low rate
      ledcWrite(BUZZER_CHANNEL, 0);
      setRhythmStripOff();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // ── Beep ON ──────────────────────────────────────────────────────────────
    ledcWrite(BUZZER_CHANNEL, 128);  // 50% duty cycle at 2 kHz
    setRhythmStripWhite();
    vTaskDelay(xBeepDuration);

    // ── Beep OFF ─────────────────────────────────────────────────────────────
    ledcWrite(BUZZER_CHANNEL, 0);
    setRhythmStripOff();
    vTaskDelay(xSilencePeriod);
  }
}

// ──────────────────────────────────────────────────────────────────────────────
//  BOOT SEQUENCE
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Display an 8-frame BMP animation from SD card (or flash) as the boot splash.
 * Images are named Step0.bmp … Step7.bmp in the root of the storage device.
 * Each frame is displayed for 1.25 seconds.
 *
 * Note: Halts with a red error message if the storage device fails to mount.
 */
void showBootSequence() {
#if defined(USE_SD_CARD)
  if (!SD.begin(SD_CS, SD_SCK_MHZ(12))) {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
    tft.setCursor(10, 100);
    tft.println("SD BEGIN FAILED");
    while (1);
  }
#else
  if (!flash.begin()) {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
    tft.setCursor(10, 100);
    tft.println("FLASH BEGIN FAILED");
    while (1);
  }
  if (!filesys.begin(&flash)) {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
    tft.setCursor(10, 100);
    tft.println("FILESYS FAILED");
    while (1);
  }
#endif

  const char* bootFiles[] = {
    "/Step0.bmp",
    "/Step1.bmp",
    "/Step2.bmp",
    "/Step3.bmp",
    "/Step4.bmp",
    "/Step5.bmp",
    "/Step6.bmp",
    "/Step7.bmp"
  };
  for (int i = 0; i < 8; i++) {
      tft.fillScreen(ILI9341_BLACK);
      ImageReturnCode stat = reader.drawBMP(bootFiles[i], tft, 0, 0);
      reader.printStatus(stat);  // Prints "OK" or error to Serial
      delay(1250);
  }
}

// ──────────────────────────────────────────────────────────────────────────────
//  MODE SELECTION UI
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Display the mode-selection idle screen.
 * Short press → Emergency; 3-second hold → Training.
 * A small hint at the bottom explains the hold gesture.
 */
void showModeSelectScreen() {
  strip.clear();
  strip.show();
  setRhythmStripOff();
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextSize(4);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(60, 60);
  tft.println("PRESS");
  tft.setCursor(48, 100);
  tft.println("BUTTON");

  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(96, 140);
  tft.println("TO");
  tft.setCursor(60, 180);
  tft.println("START");

  tft.setTextSize(1);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setCursor(10, 295);
  tft.println("Hold button 3 seconds");
  tft.setCursor(10, 308);
  tft.println("for Training Mode");
}

/**
 * Block until the user presses the start button, then determine which mode
 * was selected based on how long the button was held.
 *
 * @return MODE_TRAINING  if held ≥ TRAINING_HOLD_MS (3 s)
 *         MODE_EMERGENCY otherwise
 */
DeviceMode waitForModeSelection() {
  // Wait for button press (active-low, pulled up)
  while (digitalRead(START_BUTTON) == HIGH) delay(10);

  unsigned long pressStart = millis();
  while (digitalRead(START_BUTTON) == LOW) {
    unsigned long held = millis() - pressStart;

    // Live feedback: show remaining time until Training mode is selected
    tft.fillRect(20, 240, 280, 30, ILI9341_BLACK);
    tft.setTextColor(ILI9341_YELLOW);
    tft.setTextSize(2);
    tft.setCursor(20, 240);

    if (held >= TRAINING_HOLD_MS) {
      tft.print("Training selected");
    } else {
      tft.print("Hold: ");
      tft.print((TRAINING_HOLD_MS - held) / 1000.0f, 1);
      tft.print("s");
    }
    delay(20);
  }

  unsigned long heldTime = millis() - pressStart;
  delay(200);  // Debounce after release

  if (heldTime >= TRAINING_HOLD_MS) return MODE_TRAINING;
  return MODE_EMERGENCY;
}

// ──────────────────────────────────────────────────────────────────────────────
//  MODE ENTRY FUNCTIONS
// ──────────────────────────────────────────────────────────────────────────────

/** Zero out all training session accumulators before a new session starts. */
void resetTrainingAccumulators() {
  g_sumAvgWeight        = 0.0f;
  g_sumMaxWeight        = 0.0f;
  g_sumCompRate         = 0.0f;
  g_sumPPGSignal        = 0.0f;
  g_sumBPM              = 0.0f;
  g_sumPitch            = 0.0f;
  g_sumRoll             = 0.0f;
  g_trainingSamples     = 0;
  g_trainingResultsSent = false;
}

/**
 * Transition to Emergency mode.
 * Wi-Fi is disabled to minimise latency and power draw.
 * The buzzer starts immediately to guide the rescuer.
 */
void startEmergencyMode() {
  disableWiFiCompletely();

  strip.clear();
  strip.show();
  setRhythmStripOff();
  digitalWrite(PPG_LED_PIN, LOW);

  resetLiveSessionMetrics();  // Tare scale and clear peak

  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  g_mode             = MODE_EMERGENCY;
  g_sessionRunning   = true;   // Enables buzzer metronome
  g_goToSleep        = false;
  g_lastActivityTime = millis();
  xSemaphoreGive(xDataMutex);

  drawStaticLabels();

  // Mode badge in top-right corner
  tft.fillRect(130, 0, 110, 20, ILI9341_BLACK);
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(2);
  tft.setCursor(130, 5);
  tft.print("EMERGENCY");
}

/**
 * Transition to Training mode.
 * Attempts Wi-Fi connection (gracefully continues offline if unavailable).
 * Session duration is fixed at TRAINING_SESSION_MS; results are uploaded at end.
 */
void startTrainingMode() {
  resetTrainingAccumulators();
  connectToWiFiWithScreen();  // May show offline message; non-fatal

  resetLiveSessionMetrics();

  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  g_mode             = MODE_TRAINING;
  g_sessionRunning   = true;
  g_goToSleep        = false;
  g_lastActivityTime = millis();
  xSemaphoreGive(xDataMutex);

  g_trainingStartTime  = millis();
  g_lastTrainingSample = 0;

  drawStaticLabelsTraining();

  // Mode badge
  tft.fillRect(130, 0, 110, 20, ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);
  tft.setCursor(130, 5);
  tft.print("TRAINING");
}

// ──────────────────────────────────────────────────────────────────────────────
//  TRAINING SESSION MANAGEMENT
// ──────────────────────────────────────────────────────────────────────────────

/**
 * Build and upload the end-of-session summary to the Azure endpoint.
 * Logs results to Serial regardless of upload success.
 *
 * The "recoil" flag is inferred from the average weight: if < 2.5 lb the
 * rescuer is releasing full pressure, which indicates good recoil.
 */
void sendTrainingAveragesToWebsite(float avgWeight, float avgMaxWeight,
                                   float avgCompRate, float avgPPG,
                                   float avgBPM, float avgPitch,
                                   float avgRoll, int sampleCount) {
  SensorData data;
  data.deviceId          = "nano-esp32-test";
  data.timestamp         = getTimestampString();
  data.mode              = "TRAINING";
  data.sessionType       = "summary";
  data.ppg               = (int)(avgPPG + 0.5f);
  data.imuX              = 0.0f;    // Heading not meaningful for training summary
  data.imuY              = avgRoll;
  data.imuZ              = avgPitch;
  data.compressionRate   = (int)(avgCompRate + 0.5f);
  data.compressionDepth  = avgWeight;
  data.recoil            = (avgWeight <= 2.5f) ? 1 : 0;  // 1 = good recoil
  data.status            = determineStatus(data.compressionRate, data.compressionDepth, data.recoil);
  data.sampleCount       = sampleCount;
  data.sessionDurationMs = TRAINING_SESSION_MS;

  Serial.println("========== TRAINING SESSION SUMMARY ==========");
  Serial.printf("Samples      : %d\n",       sampleCount);
  Serial.printf("Avg Weight   : %.2f lb\n",  avgWeight);
  Serial.printf("Avg Max W    : %.2f lb\n",  avgMaxWeight);
  Serial.printf("Avg CompRate : %.2f cpm\n", avgCompRate);
  Serial.printf("Avg PPG      : %.2f\n",     avgPPG);
  Serial.printf("Avg BPM      : %.2f\n",     avgBPM);
  Serial.printf("Avg Pitch    : %.2f\n",     avgPitch);
  Serial.printf("Avg Roll     : %.2f\n",     avgRoll);
  Serial.println("==============================================");

  bool ok = sendSensorDataToAzure(data);
  Serial.print("Training summary upload: ");
  Serial.println(ok ? "SUCCESS" : "FAILED");
}

/**
 * Called when the 2-minute training session timer expires.
 * Stops the metronome, computes session averages, uploads to cloud,
 * displays the results summary, and returns to MODE_WAITING.
 */
void finishTrainingMode() {
  // Stop metronome
  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  g_sessionRunning = false;
  xSemaphoreGive(xDataMutex);

  // Compute averages (guard against zero samples)
  float avgWeight   = 0.0f, avgMaxW     = 0.0f;
  float avgCompRate = 0.0f, avgPPG      = 0.0f;
  float avgBPM      = 0.0f, avgPitch    = 0.0f, avgRoll = 0.0f;

  if (g_trainingSamples > 0) {
    avgWeight   = g_sumAvgWeight / g_trainingSamples;
    avgMaxW     = g_sumMaxWeight / g_trainingSamples;
    avgCompRate = g_sumCompRate  / g_trainingSamples;
    avgPPG      = g_sumPPGSignal / g_trainingSamples;
    avgBPM      = g_sumBPM       / g_trainingSamples;
    avgPitch    = g_sumPitch     / g_trainingSamples;
    avgRoll     = g_sumRoll      / g_trainingSamples;
  }

  // Upload results (idempotent — g_trainingResultsSent prevents double upload)
  if (!g_trainingResultsSent) {
    sendTrainingAveragesToWebsite(avgWeight, avgMaxW, avgCompRate,
                                  avgPPG, avgBPM, avgPitch, avgRoll,
                                  g_trainingSamples);
    g_trainingResultsSent = true;
  }

  disableWiFiCompletely();

  // ── Results summary screen ─────────────────────────────────────────────────
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);
  tft.setCursor(20, 60);
  tft.println("Training Complete");

  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(20, 100);
  tft.print("Samples: ");
  tft.println(g_trainingSamples);

  tft.setCursor(20, 130);
  tft.print("Avg Weight: ");
  tft.print(avgWeight, 1);
  tft.println(" lb");

  tft.setCursor(20, 160);
  tft.print("Avg Rate: ");
  tft.print(avgCompRate, 1);
  tft.println(" cpm");

  tft.setCursor(20, 190);
  tft.print("Avg BPM: ");
  if (avgBPM > 0.0f) tft.println(avgBPM, 0);
  else               tft.println("--");

  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(20, 240);
  tft.println("Press button again");

  // Return to waiting state — loop() will show mode select screen
  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  g_mode             = MODE_WAITING;
  g_sessionRunning   = false;
  g_lastActivityTime = millis();
  xSemaphoreGive(xDataMutex);
}

/**
 * Called every loop() iteration while in Training mode.
 * Samples sensor data every TRAINING_SAMPLE_MS and updates the countdown timer.
 * Calls finishTrainingMode() when the session duration has elapsed.
 */
void updateTrainingSession() {
  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  DeviceMode currentMode    = g_mode;
  bool       sessionRunning = g_sessionRunning;
  xSemaphoreGive(xDataMutex);

  if (currentMode != MODE_TRAINING || !sessionRunning) return;

  unsigned long now     = millis();
  unsigned long elapsed = now - g_trainingStartTime;

  // Countdown timer display (bottom-right of screen)
  unsigned long remainMs  = (elapsed >= TRAINING_SESSION_MS) ? 0 : (TRAINING_SESSION_MS - elapsed);
  unsigned long remainSec = remainMs / 1000UL;

  tft.fillRect(180, 305, 120, 20, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(180, 305);
  tft.print(remainSec);
  tft.print(" s");

  // ── Take a metric sample every 3 seconds ─────────────────────────────────
  if (now - g_lastTrainingSample >= TRAINING_SAMPLE_MS) {
    g_lastTrainingSample = now;

    float curW, maxW, compRate, roll, pitch;
    int   sig, bpm;

    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    curW     = g_currentWeight;
    maxW     = g_maxWeight;
    compRate = g_compressionRate;
    roll     = g_currentRoll;
    pitch    = g_currentPitch;
    sig      = g_currentSignal;
    bpm      = g_currentBPM;
    xSemaphoreGive(xDataMutex);

    // Accumulate into running sums for end-of-session averaging
    g_sumAvgWeight += curW;
    g_sumMaxWeight += maxW;
    g_sumCompRate  += compRate;
    g_sumPPGSignal += sig;
    g_sumBPM       += bpm;
    g_sumPitch     += pitch;
    g_sumRoll      += roll;
    g_trainingSamples++;

    Serial.printf("[TRAINING SAMPLE %d] W=%.1f Max=%.1f Rate=%.1f PPG=%d BPM=%d Pitch=%.1f Roll=%.1f\n",
                  g_trainingSamples, curW, maxW, compRate, sig, bpm, pitch, roll);
  }

  // ── End session when timer expires ───────────────────────────────────────
  if (elapsed >= TRAINING_SESSION_MS) {
    finishTrainingMode();
  }
}

// ──────────────────────────────────────────────────────────────────────────────
//  SETUP  — runs on Core 1
// ──────────────────────────────────────────────────────────────────────────────
/**
 * One-time initialisation called at power-on.
 * Order:
 *  1. Serial, reset-reason diagnostics, RTC log update
 *  2. Mutexes, TFT, button, LED, buzzer, ADC, Wi-Fi (off)
 *  3. I2C + BNO055
 *  4. Boot animation from SD
 *  5. NeoPixel, HX711
 *  6. Spawn sensor tasks on Core 0
 *  7. Show mode select, wait for user, enter chosen mode
 */
void setup() {
  Serial.begin(115200);
  delay(500);

  // ── Diagnostics: log and print reset reason ───────────────────────────────
  esp_reset_reason_t       rr    = esp_reset_reason();
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  rtc_resetLog[rtc_resetCount % 10] = (uint8_t)rr;
  rtc_resetCount++;

  Serial.println("========================================");
  Serial.printf("Boot #%d\n", rtc_resetCount);
  Serial.printf("THIS reset reason : %s (%d)\n", resetReasonToString(rr), (int)rr);
  Serial.printf("Wakeup cause      : %d (0=power-on, 3=ext1)\n", (int)cause);
  Serial.println("--- Reset history (newest first) ---");
  int total = min(rtc_resetCount, 10);
  for (int i = 0; i < total; i++) {
    int idx = ((rtc_resetCount - 1 - i) + 10) % 10;
    uint8_t reason = rtc_resetLog[idx];
    Serial.printf("  [-%d] %s (%d)\n", i,
                  resetReasonToString((esp_reset_reason_t)reason), (int)reason);
  }
  Serial.println("========================================");

  // ── RTOS primitives ──────────────────────────────────────────────────────
  xDataMutex = xSemaphoreCreateMutex();
  xWireMutex = xSemaphoreCreateMutex();

  // ── TFT ──────────────────────────────────────────────────────────────────
  tft.begin();
  tft.setRotation(0);  // Portrait orientation

  // ── GPIO ─────────────────────────────────────────────────────────────────
  pinMode(START_BUTTON, INPUT_PULLUP);   // Active-low button
  pinMode(PPG_LED_PIN, OUTPUT);
  digitalWrite(PPG_LED_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // ── ADC ──────────────────────────────────────────────────────────────────
  analogReadResolution(12);                              // 0–4095 range
  analogSetPinAttenuation(PULSE_PIN, ADC_11db);          // 0–3.6 V input range

  // ── Wi-Fi off at boot (enabled only for Training mode) ───────────────────
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);

  // ── BNO055 IMU ───────────────────────────────────────────────────────────
  Wire.begin(A4, A5);      // SDA, SCL
  Wire.setClock(400000);   // Fast mode (400 kHz)
  if (!bno.begin()) {
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
    tft.setCursor(10, 100);
    tft.println("BNO055 NOT FOUND");
    while (1);  // Halt: IMU is required for tilt feedback
  }
  delay(1000);
  bno.setExtCrystalUse(true);  // Use external 32 kHz crystal for better accuracy
  delay(500);

  // ── Boot animation ───────────────────────────────────────────────────────
  showBootSequence();

  // ── NeoPixel strips ──────────────────────────────────────────────────────
  strip.begin();
  strip.setBrightness(40);  // ~16% brightness — sufficient indoors
  strip.show();
  setStripWhite();

  rhythmStrip.begin();
  rhythmStrip.setBrightness(40);
  rhythmStrip.show();
  setRhythmStripOff();

  // ── Load cell ────────────────────────────────────────────────────────────
  reinitializeHX711();
  Serial.print("Zero factor: ");
  Serial.println(scale.read_average());

  // ── Initial TFT layout ───────────────────────────────────────────────────
  drawStaticLabels();

  // ── Spawn sensor tasks on Core 0 ─────────────────────────────────────────
  xTaskCreatePinnedToCore(taskHX711, "HX711",  4096, NULL, 2, &hHX711,  0);
  xTaskCreatePinnedToCore(taskIMU,   "IMU",    4096, NULL, 2, &hIMU,    0);
  xTaskCreatePinnedToCore(taskPPG,   "PPG",    4096, NULL, 2, &hPPG,    0);
  xTaskCreatePinnedToCore(taskBuzzer,"Buzzer", 2048, NULL, 3, &hBuzzer, 0);

  // ── Mode selection ───────────────────────────────────────────────────────
  showModeSelectScreen();
  DeviceMode selectedMode = waitForModeSelection();
  captureIMUBaseline();  // Capture baseline in the position device is placed

  if (selectedMode == MODE_TRAINING) startTrainingMode();
  else                               startEmergencyMode();
}

// ──────────────────────────────────────────────────────────────────────────────
//  LOOP  — TFT Display + LED refresh  (Core 1)
// ──────────────────────────────────────────────────────────────────────────────
namespace {
  unsigned long lastLEDUpdate = 0;  // Timestamp of last NeoPixel refresh
  unsigned long lastTFTUpdate = 0;  // Timestamp of last TFT refresh
}

/**
 * Main display and control loop, running on Core 1 at full speed.
 *
 * Responsibilities:
 *  - Check for sleep flag and enter low-power mode if set.
 *  - If in MODE_WAITING, re-show mode select and block for new selection.
 *  - Handle serial calibration commands (+, -, t, r).
 *  - Refresh NeoPixel bar graph at ~40 Hz.
 *  - Refresh TFT status display at 10 Hz.
 *  - Call updateTrainingSession() each iteration when in Training mode.
 */
void loop() {
  unsigned long now = millis();

  // ── Check sleep flag ──────────────────────────────────────────────────────
  bool       shouldSleep = false;
  DeviceMode modeCopy;
  xSemaphoreTake(xDataMutex, portMAX_DELAY);
  shouldSleep = g_goToSleep;
  modeCopy    = g_mode;
  xSemaphoreGive(xDataMutex);

  if (shouldSleep) {
    enterLowPowerMode();
    return;  // enterLowPowerMode() blocks; returns here only after wake
  }

  // ── Re-enter mode selection if session ended ──────────────────────────────
  if (modeCopy == MODE_WAITING) {
    showModeSelectScreen();
    DeviceMode selectedMode = waitForModeSelection();
    captureIMUBaseline();
    if (selectedMode == MODE_TRAINING) startTrainingMode();
    else                               startEmergencyMode();
    return;
  }

  // ── Serial calibration commands ───────────────────────────────────────────
  if (Serial.available()) {
    char c = Serial.read();
    if      (c == '+' || c == 'a') { calibration_factor += 10; scale.set_scale(calibration_factor); }
    else if (c == '-' || c == 'z') { calibration_factor -= 10; scale.set_scale(calibration_factor); }
    else if (c == 't' || c == 'T') { xSemaphoreTake(xDataMutex, portMAX_DELAY); g_doTare   = true; xSemaphoreGive(xDataMutex); }
    else if (c == 'r' || c == 'R') { xSemaphoreTake(xDataMutex, portMAX_DELAY); g_resetMax = true; xSemaphoreGive(xDataMutex); }
  }

  // ── Training session tick ─────────────────────────────────────────────────
  if (modeCopy == MODE_TRAINING) {
    updateTrainingSession();
  }

  // ── NeoPixel bar graph — refresh at ~40 Hz ────────────────────────────────
  if (now - lastLEDUpdate >= 25) {
    lastLEDUpdate = now;
    float w;
    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    w = g_currentWeight;
    xSemaphoreGive(xDataMutex);
    updateBarGraph(w);
  }

  // ── TFT status display — refresh at 10 Hz ────────────────────────────────
  if (now - lastTFTUpdate >= 100) {
    lastTFTUpdate = now;

    // Snapshot all shared values under mutex (minimise lock hold time)
    float curW, compRate, roll, pitch;
    int bpm;
    char ppgStat[16];

    xSemaphoreTake(xDataMutex, portMAX_DELAY);
    curW     = g_currentWeight;
    compRate = g_compressionRate;
    roll     = g_currentRoll;
    pitch    = g_currentPitch;
    bpm      = g_currentBPM;
    strncpy(ppgStat, g_ppgStatus, sizeof(ppgStat) - 1);
    ppgStat[sizeof(ppgStat) - 1] = '\0';
    xSemaphoreGive(xDataMutex);

    // Derive status strings from raw values
    const char* cpmStatus   = getCPMStatus(compRate);
    const char* forceStatus = getForceStatus(curW);
    const char* tiltStatus  = getTiltStatus(roll, pitch);
    const char* heartStatus = getHeartbeatStatus(bpm, ppgStat);

    // Serial monitor log line (useful for debugging without the TFT)
    Serial.printf("CPM: %.1f (%s) | Force: %.1f lb (%s) | Tilt: %s | BPM: %d | %s\n",
                  compRate, cpmStatus, curW, forceStatus, tiltStatus, bpm, heartStatus);

    // ── CPM section (y = 0–55) ────────────────────────────────────────────
    tft.fillRect(0, 20, 320, 35, ILI9341_BLACK);
    if      (strcmp(cpmStatus, "TOO SLOW") == 0) tft.setTextColor(ILI9341_YELLOW);
    else if (strcmp(cpmStatus, "TOO FAST") == 0) tft.setTextColor(ILI9341_RED);
    else if (strcmp(cpmStatus, "GOOD") == 0)     tft.setTextColor(ILI9341_GREEN);
    else                                         tft.setTextColor(ILI9341_WHITE);

    tft.setTextSize(2);
    tft.setCursor(20, 20);
    if (modeCopy == MODE_TRAINING) {
      // Training: show numeric value and status
      tft.print(compRate, 0);
      tft.print(" cpm  ");
      tft.print(cpmStatus);
    } else {
      // Emergency: status only (less screen clutter)
      tft.print(cpmStatus);
    }

    // ── Force section (y = 90–130) ───────────────────────────────────────
    tft.fillRect(0, 90, 320, 36, ILI9341_BLACK);
    if      (strcmp(forceStatus, "TOO LIGHT") == 0)  tft.setTextColor(ILI9341_YELLOW);
    else if (strcmp(forceStatus, "TOO STRONG") == 0) tft.setTextColor(ILI9341_RED);
    else                                             tft.setTextColor(ILI9341_GREEN);

    tft.setCursor(20, 90);
    tft.print(forceStatus);
    if (modeCopy == MODE_TRAINING) {
      // Training: show numeric force value below the status label
      tft.setCursor(20, 112);
      tft.print(curW, 1);
      tft.print(" lb");
    }

    // ── Tilt section (y = 170–210) ───────────────────────────────────────
    tft.fillRect(0, 170, 320, 36, ILI9341_BLACK);
    if (strcmp(tiltStatus, "ADJUST") == 0) tft.setTextColor(ILI9341_RED);
    else                                   tft.setTextColor(ILI9341_GREEN);

    tft.setCursor(20, 170);
    tft.print(tiltStatus);
    if (modeCopy == MODE_TRAINING) {
      // Training: show raw roll/pitch angles
      tft.setCursor(20, 192);
      tft.print("R:");
      tft.print(roll, 1);
      tft.print(" P:");
      tft.print(pitch, 1);
    }

    // ── PPG / Heartbeat section (y = 250–320) ────────────────────────────
    tft.fillRect(0, 250, 320, 70, ILI9341_BLACK);
    if (strcmp(heartStatus, "Heartbeat detect") == 0) tft.setTextColor(ILI9341_GREEN);
    else                                              tft.setTextColor(ILI9341_RED);

    tft.setCursor(20, 250);
    tft.print("BPM: ");
    if (bpm > 0) tft.print(bpm);
    else         tft.print("--");  // No valid reading

    tft.setCursor(20, 272);
    tft.print(heartStatus);
  }
}
