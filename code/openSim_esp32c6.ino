// OpenSim IMU with ESP32
// Syed Razwanul Haque and Nathan Jones
// Oregon State Universit, USA
// Disclaimer: Part of this code modified using AI

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WebServer.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// ======================================================
// XIAO ESP32-C6 pins
// ======================================================

#define BUTTON_PIN D1
#define LED_PIN    D2

#define SDA_PIN 22
#define SCL_PIN 23

#define SD_CS   21
#define SD_SCK  19
#define SD_MISO 20
#define SD_MOSI 18

// ======================================================
// Wi-Fi AP Server
// ======================================================

const char* AP_SSID = "ESP32C6_OpenSim_Syed";
const char* AP_PASS = "12345678";

WebServer server(80);
bool webServerStarted = false;
bool configConfirmed = false;

// Saved configuration file on SD card
const char* CONFIG_FILE = "/imu_config.txt";

// Hold button for this long to enter configuration mode
const unsigned long CONFIG_HOLD_TIME_MS = 3000;

// ======================================================
// TCA9548A + BNO055
// ======================================================

#define TCA_ADDR 0x70
#define MAX_IMUS 8

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Possible OpenSim IMU segment names
const char* availableSegments[MAX_IMUS] = {
  "pelvis_imu",
  "torso_imu",
  "humerus_r_imu",
  "radius_r_imu",
  "hand_r_imu",
  "humerus_l_imu",
  "radius_l_imu",
  "hand_l_imu"
};

// Dynamic user-selected setup
String selectedSegmentNames[MAX_IMUS];
uint8_t selectedChannels[MAX_IMUS];
uint8_t selectedIMUCount = 0;

// ======================================================
// SD
// ======================================================

SPIClass spi = SPIClass(FSPI);
File dataFile;

// ======================================================
// Recording states
// ======================================================

enum RecordState {
  CONFIG_WEB,
  WAITING_INITIAL,
  RECORDING_INITIAL,
  WAITING_MOTION,
  RECORDING_MOTION,
  DONE
};

RecordState state = CONFIG_WEB;

unsigned long startMillis = 0;
unsigned long lastFlush = 0;

// Initial pose recording duration
const unsigned long INITIAL_RECORD_TIME_MS = 3000;

// Button debounce + long-press detection
bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
unsigned long buttonPressStart = 0;
bool longPressHandled = false;
const unsigned long debounceDelay = 50;

// LED blinking
unsigned long lastLedToggle = 0;
bool ledState = LOW;

const unsigned long FAST_BLINK_MS = 100;  // initial pose
const unsigned long SLOW_BLINK_MS = 500;  // motion recording

// ======================================================
// Utility helpers
// ======================================================

String htmlEscape(const String &s) {
  String out = s;
  out.replace("&", "&amp;");
  out.replace("<", "&lt;");
  out.replace(">", "&gt;");
  out.replace("\"", "&quot;");
  return out;
}

bool isSegmentAlreadySelected(const String &name) {
  for (uint8_t i = 0; i < selectedIMUCount; i++) {
    if (selectedSegmentNames[i] == name) {
      return true;
    }
  }
  return false;
}

void clearSelection() {
  selectedIMUCount = 0;
  for (uint8_t i = 0; i < MAX_IMUS; i++) {
    selectedSegmentNames[i] = "";
    selectedChannels[i] = i;
  }
}

bool saveConfigurationToSD() {
  if (SD.exists(CONFIG_FILE)) {
    SD.remove(CONFIG_FILE);
  }

  File cfg = SD.open(CONFIG_FILE, FILE_WRITE);
  if (!cfg) {
    Serial.println("Failed to save IMU configuration.");
    return false;
  }

  cfg.println("# ESP32-C6 OpenSim IMU configuration");
  cfg.println("# format: channel,segment_name");

  for (uint8_t i = 0; i < selectedIMUCount; i++) {
    cfg.print(selectedChannels[i]);
    cfg.print(",");
    cfg.println(selectedSegmentNames[i]);
  }

  cfg.flush();
  cfg.close();

  Serial.print("Saved IMU configuration to ");
  Serial.println(CONFIG_FILE);
  return true;
}

bool loadConfigurationFromSD() {
  clearSelection();

  if (!SD.exists(CONFIG_FILE)) {
    Serial.println("No saved IMU configuration found.");
    return false;
  }

  File cfg = SD.open(CONFIG_FILE, FILE_READ);
  if (!cfg) {
    Serial.println("Failed to open saved IMU configuration.");
    return false;
  }

  while (cfg.available() && selectedIMUCount < MAX_IMUS) {
    String line = cfg.readStringUntil('\n');
    line.trim();

    if (line.length() == 0 || line.startsWith("#")) {
      continue;
    }

    int commaIndex = line.indexOf(',');
    if (commaIndex < 0) {
      continue;
    }

    int channel = line.substring(0, commaIndex).toInt();
    String segment = line.substring(commaIndex + 1);
    segment.trim();

    if (channel >= 0 && channel < MAX_IMUS && segment.length() > 0) {
      selectedChannels[selectedIMUCount] = (uint8_t)channel;
      selectedSegmentNames[selectedIMUCount] = segment;
      selectedIMUCount++;
    }
  }

  cfg.close();

  if (selectedIMUCount == 0) {
    Serial.println("Saved IMU configuration file was empty or invalid.");
    return false;
  }

  Serial.println("Loaded saved IMU configuration:");
  for (uint8_t i = 0; i < selectedIMUCount; i++) {
    Serial.print("TCA Channel ");
    Serial.print(selectedChannels[i]);
    Serial.print(" -> ");
    Serial.println(selectedSegmentNames[i]);
  }

  return true;
}

// ======================================================
// TCA select
// ======================================================

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();

  delay(1);
}

// ======================================================
// LED pattern update
// ======================================================

void updateLED() {
  unsigned long now = millis();

  if (state == CONFIG_WEB && webServerStarted) {
    // Config mode: short double blink every 2 seconds
    if (now - lastLedToggle >= 2000) {
      digitalWrite(LED_PIN, HIGH);
      delay(40);
      digitalWrite(LED_PIN, LOW);
      delay(80);
      digitalWrite(LED_PIN, HIGH);
      delay(40);
      digitalWrite(LED_PIN, LOW);
      lastLedToggle = now;
    }
  }
  else if (state == RECORDING_INITIAL) {
    // Fast blinking
    if (now - lastLedToggle >= FAST_BLINK_MS) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastLedToggle = now;
    }
  }
  else if (state == RECORDING_MOTION) {
    // Slow blinking
    if (now - lastLedToggle >= SLOW_BLINK_MS) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastLedToggle = now;
    }
  }
  else if (state == DONE && webServerStarted) {
    // Download mode: short blink every 2 seconds
    if (now - lastLedToggle >= 2000) {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      lastLedToggle = now;
    }
  }
  else {
    ledState = LOW;
    digitalWrite(LED_PIN, LOW);
  }
}

// ======================================================
// SD init
// ======================================================

bool initSDCard() {
  Serial.println("Initializing SD card...");

  spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, spi)) {
    Serial.println("SD card initialization failed!");
    return false;
  }

  Serial.println("SD card initialized.");
  return true;
}

// ======================================================
// BNO055 init
// ======================================================

bool initBNO(uint8_t channel) {
  tcaSelect(channel);
  delay(100);

  Serial.print("Checking BNO055 on TCA channel ");
  Serial.println(channel);

  if (!bno.begin(OPERATION_MODE_NDOF)) {
    Serial.print("BNO055 NOT detected on TCA channel ");
    Serial.println(channel);
    return false;
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.print("BNO055 detected on TCA channel ");
  Serial.println(channel);

  return true;
}

void initSelectedBNOs() {
  Serial.println();
  Serial.println("Initializing selected BNO055 sensors...");

  for (uint8_t i = 0; i < selectedIMUCount; i++) {
    Serial.print("Selected channel ");
    Serial.print(selectedChannels[i]);
    Serial.print(" -> ");
    Serial.println(selectedSegmentNames[i]);
    initBNO(selectedChannels[i]);
  }
}

// ======================================================
// Calibration print
// ======================================================

void printCalibrationStatus() {
  Serial.println();
  Serial.println("Calibration status:");

  for (uint8_t i = 0; i < selectedIMUCount; i++) {
    uint8_t channel = selectedChannels[i];
    tcaSelect(channel);

    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    Serial.print("Channel ");
    Serial.print(channel);
    Serial.print(" (");
    Serial.print(selectedSegmentNames[i]);
    Serial.print(") | Sys=");
    Serial.print(sys);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);
  }

  Serial.println();
}

// ======================================================
// OpenSim STO header - dynamic selected sensor labels
// ======================================================

void writeSTOHeader(File &file) {
  file.println("DataRate=100.000000");
  file.println("DataType=Quaternion");
  file.println("version=3");
  file.println("OpenSimVersion=4.5");
  file.println("endheader");

  file.print("time");
  Serial.println("DataRate=100.000000");
  Serial.println("DataType=Quaternion");
  Serial.println("version=3");
  Serial.println("OpenSimVersion=4.5");
  Serial.println("endheader");
  Serial.print("time");

  for (uint8_t i = 0; i < selectedIMUCount; i++) {
    file.print("\t");
    file.print(selectedSegmentNames[i]);
    Serial.print("\t");
    Serial.print(selectedSegmentNames[i]);
  }

  file.println();
  Serial.println();
  file.flush();
}

// ======================================================
// Read quaternion from one IMU
// ======================================================

String readQuaternionString(uint8_t channel) {
  tcaSelect(channel);

  imu::Quaternion q = bno.getQuat();

  float q0 = q.w();
  float q1 = q.x();
  float q2 = q.y();
  float q3 = q.z();

  char qString[80];
  snprintf(qString, sizeof(qString), "%.6f,%.6f,%.6f,%.6f", q0, q1, q2, q3);

  return String(qString);
}

// ======================================================
// Save one OpenSim row - dynamic selected channels
// ======================================================

void saveOpenSimRow() {
  float time_sec = (millis() - startMillis) / 1000.0;

  if (dataFile) {
    dataFile.print(time_sec, 4);
  }

  Serial.print(time_sec, 4);

  for (uint8_t i = 0; i < selectedIMUCount; i++) {
    String q = readQuaternionString(selectedChannels[i]);

    if (dataFile) {
      dataFile.print("\t");
      dataFile.print(q);
    }

    Serial.print("\t");
    Serial.print(q);
  }

  if (dataFile) {
    dataFile.println();
  }

  Serial.println();
}

// ======================================================
// Web Server: common page header/footer
// ======================================================

String pageStart(const String &title) {
  String html = "";
  html += "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>" + title + "</title>";
  html += "<style>";
  html += "body{font-family:Arial;margin:20px;background:#fafafa;}";
  html += "h2{color:#222;margin-bottom:6px;}";
  html += ".small{color:#666;font-size:14px;}";
  html += ".card{background:white;border:1px solid #ddd;border-radius:10px;padding:14px;margin:12px 0;}";
  html += ".row{display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid #eee;padding:8px 0;}";
  html += ".row:last-child{border-bottom:none;}";
  html += ".btn{display:inline-block;padding:8px 12px;background:#222;color:#fff;text-decoration:none;border-radius:6px;font-weight:bold;}";
  html += ".plus{display:inline-block;width:34px;height:34px;line-height:34px;text-align:center;background:#0a7;color:white;text-decoration:none;border-radius:50%;font-size:24px;font-weight:bold;}";
  html += ".disabled{display:inline-block;width:34px;height:34px;line-height:34px;text-align:center;background:#bbb;color:white;text-decoration:none;border-radius:50%;font-size:18px;}";
  html += ".danger{background:#a22;}";
  html += "table{width:100%;border-collapse:collapse;}";
  html += "td,th{border:1px solid #ddd;padding:8px;text-align:left;}";
  html += "th{background:#f0f0f0;}";
  html += "a.file{display:block;padding:10px;margin:6px 0;background:#eee;text-decoration:none;color:#111;border-radius:6px;}";
  html += "</style></head><body>";
  return html;
}

String pageEnd() {
  return "</body></html>";
}

// ======================================================
// Web Server: setup/config page
// ======================================================

void handleConfigPage() {
  String html = pageStart("ESP32-C6 OpenSim Setup");
  html += "<h2>ESP32-C6 OpenSim IMU Setup</h2>";
  html += "<p class='small'>Select body locations. Each selected location is assigned to the next TCA channel automatically. Confirmed setup is saved on the SD card.</p>";

  html += "<div class='card'><h3>Available OpenSim IMU body locations</h3>";
  for (uint8_t i = 0; i < MAX_IMUS; i++) {
    String seg = String(availableSegments[i]);
    bool already = isSegmentAlreadySelected(seg);
    html += "<div class='row'><span>" + seg + "</span>";
    if (already || selectedIMUCount >= MAX_IMUS) {
      html += "<span class='disabled'>&#10003;</span>";
    } else {
      html += "<a class='plus' href='/add?seg=" + seg + "'>+</a>";
    }
    html += "</div>";
  }
  html += "</div>";

  html += "<div class='card'><h3>Selected channel mapping</h3>";
  if (selectedIMUCount == 0) {
    html += "<p>No sensors selected yet.</p>";
  } else {
    html += "<table><tr><th>TCA Channel</th><th>OpenSim Segment Name</th></tr>";
    for (uint8_t i = 0; i < selectedIMUCount; i++) {
      html += "<tr><td>Channel " + String(selectedChannels[i]) + "</td><td>" + htmlEscape(selectedSegmentNames[i]) + "</td></tr>";
    }
    html += "</table>";
  }
  html += "</div>";

  html += "<p>";
  html += "<a class='btn' href='/confirm'>Confirm Setup</a> ";
  html += "<a class='btn danger' href='/reset_selection'>Reset Selection</a>";
  html += "</p>";

  html += "<p class='small'>After confirmation, Wi-Fi will turn off and the device will enter normal button recording mode.</p>";
  html += pageEnd();
  server.send(200, "text/html", html);
}

void handleAddSegment() {
  if (!server.hasArg("seg")) {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
    return;
  }

  String seg = server.arg("seg");

  if (selectedIMUCount < MAX_IMUS && !isSegmentAlreadySelected(seg)) {
    selectedSegmentNames[selectedIMUCount] = seg;
    selectedChannels[selectedIMUCount] = selectedIMUCount;  // selected order maps to TCA channel 0,1,2,...
    selectedIMUCount++;
  }

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleResetSelection() {
  clearSelection();

  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void handleConfirmSetup() {
  if (selectedIMUCount == 0) {
    String html = pageStart("Setup Error");
    html += "<h2>No IMU location selected</h2>";
    html += "<p>Please select at least one body location before confirming setup.</p>";
    html += "<p><a class='btn' href='/'>Back to setup</a></p>";
    html += pageEnd();
    server.send(400, "text/html", html);
    return;
  }

  saveConfigurationToSD();

  String html = pageStart("Setup Confirmed");
  html += "<h2>Setup confirmed and saved</h2>";
  html += "<p>The device will now turn Wi-Fi off and enter normal recording mode.</p>";
  html += "<div class='card'><h3>Selected mapping</h3><table><tr><th>TCA Channel</th><th>Segment</th></tr>";
  for (uint8_t i = 0; i < selectedIMUCount; i++) {
    html += "<tr><td>Channel " + String(selectedChannels[i]) + "</td><td>" + htmlEscape(selectedSegmentNames[i]) + "</td></tr>";
  }
  html += "</table></div>";
  html += "<p class='small'>Button sequence: Click 1 = calibration, Click 2 = motion recording, Click 3 = stop and restart Wi-Fi for download.</p>";
  html += pageEnd();
  server.send(200, "text/html", html);

  configConfirmed = true;
}

void finishConfigurationAndStartRecordingMode() {
  Serial.println();
  Serial.println("Configuration confirmed.");
  Serial.println("Selected IMU mapping:");
  for (uint8_t i = 0; i < selectedIMUCount; i++) {
    Serial.print("TCA Channel ");
    Serial.print(selectedChannels[i]);
    Serial.print(" -> ");
    Serial.println(selectedSegmentNames[i]);
  }

  delay(1000);

  if (webServerStarted) {
    server.stop();
    webServerStarted = false;
  }

  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  initSelectedBNOs();
  printCalibrationStatus();

  Serial.println("Ready for recording.");
  Serial.println("Button sequence:");
  Serial.println("Click 1 = record initial pose for 3 seconds, LED blinks fast");
  Serial.println("Click 2 = start motion recording, LED blinks slow");
  Serial.println("Click 3 = stop motion recording, LED off, then Wi-Fi download server starts");

  state = WAITING_INITIAL;
  configConfirmed = false;
}

// ======================================================
// Web Server: download file list
// ======================================================

void handleDownloadPage() {
  String html = pageStart("ESP32-C6 OpenSim Download");
  html += "<h2>ESP32-C6 OpenSim IMU Logger</h2>";
  html += "<p class='small'>Click a file to download from SD card.</p>";

  File root = SD.open("/");
  if (!root) {
    server.send(500, "text/html", "<h2>Failed to open SD root.</h2>");
    return;
  }

  File file = root.openNextFile();
  bool foundFile = false;

  while (file) {
    String fileName = String(file.name());

    if (!file.isDirectory()) {
      foundFile = true;

      html += "<a class='file' href='/download?file=";
      html += fileName;
      html += "'>";
      html += fileName;
      html += " <span class='small'>(";
      html += String(file.size());
      html += " bytes)</span></a>";
    }

    file.close();
    file = root.openNextFile();
  }

  root.close();

  if (!foundFile) {
    html += "<p>No files found on SD card.</p>";
  }

  html += "<hr>";
  html += "<p class='small'>Default files: placement_orientations.sto and walking_orientations.sto</p>";
  html += pageEnd();

  server.send(200, "text/html", html);
}

// ======================================================
// Web Server: root router
// ======================================================

void handleRoot() {
  if (state == CONFIG_WEB) {
    handleConfigPage();
  } else if (state == DONE) {
    handleDownloadPage();
  } else {
    String html = pageStart("ESP32-C6 OpenSim Logger");
    html += "<h2>Recording mode active</h2>";
    html += "<p>Wi-Fi should be off during recording. Use the button to continue the recording sequence.</p>";
    html += pageEnd();
    server.send(200, "text/html", html);
  }
}

// ======================================================
// Web Server: Download file
// ======================================================

void handleDownload() {
  if (!server.hasArg("file")) {
    server.send(400, "text/plain", "Missing file parameter");
    return;
  }

  String fileName = server.arg("file");

  if (!fileName.startsWith("/")) {
    fileName = "/" + fileName;
  }

  if (!SD.exists(fileName)) {
    server.send(404, "text/plain", "File not found");
    return;
  }

  File file = SD.open(fileName, FILE_READ);

  if (!file) {
    server.send(500, "text/plain", "Failed to open file");
    return;
  }

  String downloadName = fileName;
  downloadName.replace("/", "");

  server.sendHeader("Content-Disposition", "attachment; filename=" + downloadName);
  server.streamFile(file, "application/octet-stream");
  file.close();
}

// ======================================================
// Start Wi-Fi AP + Web Server
// ======================================================

void registerWebRoutes() {
  server.on("/", handleRoot);
  server.on("/add", handleAddSegment);
  server.on("/reset_selection", handleResetSelection);
  server.on("/confirm", handleConfirmSetup);
  server.on("/download", handleDownload);
}

void startWebServer(const char* purpose) {
  if (webServerStarted) {
    return;
  }

  Serial.println();
  Serial.print("Starting Wi-Fi web server for ");
  Serial.println(purpose);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  IPAddress ip = WiFi.softAPIP();

  server.begin();
  webServerStarted = true;

  Serial.println("====================================");
  Serial.print("Wi-Fi AP Name: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASS);
  Serial.print("Open browser: http://");
  Serial.println(ip);
  Serial.println("====================================");
  Serial.println();
}

// ======================================================
// Start initial pose recording
// ======================================================

void startInitialRecording() {
  Serial.println();
  Serial.println("START INITIAL POSE RECORDING");
  Serial.println("Keep selected body segments in neutral pose. Recording for 3 seconds...");

  if (SD.exists("/placement_orientations.sto")) {
    SD.remove("/placement_orientations.sto");
  }

  dataFile = SD.open("/placement_orientations.sto", FILE_WRITE);

  if (!dataFile) {
    Serial.println("Failed to open placement_orientations.sto");
    return;
  }

  writeSTOHeader(dataFile);

  startMillis = millis();
  lastFlush = millis();

  lastLedToggle = millis();
  ledState = LOW;
  digitalWrite(LED_PIN, LOW);

  state = RECORDING_INITIAL;
}

// ======================================================
// Stop initial pose recording
// ======================================================

void stopInitialRecording() {
  if (dataFile) {
    dataFile.flush();
    dataFile.close();
  }

  digitalWrite(LED_PIN, LOW);
  ledState = LOW;

  Serial.println("STOP INITIAL POSE RECORDING");
  Serial.println("Saved: /placement_orientations.sto");
  Serial.println();
  Serial.println("Now press button again to start motion recording.");

  printCalibrationStatus();

  state = WAITING_MOTION;
}

// ======================================================
// Start motion recording
// ======================================================

void startMotionRecording() {
  Serial.println();
  Serial.println("START MOTION RECORDING");

  if (SD.exists("/walking_orientations.sto")) {
    SD.remove("/walking_orientations.sto");
  }

  dataFile = SD.open("/walking_orientations.sto", FILE_WRITE);

  if (!dataFile) {
    Serial.println("Failed to open walking_orientations.sto");
    return;
  }

  writeSTOHeader(dataFile);

  startMillis = millis();
  lastFlush = millis();

  lastLedToggle = millis();
  ledState = LOW;
  digitalWrite(LED_PIN, LOW);

  state = RECORDING_MOTION;
}

// ======================================================
// Stop motion recording
// ======================================================

void stopMotionRecording() {
  if (dataFile) {
    dataFile.flush();
    dataFile.close();
  }

  digitalWrite(LED_PIN, LOW);
  ledState = LOW;

  Serial.println("STOP MOTION RECORDING");
  Serial.println("Saved: /walking_orientations.sto");
  Serial.println("Done. Both OpenSim files are saved on SD card.");

  printCalibrationStatus();

  state = DONE;

  // Start Wi-Fi server only AFTER recording is fully stopped
  startWebServer("file download");
}

// ======================================================
// Enter configuration mode by long button press
// ======================================================

void enterConfigurationMode() {
  Serial.println();
  Serial.println("Long button press detected: entering configuration mode.");
  Serial.println("Current saved/loaded mapping will be shown in the web interface.");
  Serial.println("Use Reset Selection on the web page if you want to start from an empty setup.");

  if (dataFile) {
    dataFile.flush();
    dataFile.close();
  }

  state = CONFIG_WEB;
  configConfirmed = false;
  startWebServer("IMU body-location setup");
}

void handleShortButtonClick() {
  if (state == CONFIG_WEB) {
    Serial.println("Configure IMU locations from the web interface first.");
  }
  else if (state == WAITING_INITIAL) {
    startInitialRecording();
  }
  else if (state == WAITING_MOTION) {
    startMotionRecording();
  }
  else if (state == RECORDING_MOTION) {
    stopMotionRecording();
  }
  else if (state == RECORDING_INITIAL) {
    Serial.println("Initial pose recording is automatic. Wait 3 seconds.");
  }
  else if (state == DONE) {
    Serial.println("Recording done. Web server should already be running.");
    Serial.println("Connect to Wi-Fi: ESP32C6_OpenSim_Syed");
    Serial.println("Open browser: http://192.168.4.1");
  }
}

void handleLongButtonPress() {
  if (state == WAITING_INITIAL) {
    enterConfigurationMode();
  }
  else {
    Serial.println("Long press ignored. Reconfiguration is allowed only before recording starts.");
  }
}

// ======================================================
// Button handling
// ======================================================

void checkButton() {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
    lastButtonReading = reading;
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableButtonState) {
      stableButtonState = reading;

      if (stableButtonState == LOW) {
        buttonPressStart = millis();
        longPressHandled = false;
      }
      else {
        if (!longPressHandled) {
          handleShortButtonClick();
        }
      }
    }

    if (stableButtonState == LOW && !longPressHandled) {
      if (millis() - buttonPressStart >= CONFIG_HOLD_TIME_MS) {
        longPressHandled = true;
        handleLongButtonPress();
      }
    }
  }
}

// ======================================================
// Setup
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, LOW);

  WiFi.mode(WIFI_OFF);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println();
  Serial.println("XIAO ESP32-C6 + TCA9548A + BNO055 + OpenSim STO Logger + Saved Dynamic Web Setup");
  Serial.println();

  if (!initSDCard()) {
    Serial.println("Fix SD card before recording.");

    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(200);
      digitalWrite(LED_PIN, LOW);
      delay(200);
    }
  }

  clearSelection();
  registerWebRoutes();

  if (loadConfigurationFromSD()) {
    Serial.println("SD card verified and saved configuration loaded.");
    Serial.println("Starting normal recording mode using the saved body-segment mapping.");
    Serial.println("Hold the button for 3 seconds before recording if you want to reconfigure sensors.");

    WiFi.mode(WIFI_OFF);
    initSelectedBNOs();
    printCalibrationStatus();

    state = WAITING_INITIAL;

    Serial.println("Ready for recording.");
    Serial.println("Button sequence:");
    Serial.println("Click 1 = record initial pose for 3 seconds, LED blinks fast");
    Serial.println("Click 2 = start motion recording, LED blinks slow");
    Serial.println("Click 3 = stop motion recording, LED off, then Wi-Fi download server starts");
  }
  else {
    Serial.println("SD card verified. No saved setup found, starting configuration web interface...");
    state = CONFIG_WEB;
    startWebServer("IMU body-location setup");

    Serial.println("Open the web interface and select body locations.");
    Serial.println("After confirming setup, Wi-Fi will turn off and button recording mode will start.");
  }
}

// ======================================================
// Main loop
// ======================================================

void loop() {
  updateLED();

  if (webServerStarted) {
    server.handleClient();
  }

  if (state == CONFIG_WEB) {
    if (configConfirmed) {
      finishConfigurationAndStartRecordingMode();
    }
    return;
  }

  checkButton();

  if (state == RECORDING_INITIAL) {
    saveOpenSimRow();

    if (millis() - startMillis >= INITIAL_RECORD_TIME_MS) {
      stopInitialRecording();
    }

    delay(5);
  }

  else if (state == RECORDING_MOTION) {
    saveOpenSimRow();

    if (millis() - lastFlush > 1000) {
      if (dataFile) {
        dataFile.flush();
      }
      lastFlush = millis();
    }

    delay(5);
  }
}
