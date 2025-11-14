#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>

// Pin definitions
#define FILL_SOLENOID_PIN 25
#define EMPTY_SOLENOID_PIN 26
#define MEASURING_RELAY_PIN 27 // Single relay for both measuring lights (NO=Green, NC=Red)
#define SDA_PIN 21             // I2C Bus 1 for MPU6050 and OLED1
#define SCL_PIN 22             // I2C Bus 1 for MPU6050 and OLED1
#define SDA2_PIN 18            // I2C Bus 2 for DS3231 and OLED2
#define SCL2_PIN 19            // I2C Bus 2 for DS3231 and OLED2

// I2C Addresses
#define MPU6050_ADDRESS 0x68
#define DS3231_ADDRESS 0x68 // Same as MPU6050 - handled by different libraries
#define OLED_ADDRESS 0x3C   // Both OLEDs use same address but different I2C buses

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED1_ADDRESS 0x3C
#define OLED2_ADDRESS 0x3D

// Serial logging buffer configuration
#define SERIAL_BUFFER_SIZE 100 // Reduced buffer size

// Optimized circular buffer for serial messages
struct LogMessage
{
  unsigned long timestamp;
  char message[80]; // Fixed size message buffer
};

LogMessage serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
int totalMessages = 0;

// Global objects
Adafruit_MPU6050 mpu;
TwoWire I2C_1 = TwoWire(0); // I2C Bus 1 for MPU6050 and OLED1
TwoWire I2C_2 = TwoWire(1); // I2C Bus 2 for DS3231 and OLED2
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_1, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_2, OLED_RESET);
RTC_DS3231 rtc;
WebServer server(80);

// Add these global variables for non-blocking measurement
enum MeasurementState
{
  IDLE,
  EMPTYING_INITIAL,
  FILLING,
  WAITING_TO_SETTLE,
  MEASURING,
  EMPTYING_FINAL
};

MeasurementState measurementState = IDLE;
unsigned long stateStartTime = 0;
float angleSum = 0.0;
int validReadings = 0;
int measurementCount = 0;
unsigned long lastAngleReadTime = 0;

// Updated Configuration structure with angle ranges
struct Config
{
  float desiredDensity = 1.025;
  int measurementInterval = 30; // minutes
  int fillDuration = 5;         // seconds
  int waitDuration = 60;        // seconds
  int measurementDuration = 10; // seconds
  int emptyDuration = 120;      // seconds
  float calibrationOffset = 0.0;
  float calibrationScale = 1.0;
  float lastMeasurementValue = 0.0;
  unsigned long lastMeasurementTime = 0; // Unix timestamp
  float targetAngleMin = 40.0;
  float targetAngleMax = 45.0;
  float lastMeasurementAngle = 0.0;
  bool autoMeasurementEnabled = false; // NEW: Default to disabled
} config;

// Global variables
float currentAngle = 0.0;
float currentDensity = 0.0;
float lastMeasurement = 0.0;
DateTime lastMeasurementTime;
DateTime nextMeasurementTime;
bool isMeasuring = false;
bool isManualMode = false;
unsigned long lastMeasurementMillis = 0;
bool rtcAvailable = false;

unsigned long lastDisplayUpdate = 0;
int displayPage = 0; // 0 or 1 for alternating pages

// Function prototypes
void initializeSystem();
void loadConfig();
void saveConfig();
void createDefaultConfig();
void calculateNextMeasurementTime();
void setupWiFiHotspot();
void setupWebServer();
void handleWebRequests();
void performMeasurement();
void controlRelays();
void updateDisplays();
void saveMeasurementData(float density, float angle, DateTime timestamp);
String getMeasurementData();
void deleteMeasurementData();
float angleToDensity(float angle);
void calibrateMPU();
void setDateTime(int year, int month, int day, int hour, int minute, int second);
void scanI2CDevices();
bool checkRTCConnection();
bool checkMPUConnection();
void updateMeasurementState();
void logSerial(String message);
String getFileList();
bool deleteFile(String filename);
String getFileInfo(String filename);
String formatTime(DateTime dt);
void handleSerial();
void setupEnhancedSerialEndpoints();
void serialPrintln(const char *message);
void handleSerialText();
void clearSerialBuffer();

// Enhanced logSerial function with better formatting and timestamp handling
void logSerial(String message)
{
  DateTime now;
  String timestampStr;

  if (rtcAvailable)
  {
    now = rtc.now();
    timestampStr = formatTime(now);
  }
  else
  {
    timestampStr = "??:??:??";
  }

  // Create the complete log message with milliseconds for better precision
  String logMessage = "[" + timestampStr + "] " + message;

  // Print to both Serial (PlatformIO terminal) and web buffer
  Serial.println(logMessage);

  // Add to circular buffer for web interface
  strncpy(serialBuffer[serialBufferIndex].message, logMessage.c_str(), 79);
  serialBuffer[serialBufferIndex].message[79] = '\0'; // Ensure null termination
  serialBuffer[serialBufferIndex].timestamp = millis();

  serialBufferIndex = (serialBufferIndex + 1) % SERIAL_BUFFER_SIZE;
  totalMessages++;
}

// Enhanced function to get serial buffer for web interface with proper ordering
String getSerialBuffer()
{
  String result = "";
  result.reserve(SERIAL_BUFFER_SIZE * 100); // Pre-allocate space for better performance

  int start = (serialBufferIndex - min(totalMessages, SERIAL_BUFFER_SIZE) + SERIAL_BUFFER_SIZE) % SERIAL_BUFFER_SIZE;
  int count = min(totalMessages, SERIAL_BUFFER_SIZE);

  for (int i = 0; i < count; i++)
  {
    int index = (start + i) % SERIAL_BUFFER_SIZE;
    result += serialBuffer[index].message;
    result += "\n";
  }

  return result;
}

// Updated setupWebServer() function - add these endpoints to your existing setup
void setupEnhancedSerialEndpoints()
{
  // Enhanced serial output endpoint
  server.on("/api/serial", HTTP_GET, []()
            {
    DynamicJsonDocument doc(4096);
    doc["output"] = getSerialBuffer();
    doc["totalMessages"] = totalMessages;
    doc["bufferSize"] = SERIAL_BUFFER_SIZE;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response); });

  // Add API endpoint to clear serial buffer
  server.on("/api/serial/clear", HTTP_POST, []()
            {
    clearSerialBuffer();
    logSerial("Serial buffer cleared via web interface");
    server.send(200, "application/json", "{\"status\":\"success\"}"); });
}

// Function to clear serial buffer
void clearSerialBuffer()
{
  serialBufferIndex = 0;
  totalMessages = 0;
  memset(serialBuffer, 0, sizeof(serialBuffer));
}

// Alternative logSerial function that matches GPS tracker style exactly
void serialPrintln(const char *message)
{
  DateTime now;
  String timestampStr;

  if (rtcAvailable)
  {
    now = rtc.now();
    timestampStr = formatTime(now);
  }
  else
  {
    timestampStr = "??:??:??";
  }

  // Create timestamped message
  String logMessage = "[" + timestampStr + "] " + String(message);

  // Print to Serial
  Serial.println(logMessage);

  // Store in buffer with exact GPS tracker logic
  strncpy(serialBuffer[serialBufferIndex].message, logMessage.c_str(), sizeof(serialBuffer[0].message) - 1);
  serialBuffer[serialBufferIndex].message[sizeof(serialBuffer[0].message) - 1] = '\0';
  serialBuffer[serialBufferIndex].timestamp = millis();

  serialBufferIndex = (serialBufferIndex + 1) % SERIAL_BUFFER_SIZE;
  if (totalMessages < SERIAL_BUFFER_SIZE)
    totalMessages++;
}

// Enhanced web handler for serial with text response (GPS tracker style)
void handleSerialText()
{
  String logs;
  logs.reserve(SERIAL_BUFFER_SIZE * 100); // Pre-allocate space

  int start = (serialBufferIndex - totalMessages + SERIAL_BUFFER_SIZE) % SERIAL_BUFFER_SIZE;
  for (int i = 0; i < totalMessages; i++)
  {
    int index = (start + i) % SERIAL_BUFFER_SIZE;
    logs += String(serialBuffer[index].timestamp);
    logs += ": ";
    logs += serialBuffer[index].message;
    logs += "\n";
  }

  server.send(200, "text/plain", logs);
}

// Enhanced format time function with better precision
String formatTime(DateTime dt)
{
  char buffer[32];
  sprintf(buffer, "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
  return String(buffer);
}

void setup()
{
  Serial.begin(115200);

  // Initialize pins
  pinMode(FILL_SOLENOID_PIN, OUTPUT);
  pinMode(EMPTY_SOLENOID_PIN, OUTPUT);
  pinMode(MEASURING_RELAY_PIN, OUTPUT);

  // Ensure solenoids are closed and measuring light shows red (relay OFF = NC = Red light)
  digitalWrite(FILL_SOLENOID_PIN, HIGH);
  digitalWrite(EMPTY_SOLENOID_PIN, HIGH);
  digitalWrite(MEASURING_RELAY_PIN, HIGH); // LOW = Red light (NC), HIGH = Green light (NO)

  // Initialize I2C buses with custom pins
  I2C_1.begin(SDA_PIN, SCL_PIN);   // Primary I2C bus
  I2C_2.begin(SDA2_PIN, SCL2_PIN); // Secondary I2C bus
  I2C_1.setClock(100000);          // Set I2C clock to 100kHz for stability
  I2C_2.setClock(100000);          // Set I2C clock to 100kHz for stability

  // Initialize system first (this will initialize RTC)
  initializeSystem();

  // Now scan I2C devices after RTC is initialized
  scanI2CDevices();

  // Setup WiFi hotspot
  setupWiFiHotspot();

  // Setup web server
  setupWebServer();

  logSerial("Claybath density measurement system initialized");
}

// Enhanced web handler for serial data with JSON response
void handleSerial()
{
  DynamicJsonDocument doc(4096);
  doc["output"] = getSerialBuffer();
  doc["totalMessages"] = totalMessages;
  doc["bufferSize"] = SERIAL_BUFFER_SIZE;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void loop()
{
  server.handleClient();
  updateMeasurementState();

  // Check for automatic measurement - NEW: Check if auto-measurement is enabled
  if (measurementState == IDLE && !isManualMode &&
      config.autoMeasurementEnabled && // NEW: Only trigger if enabled
      nextMeasurementTime.unixtime() > 0 &&
      rtc.now().unixtime() >= nextMeasurementTime.unixtime())
  {
    logSerial("Automatic measurement triggered");
    performMeasurement();
  }

  updateDisplays();
  controlRelays();
  delay(10);
}

void initializeSystem()
{
  // Initialize LittleFS
  if (!LittleFS.begin())
  {
    logSerial("LittleFS initialization failed!");
    return;
  }

  // Load configuration
  loadConfig();

  // Initialize DS3231 RTC on I2C Bus 2
  if (!rtc.begin(&I2C_2))
  {
    logSerial("Couldn't find DS3231 RTC on I2C Bus 2");
    rtcAvailable = false;
  }
  else
  {
    rtcAvailable = true;
    logSerial("DS3231 RTC initialized successfully on I2C Bus 2");

    // Check if RTC lost power and if so, set the time
    if (rtc.lostPower())
    {
      logSerial("RTC lost power, setting time to compile time");
      // Set to compile time as fallback
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    // Print current time
    DateTime now = rtc.now();
    logSerial("Current time: " + String(now.year()) + "/" +
              String(now.month()) + "/" + String(now.day()) + " " +
              String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()));
  }

  // Initialize MPU6050 on I2C Bus 1
  if (!mpu.begin(MPU6050_ADDRESS, &I2C_1))
  {
    logSerial("Failed to find MPU6050 chip on I2C Bus 1");
    while (1)
    {
      delay(10);
    }
  }
  logSerial("MPU6050 initialized successfully on I2C Bus 1");

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Allow sensor to stabilize
  delay(100);

  // Initialize displays on separate I2C buses
  if (!display1.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS))
  {
    logSerial("SSD1306 allocation failed for display 1");
  }
  else
  {
    logSerial("OLED Display 1 initialized successfully on I2C_1");
    // Flip display 1 horizontally and vertically
    display1.setRotation(2); // 180 degree rotation (flip both horizontal and vertical)
  }

  if (!display2.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS))
  {
    logSerial("SSD1306 allocation failed for display 2");
  }
  else
  {
    logSerial("OLED Display 2 initialized successfully on I2C_2");
    // Flip display 2 horizontally and vertically
    display2.setRotation(2); // 180 degree rotation (flip both horizontal and vertical)
  }

  display1.clearDisplay();
  display2.clearDisplay();
  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);
  display2.setTextSize(1);
  display2.setTextColor(SSD1306_WHITE);
  ;

  delay(2000);

  // Calculate next measurement time based on last measurement
  calculateNextMeasurementTime();

  logSerial("System initialization complete");
}

// FIXED: calculateNextMeasurementTime function
void calculateNextMeasurementTime()
{
  if (!rtcAvailable)
  {
    nextMeasurementTime = DateTime((uint32_t)0);
    logSerial("RTC not available, no automatic measurement scheduled");
    return;
  }

  DateTime now = rtc.now();

  // Check if we have a valid last measurement time
  if (config.lastMeasurementTime > 0)
  {
    DateTime lastMeasurement = DateTime(config.lastMeasurementTime);

    // Check if last measurement was today
    if (lastMeasurement.day() == now.day() &&
        lastMeasurement.month() == now.month() &&
        lastMeasurement.year() == now.year())
    {
      // Last measurement was today, calculate next measurement time
      uint32_t nextTime = config.lastMeasurementTime + (config.measurementInterval * 60);
      nextMeasurementTime = DateTime(nextTime);

      // If the calculated next time is in the past, don't schedule automatic measurement
      if (nextMeasurementTime.unixtime() <= now.unixtime())
      {
        nextMeasurementTime = DateTime((uint32_t)0);
        logSerial("Calculated next measurement time is in the past, no automatic measurement scheduled");
      }
      else
      {
        logSerial("Next measurement scheduled for: " + formatTime(nextMeasurementTime) +
                  " on " + String(nextMeasurementTime.day()) + "/" +
                  String(nextMeasurementTime.month()) + "/" + String(nextMeasurementTime.year()));
      }
    }
    else
    {
      // Last measurement was not today, no automatic measurement scheduled
      nextMeasurementTime = DateTime((uint32_t)0);
      logSerial("No automatic measurement scheduled (last measurement was not today)");
    }
  }
  else
  {
    // No previous measurement exists, no automatic measurement scheduled
    nextMeasurementTime = DateTime((uint32_t)0);
    logSerial("No previous measurement found, no automatic measurement scheduled");
    logSerial("First measurement of the day must be started manually");
  }
}

void loadConfig()
{
  if (LittleFS.exists("/settings.json"))
  {
    File file = LittleFS.open("/settings.json", "r");
    if (file)
    {
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, file);

      if (error)
      {
        logSerial("Failed to parse settings.json, creating new one");
        file.close();
        createDefaultConfig();
        return;
      }

      config.desiredDensity = doc["desiredDensity"] | 1.025;
      config.measurementInterval = doc["measurementInterval"] | 30;
      config.fillDuration = doc["fillDuration"] | 5;
      config.waitDuration = doc["waitDuration"] | 60;
      config.measurementDuration = doc["measurementDuration"] | 10;
      config.emptyDuration = doc["emptyDuration"] | 120;
      config.calibrationOffset = doc["calibrationOffset"] | 0.0;
      config.calibrationScale = doc["calibrationScale"] | 1.0;
      config.lastMeasurementValue = doc["lastMeasurementValue"] | 0.0;
      config.lastMeasurementTime = doc["lastMeasurementTime"] | 0;
      config.targetAngleMin = doc["targetAngleMin"] | 40.0;
      config.targetAngleMax = doc["targetAngleMax"] | 45.0;
      config.lastMeasurementAngle = doc["lastMeasurementAngle"] | 0.0;
      config.autoMeasurementEnabled = doc["autoMeasurementEnabled"] | false; // NEW

      file.close();
      logSerial("Configuration loaded from settings.json");

      if (config.lastMeasurementTime > 0)
      {
        lastMeasurement = config.lastMeasurementValue;
        currentAngle = config.lastMeasurementAngle;
        lastMeasurementTime = DateTime(config.lastMeasurementTime);
        logSerial("Last measurement restored: " + String(lastMeasurement, 3) +
                  " (angle: " + String(config.lastMeasurementAngle, 1) + "°) at " +
                  String(lastMeasurementTime.timestamp()));
      }

      // Log auto-measurement status
      logSerial("Automatic measurements: " + String(config.autoMeasurementEnabled ? "ENABLED" : "DISABLED"));
    }
    else
    {
      logSerial("Failed to open settings.json, creating new one");
      createDefaultConfig();
    }
  }
  else
  {
    logSerial("settings.json not found, creating default configuration");
    createDefaultConfig();
  }
}

void createDefaultConfig()
{
  config.desiredDensity = 1.025;
  config.measurementInterval = 30;
  config.fillDuration = 5;
  config.waitDuration = 5;
  config.measurementDuration = 10;
  config.emptyDuration = 5;
  config.calibrationOffset = 0.0;
  config.calibrationScale = 1.0;
  config.lastMeasurementValue = 0.0;
  config.lastMeasurementTime = 0;
  config.targetAngleMin = 40.0;
  config.targetAngleMax = 45.0;
  config.lastMeasurementAngle = 0.0;
  config.autoMeasurementEnabled = false; // NEW: Default to disabled

  saveConfig();
  logSerial("Default configuration created and saved");
}

void saveConfig()
{
  DynamicJsonDocument doc(1024);
  doc["desiredDensity"] = config.desiredDensity;
  doc["measurementInterval"] = config.measurementInterval;
  doc["fillDuration"] = config.fillDuration;
  doc["waitDuration"] = config.waitDuration;
  doc["measurementDuration"] = config.measurementDuration;
  doc["emptyDuration"] = config.emptyDuration;
  doc["calibrationOffset"] = config.calibrationOffset;
  doc["calibrationScale"] = config.calibrationScale;
  doc["lastMeasurementValue"] = config.lastMeasurementValue;
  doc["lastMeasurementTime"] = config.lastMeasurementTime;
  doc["targetAngleMin"] = config.targetAngleMin;
  doc["targetAngleMax"] = config.targetAngleMax;
  doc["lastMeasurementAngle"] = config.lastMeasurementAngle;
  doc["autoMeasurementEnabled"] = config.autoMeasurementEnabled; // NEW

  File file = LittleFS.open("/settings.json", "w");
  if (file)
  {
    serializeJson(doc, file);
    file.close();
    logSerial("Configuration saved to settings.json");
  }
  else
  {
    logSerial("Failed to save configuration to settings.json");
  }
}

void setupWiFiHotspot()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ClaybathDensityMeter", "12345678");

  logSerial("WiFi Hotspot started");
  logSerial("IP address: " + WiFi.softAPIP().toString());
}

void setupWebServer()
{
  // Serve static files from LittleFS
  server.serveStatic("/", LittleFS, "/index.html");

  // API endpoints
  server.on("/api/status", HTTP_GET, []()
            {
  DynamicJsonDocument doc(1024);
  doc["currentAngle"] = currentAngle;
  doc["currentDensity"] = currentDensity;
  doc["lastMeasurement"] = lastMeasurement;
  doc["lastMeasurementTime"] = config.lastMeasurementTime;
  doc["lastMeasurementAngle"] = config.lastMeasurementAngle;
  doc["nextMeasurementTime"] = nextMeasurementTime.unixtime() > 0 ? nextMeasurementTime.unixtime() : 0;
  doc["isMeasuring"] = isMeasuring;
  doc["isManualMode"] = isManualMode;
  doc["hasScheduledMeasurement"] = nextMeasurementTime.unixtime() > 0;
  doc["autoMeasurementEnabled"] = config.autoMeasurementEnabled; // NEW
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response); });

  // Add new API endpoint for file list
  server.on("/api/files", HTTP_GET, []()
            {
    String fileList = getFileList();
    server.send(200, "application/json", fileList); });

  // Updated API endpoint for serial output
  server.on("/api/serial", HTTP_GET, []()
            {
    DynamicJsonDocument doc(4096);
    doc["output"] = getSerialBuffer();
    doc["totalMessages"] = totalMessages;
    doc["bufferSize"] = SERIAL_BUFFER_SIZE;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response); });

  // Add API endpoint to clear serial buffer
  server.on("/api/serial/clear", HTTP_POST, []()
            {
    clearSerialBuffer();
    logSerial("Serial buffer cleared via web interface");
    server.send(200, "application/json", "{\"status\":\"success\"}"); });

  // FIXED: Add missing GET handler for individual file downloads
  server.on("/api/file", HTTP_GET, []()
            {
    if (server.hasArg("name")) {
      String filename = server.arg("name");
      
      // Add leading slash if not present
      if (!filename.startsWith("/")) {
        filename = "/" + filename;
      }
      
      if (LittleFS.exists(filename)) {
        File file = LittleFS.open(filename, "r");
        if (file) {
          // Set appropriate headers for file download
          String contentType = "application/octet-stream";
          if (filename.endsWith(".csv")) {
            contentType = "text/csv";
          } else if (filename.endsWith(".json")) {
            contentType = "application/json";
          }
          
          server.sendHeader("Content-Disposition", "attachment; filename=\"" + filename.substring(1) + "\"");
          server.streamFile(file, contentType);
          file.close();
          
          logSerial("File downloaded: " + filename);
        } else {
          server.send(500, "application/json", "{\"error\":\"failed_to_open_file\"}");
        }
      } else {
        server.send(404, "application/json", "{\"error\":\"file_not_found\"}");
      }
    } else {
      server.send(400, "application/json", "{\"error\":\"filename_required\"}");
    } });

  // Add new API endpoint for individual file operations (DELETE)
  server.on("/api/file", HTTP_DELETE, []()
            {
    if (server.hasArg("name")) {
      String filename = server.arg("name");
      bool success = deleteFile(filename);
      
      DynamicJsonDocument doc(256);
      doc["success"] = success;
      doc["message"] = success ? "File deleted successfully" : "Failed to delete file";
      
      String response;
      serializeJson(doc, response);
      server.send(success ? 200 : 400, "application/json", response);
    } else {
      server.send(400, "application/json", "{\"error\":\"filename_required\"}");
    } });

  server.on("/api/config", HTTP_GET, []()
            {
  DynamicJsonDocument doc(1024);
  doc["desiredDensity"] = config.desiredDensity;
  doc["measurementInterval"] = config.measurementInterval;
  doc["fillDuration"] = config.fillDuration;
  doc["waitDuration"] = config.waitDuration;
  doc["measurementDuration"] = config.measurementDuration;
  doc["emptyDuration"] = config.emptyDuration;
  doc["calibrationOffset"] = config.calibrationOffset;
  doc["calibrationScale"] = config.calibrationScale;
  doc["lastMeasurementValue"] = config.lastMeasurementValue;
  doc["lastMeasurementTime"] = config.lastMeasurementTime;
  doc["targetAngleMin"] = config.targetAngleMin;
  doc["targetAngleMax"] = config.targetAngleMax;
  doc["lastMeasurementAngle"] = config.lastMeasurementAngle;
  doc["autoMeasurementEnabled"] = config.autoMeasurementEnabled; // NEW
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response); });

  // ADD THIS NEW HANDLER HERE:
  server.on("/api/config", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, body);
      
      if (error) {
        server.send(400, "application/json", "{\"error\":\"invalid_json\"}");
        return;
      }
      
      // Update config values if they exist in the request
      if (doc.containsKey("desiredDensity")) 
        config.desiredDensity = doc["desiredDensity"];
      if (doc.containsKey("measurementInterval")) 
        config.measurementInterval = doc["measurementInterval"];
      if (doc.containsKey("fillDuration")) 
        config.fillDuration = doc["fillDuration"];
      if (doc.containsKey("waitDuration")) 
        config.waitDuration = doc["waitDuration"];
      if (doc.containsKey("measurementDuration")) 
        config.measurementDuration = doc["measurementDuration"];
      if (doc.containsKey("emptyDuration")) 
        config.emptyDuration = doc["emptyDuration"];
      if (doc.containsKey("calibrationOffset")) 
        config.calibrationOffset = doc["calibrationOffset"];
      if (doc.containsKey("calibrationScale")) 
        config.calibrationScale = doc["calibrationScale"];
      if (doc.containsKey("targetAngleMin")) 
        config.targetAngleMin = doc["targetAngleMin"];
      if (doc.containsKey("targetAngleMax")) 
        config.targetAngleMax = doc["targetAngleMax"];
      if (doc.containsKey("autoMeasurementEnabled")) 
        config.autoMeasurementEnabled = doc["autoMeasurementEnabled"];
      
      // Save the updated configuration
      saveConfig();
      
      logSerial("Configuration updated via web interface");
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"no_data\"}");
    }
  });

  server.on("/api/measure", HTTP_POST, []()
            {
    if (!isMeasuring) {
      performMeasurement();
      logSerial("Manual measurement started via web interface");
      server.send(200, "application/json", "{\"status\":\"measurement_started\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"measurement_in_progress\"}");
    } });

  server.on("/api/control", HTTP_POST, []()
            {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      DynamicJsonDocument doc(512);
      deserializeJson(doc, body);
      
      String action = doc["action"];
      bool state = doc["state"];
      
      if (action == "fill_solenoid") {
        digitalWrite(FILL_SOLENOID_PIN, state ? LOW : HIGH);
        logSerial("Fill solenoid " + String(state ? "activated" : "deactivated") + " via web interface");
      } else if (action == "empty_solenoid") {
        digitalWrite(EMPTY_SOLENOID_PIN, state ? LOW : HIGH);
        logSerial("Empty solenoid " + String(state ? "activated" : "deactivated") + " via web interface");
      } else if (action == "measuring_relay") {
        digitalWrite(MEASURING_RELAY_PIN, state ? LOW : HIGH);
        logSerial("Measuring relay " + String(state ? "activated" : "deactivated") + " via web interface");
      }
      
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"no_data\"}");
    } });

  server.on("/api/datetime", HTTP_POST, []()
            {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      DynamicJsonDocument doc(512);
      deserializeJson(doc, body);
      
      int year = doc["year"];
      int month = doc["month"];
      int day = doc["day"];
      int hour = doc["hour"];
      int minute = doc["minute"];
      int second = doc["second"];
      
      // Set DS3231 RTC time
      rtc.adjust(DateTime(year, month, day, hour, minute, second));
      
      // Verify the time was set
      DateTime now = rtc.now();
      logSerial("RTC time set to: " + String(now.year()) + "/" + 
                String(now.month()) + "/" + String(now.day()) + " " + 
                String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()));
      
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"no_data\"}");
    } });

  server.on("/api/data", HTTP_GET, []()
            {
    String data = getMeasurementData();
    server.send(200, "text/plain", data); });

  server.on("/api/data", HTTP_DELETE, []()
            {
    deleteMeasurementData();
    logSerial("All measurement data deleted via web interface");
    server.send(200, "application/json", "{\"status\":\"success\"}"); });

  // Serve the main page
  server.on("/", []()
            {
    if (LittleFS.exists("/index.html")) {
      File file = LittleFS.open("/index.html", "r");
      server.streamFile(file, "text/html");
      file.close();
    } else {
      server.send(404, "text/plain", "index.html not found");
    } });

  // Handle 404
  server.onNotFound([]()
                    { server.send(404, "text/plain", "Not Found"); });

  server.begin();
  logSerial("Web server started");
}

// File management functions
String getFileList()
{
  DynamicJsonDocument doc(2048);
  JsonArray files = doc.createNestedArray("files");

  File root = LittleFS.open("/");
  File file = root.openNextFile();

  while (file)
  {
    if (!file.isDirectory())
    {
      JsonObject fileObj = files.createNestedObject();
      fileObj["name"] = String(file.name());
      fileObj["size"] = file.size();

      // Get file modification time if available
      time_t t = file.getLastWrite();
      fileObj["lastModified"] = t;
    }
    file = root.openNextFile();
  }

  String result;
  serializeJson(doc, result);
  return result;
}

bool deleteFile(String filename)
{
  if (filename.startsWith("/"))
  {
    return LittleFS.remove(filename);
  }
  else
  {
    return LittleFS.remove("/" + filename);
  }
}

String getFileInfo(String filename)
{
  if (!filename.startsWith("/"))
  {
    filename = "/" + filename;
  }

  if (LittleFS.exists(filename))
  {
    File file = LittleFS.open(filename, "r");
    if (file)
    {
      DynamicJsonDocument doc(512);
      doc["name"] = filename;
      doc["size"] = file.size();
      doc["lastModified"] = file.getLastWrite();
      file.close();

      String result;
      serializeJson(doc, result);
      return result;
    }
  }
  return "{}";
}

// Replace the blocking performMeasurement() function with this non-blocking version
void performMeasurement()
{
  if (measurementState == IDLE)
  {
    // Start the measurement sequence
    measurementState = EMPTYING_INITIAL;
    stateStartTime = millis();
    isMeasuring = true;

    // Reset measurement variables
    angleSum = 0.0;
    validReadings = 0;
    measurementCount = 0;
    lastAngleReadTime = 0;

    // Ensure empty solenoid is closed
    digitalWrite(EMPTY_SOLENOID_PIN, HIGH);

    logSerial("Starting measurement sequence...");
  }
}

// Add this function to handle the measurement state machine
void updateMeasurementState()
{
  if (measurementState == IDLE)
  {
    return;
  }

  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - stateStartTime;

  switch (measurementState)
  {
  case EMPTYING_INITIAL:
    if (elapsedTime >= 1000)
    { // 1 second delay
      // Step 2: Fill chamber
      digitalWrite(FILL_SOLENOID_PIN, LOW);
      measurementState = FILLING;
      stateStartTime = currentTime;
      logSerial("Filling chamber...");
    }
    break;

  case FILLING:
    if (elapsedTime >= (config.fillDuration * 1000))
    {
      digitalWrite(FILL_SOLENOID_PIN, HIGH);
      measurementState = WAITING_TO_SETTLE;
      stateStartTime = currentTime;
      logSerial("Waiting for settling...");
    }
    break;

  case WAITING_TO_SETTLE:
    if (elapsedTime >= (config.waitDuration * 1000))
    {
      measurementState = MEASURING;
      stateStartTime = currentTime;
      lastAngleReadTime = currentTime;
      logSerial("Starting angle measurements...");
    }
    break;

  case MEASURING:
    // Take angle readings every second
    if (currentTime - lastAngleReadTime >= 1000)
    {
      if (measurementCount < config.measurementDuration)
      {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        // Calculate angle from accelerometer
        float angle = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;

        // Validate reading
        if (abs(angle) < 90)
        { // Reasonable angle range
          angleSum += angle;
          validReadings++;
        }

        measurementCount++;
        lastAngleReadTime = currentTime;

        logSerial("Measurement " + String(measurementCount) + "/" +
                  String(config.measurementDuration) + " - Angle: " +
                  String(angle, 2) + "°");
      }
      else
      {
        // Measurement complete, process results
        if (validReadings > 0)
        {
          currentAngle = (angleSum / validReadings) + config.calibrationOffset;
          currentDensity = angleToDensity(currentAngle * config.calibrationScale);
          lastMeasurement = currentDensity;
          lastMeasurementTime = rtc.now();

          // Update config with new measurement data including angle
          config.lastMeasurementValue = currentDensity;
          config.lastMeasurementAngle = currentAngle;
          config.lastMeasurementTime = lastMeasurementTime.unixtime();
          saveConfig(); // Save the measurement data to settings

          // Log measurement details
          logSerial("Measurement completed - Angle: " + String(currentAngle, 2) +
                    "°, Density: " + String(currentDensity, 4) +
                    ", Valid readings: " + String(validReadings) + "/" +
                    String(config.measurementDuration));

          // Save measurement data including angle
          saveMeasurementData(currentDensity, currentAngle, lastMeasurementTime);
        }
        else
        {
          logSerial("No valid readings obtained during measurement");
        }

        // Move to emptying phase
        digitalWrite(EMPTY_SOLENOID_PIN, LOW);
        measurementState = EMPTYING_FINAL;
        stateStartTime = currentTime;
        logSerial("Emptying chamber...");
      }
    }
    break;

  case EMPTYING_FINAL:
    if (elapsedTime >= (config.emptyDuration * 1000))
    {
      digitalWrite(EMPTY_SOLENOID_PIN, HIGH);

      // Calculate next measurement time based on current measurement
      DateTime now = rtc.now();
      // Convert minutes to seconds: config.measurementInterval * 60
      nextMeasurementTime = DateTime((uint32_t)(now.unixtime() + (config.measurementInterval * 60)));

      // Reset state
      measurementState = IDLE;
      isMeasuring = false;

      logSerial("Measurement sequence complete");
      logSerial("Next measurement scheduled for: " + String(nextMeasurementTime.timestamp()));
    }
    break;
  }
}

// Update the controlRelays function to use the state machine
void controlRelays()
{
  // Control measuring pilot lamp using single relay
  // LOW = Red light (NC), HIGH = Green light (NO)
  if (isMeasuring)
  {
    digitalWrite(MEASURING_RELAY_PIN, HIGH); // Green light (NO)
  }
  else
  {
    digitalWrite(MEASURING_RELAY_PIN, LOW); // Red light (NC)
  }
}

// Updated updateDisplays function with angle information
void updateDisplays()
{
  DateTime now = rtc.now();

  // Check if it's time to switch display pages (every 3 seconds)
  if (millis() - lastDisplayUpdate >= 3000)
  {
    displayPage = (displayPage + 1) % 2; // Toggle between 0 and 1
    lastDisplayUpdate = millis();
  }

  // Display 1: Target angle range and next measurement time (2 pages)
  display1.clearDisplay();
  display1.setTextSize(1);

  if (displayPage == 0)
  {
    // Page 0: Target angle range
    display1.setCursor(0, 0);
    display1.println("TARGET ANGLE");
    display1.setCursor(0, 16);
    display1.setTextSize(2); // Larger text for the values
    display1.printf("%.0f-%.0f°", config.targetAngleMin, config.targetAngleMax);
  }
  else
  {
    // Page 1: Next measurement time
    display1.setCursor(0, 0);
    if (nextMeasurementTime.unixtime() > 0)
    {
      display1.println("NEXT MEASUREMENT");
      display1.setCursor(0, 16);
      display1.printf("%02d:%02d %02d/%02d/%02d",
                      nextMeasurementTime.hour(),
                      nextMeasurementTime.minute(),
                      nextMeasurementTime.day(),
                      nextMeasurementTime.month(),
                      nextMeasurementTime.year() % 100);
    }
    else
    {
      display1.println("NO SCHEDULED");
      display1.setCursor(0, 16);
      display1.println("MEASUREMENT");
    }
  }
  display1.display();

  // Display 2: Last measurement angle and current time (2 pages)
  display2.clearDisplay();
  display2.setTextSize(1);

  if (displayPage == 0)
  {
    // Page 0: Last measurement angle
    display2.setCursor(0, 0);
    display2.println("LAST MEASUREMENT");
    display2.setCursor(0, 16);
    display2.setTextSize(2); // Larger text for the value
    if (config.lastMeasurementAngle > 0)
    {
      display2.printf("%.1f°", config.lastMeasurementAngle);
    }
    else
    {
      display2.print("--°");
    }
  }
  else
  {
    // Page 1: Current time and status
    display2.setCursor(0, 0);
    display2.println("CURRENT TIME");
    display2.setCursor(0, 12);
    display2.printf("%02d:%02d:%02d %02d/%02d",
                    now.hour(),
                    now.minute(),
                    now.second(),
                    now.day(),
                    now.month());

    // Show measurement status on second line
    display2.setCursor(0, 20);
    if (isMeasuring)
    {
      switch (measurementState)
      {
      case EMPTYING_INITIAL:
        display2.print("PREPARING");
        break;
      case FILLING:
        display2.print("FILLING");
        break;
      case WAITING_TO_SETTLE:
        display2.print("SETTLING");
        break;
      case MEASURING:
        display2.printf("MEAS %d/%d", measurementCount, config.measurementDuration);
        break;
      case EMPTYING_FINAL:
        display2.print("EMPTYING");
        break;
      default:
        display2.print("MEASURING");
      }
    }
    else
    {
      display2.print("READY");
    }
  }
  display2.display();
}

// Updated saveMeasurementData function to include angle
void saveMeasurementData(float density, float angle, DateTime timestamp)
{
  String filename = "/data_" + String(timestamp.year()) +
                    String(timestamp.month()) +
                    String(timestamp.day()) + ".csv";

  File file = LittleFS.open(filename, "a");
  if (file)
  {
    file.printf("%04d-%02d-%02d %02d:%02d:%02d,%.4f,%.2f\n",
                timestamp.year(), timestamp.month(), timestamp.day(),
                timestamp.hour(), timestamp.minute(), timestamp.second(),
                density, angle);
    file.close();
    logSerial("Measurement data saved to " + filename);
  }
  else
  {
    logSerial("Failed to save measurement data to " + filename);
  }
}

String getMeasurementData()
{
  String data = "Timestamp,Density,Angle\n";

  File root = LittleFS.open("/");
  File file = root.openNextFile();

  while (file)
  {
    String filename = file.name();
    if (filename.startsWith("/data_") && filename.endsWith(".csv"))
    {
      File dataFile = LittleFS.open(filename, "r");
      if (dataFile)
      {
        while (dataFile.available())
        {
          data += dataFile.readString();
        }
        dataFile.close();
      }
    }
    file = root.openNextFile();
  }

  return data;
}

void deleteMeasurementData()
{
  File root = LittleFS.open("/");
  File file = root.openNextFile();

  while (file)
  {
    String filename = file.name();
    if (filename.startsWith("/data_") && filename.endsWith(".csv"))
    {
      LittleFS.remove(filename);
      logSerial("Deleted data file: " + filename);
    }
    file = root.openNextFile();
  }
}

float angleToDensity(float angle)
{
  // Convert angle to density using calibration
  // This function converts the MPU6050 tilt angle to Claybath density
  // You'll need to calibrate this based on your specific probe setup

  // Apply calibration offset and scale
  float calibratedAngle = (angle + config.calibrationOffset) * config.calibrationScale;

  // Example conversion formula - customize based on your probe design
  // This assumes a linear relationship, but you might need a more complex formula
  float density = 1.000 + (calibratedAngle / 45.0) * 0.050; // 45° = 0.05 density units

  // Ensure reasonable bounds
  if (density < 0.900)
    density = 0.900;
  if (density > 1.200)
    density = 1.200;

  return density;
}

void scanI2CDevices()
{
  logSerial("Scanning I2C Bus 1 (GPIO21/22) - MPU6050 & OLED1...");
  int deviceCount1 = 0;

  for (byte address = 1; address < 127; address++)
  {
    I2C_1.beginTransmission(address);
    byte error = I2C_1.endTransmission();

    if (error == 0)
    {
      String deviceInfo = "I2C device found on Bus 1 at address 0x";
      if (address < 16)
        deviceInfo += "0";
      deviceInfo += String(address, HEX);

      // Identify known devices
      if (address == 0x68)
      {
        deviceInfo += " (MPU6050)";
      }
      else if (address == 0x3C)
      {
        deviceInfo += " (OLED Display 1)";
      }
      logSerial(deviceInfo);
      deviceCount1++;
    }
  }

  logSerial("Scanning I2C Bus 2 (GPIO18/19) - DS3231 & OLED2...");
  int deviceCount2 = 0;

  for (byte address = 1; address < 127; address++)
  {
    I2C_2.beginTransmission(address);
    byte error = I2C_2.endTransmission();

    if (error == 0)
    {
      String deviceInfo = "I2C device found on Bus 2 at address 0x";
      if (address < 16)
        deviceInfo += "0";
      deviceInfo += String(address, HEX);

      // Identify known devices
      if (address == 0x68)
      {
        deviceInfo += " (DS3231)";
      }
      else if (address == 0x3C)
      {
        deviceInfo += " (OLED Display 2)";
      }
      logSerial(deviceInfo);
      deviceCount2++;
    }
  }

  logSerial("Total devices found: Bus 1: " + String(deviceCount1) +
            ", Bus 2: " + String(deviceCount2));
}

bool checkRTCConnection()
{
  I2C_2.beginTransmission(DS3231_ADDRESS);
  byte error = I2C_2.endTransmission();
  return (error == 0);
}

bool checkMPUConnection()
{
  I2C_1.beginTransmission(MPU6050_ADDRESS);
  byte error = I2C_1.endTransmission();
  return (error == 0);
}

void setDateTime(int year, int month, int day, int hour, int minute, int second)
{
  rtc.adjust(DateTime(year, month, day, hour, minute, second));
  logSerial("RTC date/time manually set");
}