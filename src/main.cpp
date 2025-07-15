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
#include <time.h>

// Pin definitions
#define FILL_SOLENOID_PIN 25
#define EMPTY_SOLENOID_PIN 26
#define MEASURING_RELAY_PIN 27  // Single relay for both measuring lights (NO=Green, NC=Red)
#define SDA_PIN 21   // I2C Bus 1 for MPU6050 and OLED1
#define SCL_PIN 22   // I2C Bus 1 for MPU6050 and OLED1
#define SDA2_PIN 18  // I2C Bus 2 for DS3231 and OLED2
#define SCL2_PIN 19  // I2C Bus 2 for DS3231 and OLED2

// I2C Addresses
#define MPU6050_ADDRESS 0x68
#define DS3231_ADDRESS 0x68  // Same as MPU6050 - handled by different libraries
#define OLED_ADDRESS 0x3C    // Both OLEDs use same address but different I2C buses

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define OLED1_ADDRESS 0x3C
#define OLED2_ADDRESS 0x3D

// Global objects
Adafruit_MPU6050 mpu;
TwoWire I2C_1 = TwoWire(0);  // I2C Bus 1 for MPU6050 and OLED1
TwoWire I2C_2 = TwoWire(1);  // I2C Bus 2 for DS3231 and OLED2
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_1, OLED_RESET);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_2, OLED_RESET);
RTC_DS3231 rtc;
WebServer server(80);

// Configuration structure
struct Config {
  float desiredDensity = 1.025;
  int measurementInterval = 2; // hours
  int fillDuration = 5; // seconds
  int waitDuration = 60; // seconds
  int measurementDuration = 10; // seconds
  int emptyDuration = 120; // seconds
  float calibrationOffset = 0.0;
  float calibrationScale = 1.0;
  String timezone = "UTC";
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

// Function prototypes
void initializeSystem();
void loadConfig();
void saveConfig();
void setupWiFiHotspot();
void setupWebServer();
void handleWebRequests();
void performMeasurement();
void controlRelays();
void updateDisplays();
void saveMeasurementData(float density, DateTime timestamp);
String getMeasurementData();
void deleteMeasurementData();
float angleToDensity(float angle);
void calibrateMPU();
void setDateTime(int year, int month, int day, int hour, int minute, int second);
void scanI2CDevices();
bool checkRTCConnection();
bool checkMPUConnection();

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(FILL_SOLENOID_PIN, OUTPUT);
  pinMode(EMPTY_SOLENOID_PIN, OUTPUT);
  pinMode(MEASURING_RELAY_PIN, OUTPUT);
  
  // Ensure solenoids are closed and measuring light shows red (relay OFF = NC = Red light)
  digitalWrite(FILL_SOLENOID_PIN, HIGH);
  digitalWrite(EMPTY_SOLENOID_PIN, HIGH);
  digitalWrite(MEASURING_RELAY_PIN, HIGH);  // LOW = Red light (NC), HIGH = Green light (NO)
  
  // Initialize I2C buses with custom pins
  I2C_1.begin(SDA_PIN, SCL_PIN);     // Primary I2C bus
  I2C_2.begin(SDA2_PIN, SCL2_PIN);   // Secondary I2C bus
  I2C_1.setClock(100000); // Set I2C clock to 100kHz for stability
  I2C_2.setClock(100000); // Set I2C clock to 100kHz for stability
  
  // Scan I2C devices for debugging
  scanI2CDevices();
  
  // Initialize system
  initializeSystem();
  
  // Setup WiFi hotspot
  setupWiFiHotspot();
  
  // Setup web server
  setupWebServer();
  
  Serial.println("Claybath density measurement system initialized");
}

void loop() {
  // Handle web server requests
  server.handleClient();
  
  // Check if it's time for automatic measurement
  if (!isManualMode && millis() - lastMeasurementMillis >= (config.measurementInterval * 3600000)) {
    performMeasurement();
    lastMeasurementMillis = millis();
  }
  
  // Update displays
  updateDisplays();
  
  // Control pilot lamps
  controlRelays();
  
  delay(1000);
}

void initializeSystem() {
  // Initialize LittleFS
  if (!LittleFS.begin()) {
    Serial.println("LittleFS initialization failed!");
    return;
  }
  
  // Load configuration
  loadConfig();
  
  // Initialize DS3231 RTC on I2C Bus 2
  if (!rtc.begin(&I2C_2)) {
    Serial.println("Couldn't find DS3231 RTC on I2C Bus 2");
    // Continue without RTC but log error
  } else {
    Serial.println("DS3231 RTC initialized successfully on I2C Bus 2");
    
    // Check if RTC lost power and if so, set the time
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, setting time to compile time");
      // Set to compile time as fallback
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    
    // Print current time
    DateTime now = rtc.now();
    Serial.print("Current RTC time: ");
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.println(now.second(), DEC);
  }
  
  // Initialize MPU6050 on I2C Bus 1
  if (!mpu.begin(MPU6050_ADDRESS, &I2C_1)) {
    Serial.println("Failed to find MPU6050 chip on I2C Bus 1");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 initialized successfully on I2C Bus 1");
  
  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Allow sensor to stabilize
  delay(100);
  
  // Initialize displays on separate I2C buses
  if (!display1.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed for display 1"));
  } else {
    Serial.println("OLED Display 1 initialized successfully on I2C_1");
  }
  
  if (!display2.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed for display 2"));
  } else {
    Serial.println("OLED Display 2 initialized successfully on I2C_2");
  }
  
  display1.clearDisplay();
  display2.clearDisplay();
  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);
  display2.setTextSize(1);
  display2.setTextColor(SSD1306_WHITE);
  
  // Show initialization message
  display1.setCursor(0, 0);
  display1.println("Claybath Density");
  display1.println("Measurement");
  display1.println("System");
  display1.println("Initializing...");
  display1.display();
  
  display2.setCursor(0, 0);
  display2.println("DS3231 RTC");
  display2.println("MPU6050 Sensor");
  display2.println("WiFi Hotspot");
  display2.println("Starting...");
  display2.display();
  
  delay(2000);
  
  // Calculate next measurement time
  DateTime now = rtc.now();
  nextMeasurementTime = DateTime(now.unixtime() + (config.measurementInterval * 3600));
  
  Serial.println("System initialization complete");
}

void loadConfig() {
  if (LittleFS.exists("/settings.json")) {
    File file = LittleFS.open("/settings.json", "r");
    if (file) {
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, file);
      
      config.desiredDensity = doc["desiredDensity"] | 1.025;
      config.measurementInterval = doc["measurementInterval"] | 2;
      config.fillDuration = doc["fillDuration"] | 5;
      config.waitDuration = doc["waitDuration"] | 60;
      config.measurementDuration = doc["measurementDuration"] | 10;
      config.emptyDuration = doc["emptyDuration"] | 120;
      config.calibrationOffset = doc["calibrationOffset"] | 0.0;
      config.calibrationScale = doc["calibrationScale"] | 1.0;
      
      // Handle String assignment properly
      const char* timezoneStr = doc["timezone"] | "UTC";
      config.timezone = String(timezoneStr);
      
      file.close();
    }
  }
}

void saveConfig() {
  DynamicJsonDocument doc(1024);
  doc["desiredDensity"] = config.desiredDensity;
  doc["measurementInterval"] = config.measurementInterval;
  doc["fillDuration"] = config.fillDuration;
  doc["waitDuration"] = config.waitDuration;
  doc["measurementDuration"] = config.measurementDuration;
  doc["emptyDuration"] = config.emptyDuration;
  doc["calibrationOffset"] = config.calibrationOffset;
  doc["calibrationScale"] = config.calibrationScale;
  doc["timezone"] = config.timezone;
  
  File file = LittleFS.open("/settings.json", "w");
  if (file) {
    serializeJson(doc, file);
    file.close();
  }
}

void setupWiFiHotspot() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ClaybathDensityMeter", "12345678");
  
  Serial.println("WiFi Hotspot started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
}

void setupWebServer() {
  // Serve static files from LittleFS
  server.serveStatic("/", LittleFS, "/index.html");
  
  // API endpoints
  server.on("/api/status", HTTP_GET, []() {
    DynamicJsonDocument doc(1024);
    doc["currentAngle"] = currentAngle;
    doc["currentDensity"] = currentDensity;
    doc["lastMeasurement"] = lastMeasurement;
    doc["lastMeasurementTime"] = lastMeasurementTime.unixtime();
    doc["nextMeasurementTime"] = nextMeasurementTime.unixtime();
    doc["isMeasuring"] = isMeasuring;
    doc["isManualMode"] = isManualMode;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  server.on("/api/config", HTTP_GET, []() {
    DynamicJsonDocument doc(1024);
    doc["desiredDensity"] = config.desiredDensity;
    doc["measurementInterval"] = config.measurementInterval;
    doc["fillDuration"] = config.fillDuration;
    doc["waitDuration"] = config.waitDuration;
    doc["measurementDuration"] = config.measurementDuration;
    doc["emptyDuration"] = config.emptyDuration;
    doc["calibrationOffset"] = config.calibrationOffset;
    doc["calibrationScale"] = config.calibrationScale;
    doc["timezone"] = config.timezone;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
  });
  
  server.on("/api/config", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, body);
      
      config.desiredDensity = doc["desiredDensity"];
      config.measurementInterval = doc["measurementInterval"];
      config.fillDuration = doc["fillDuration"];
      config.waitDuration = doc["waitDuration"];
      config.measurementDuration = doc["measurementDuration"];
      config.emptyDuration = doc["emptyDuration"];
      config.calibrationOffset = doc["calibrationOffset"];
      config.calibrationScale = doc["calibrationScale"];
      
      // Handle String assignment properly
      if (doc.containsKey("timezone")) {
        const char* timezoneStr = doc["timezone"];
        config.timezone = String(timezoneStr);
      }
      
      saveConfig();
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"no_data\"}");
    }
  });
  
  server.on("/api/measure", HTTP_POST, []() {
    if (!isMeasuring) {
      performMeasurement();
      server.send(200, "application/json", "{\"status\":\"measurement_started\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"measurement_in_progress\"}");
    }
  });
  
  server.on("/api/control", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      DynamicJsonDocument doc(512);
      deserializeJson(doc, body);
      
      String action = doc["action"];
      bool state = doc["state"];
      
      if (action == "fill_solenoid") {
        digitalWrite(FILL_SOLENOID_PIN, state ? LOW : HIGH);
      } else if (action == "empty_solenoid") {
        digitalWrite(EMPTY_SOLENOID_PIN, state ? LOW : HIGH);
      } else if (action == "measuring_relay") {
        digitalWrite(MEASURING_RELAY_PIN, state ? LOW : HIGH);
      }
      
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"no_data\"}");
    }
  });
  
  server.on("/api/datetime", HTTP_POST, []() {
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
      Serial.print("RTC time set to: ");
      Serial.print(now.year(), DEC);
      Serial.print('/');
      Serial.print(now.month(), DEC);
      Serial.print('/');
      Serial.print(now.day(), DEC);
      Serial.print(" ");
      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.println(now.second(), DEC);
      
      server.send(200, "application/json", "{\"status\":\"success\"}");
    } else {
      server.send(400, "application/json", "{\"error\":\"no_data\"}");
    }
  });
  
  server.on("/api/data", HTTP_GET, []() {
    String data = getMeasurementData();
    server.send(200, "text/plain", data);
  });
  
  server.on("/api/data", HTTP_DELETE, []() {
    deleteMeasurementData();
    server.send(200, "application/json", "{\"status\":\"success\"}");
  });
  
  // Serve the main page
  server.on("/", []() {
    if (LittleFS.exists("/index.html")) {
      File file = LittleFS.open("/index.html", "r");
      server.streamFile(file, "text/html");
      file.close();
    } else {
      server.send(404, "text/plain", "index.html not found");
    }
  });
  
  // Handle 404
  server.onNotFound([]() {
    server.send(404, "text/plain", "Not Found");
  });
  
  server.begin();
  Serial.println("Web server started");
}

void performMeasurement() {
  isMeasuring = true;
  
  // Step 1: Ensure empty solenoid is closed
  digitalWrite(EMPTY_SOLENOID_PIN, HIGH);
  delay(1000);
  
  // Step 2: Fill chamber
  digitalWrite(FILL_SOLENOID_PIN, LOW);
  delay(config.fillDuration * 1000);
  digitalWrite(FILL_SOLENOID_PIN, HIGH);
  
  // Step 3: Wait for settling
  delay(config.waitDuration * 1000);
  
  // Step 4: Measure angle
  float angleSum = 0.0;
  int validReadings = 0;
  
  for (int i = 0; i < config.measurementDuration; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Calculate angle from accelerometer
    float angle = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    
    // Validate reading
    if (abs(angle) < 90) { // Reasonable angle range
      angleSum += angle;
      validReadings++;
    }
    
    delay(1000);
  }
  
  if (validReadings > 0) {
    currentAngle = (angleSum / validReadings) + config.calibrationOffset;
    currentDensity = angleToDensity(currentAngle * config.calibrationScale);
    lastMeasurement = currentDensity;
    lastMeasurementTime = rtc.now();
    
    // Log measurement details
    Serial.print("Measurement completed - Angle: ");
    Serial.print(currentAngle);
    Serial.print("°, Density: ");
    Serial.print(currentDensity, 4);
    Serial.print(", Valid readings: ");
    Serial.print(validReadings);
    Serial.print("/");
    Serial.println(config.measurementDuration);
    
    // Save measurement data
    saveMeasurementData(currentDensity, lastMeasurementTime);
  } else {
    Serial.println("No valid readings obtained during measurement");
  }
  
  // Step 5: Empty chamber
  digitalWrite(EMPTY_SOLENOID_PIN, LOW);
  delay(config.emptyDuration * 1000);
  digitalWrite(EMPTY_SOLENOID_PIN, HIGH);
  
  // Calculate next measurement time
  DateTime now = rtc.now();
  nextMeasurementTime = DateTime(now.unixtime() + (config.measurementInterval * 3600));
  
  isMeasuring = false;
}

void controlRelays() {
  // Control measuring pilot lamp using single relay
  // LOW = Red light (NC), HIGH = Green light (NO)
  if (isMeasuring) {
    digitalWrite(MEASURING_RELAY_PIN, HIGH);  // Green light (NO)
  } else {
    digitalWrite(MEASURING_RELAY_PIN, HIGH);   // Red light (NC)
  }
}

void updateDisplays() {
  DateTime now = rtc.now();
  
  // Display 1: Desired density and next measurement time
  display1.clearDisplay();
  display1.setTextSize(1);
  display1.setCursor(0, 0);
  display1.println("DESIRED DENSITY");
  display1.setTextSize(2);
  display1.setCursor(0, 12);
  display1.printf("%.3f", config.desiredDensity);
  display1.setTextSize(1);
  display1.setCursor(0, 35);
  display1.println("NEXT MEASUREMENT");
  display1.setCursor(0, 45);
  display1.printf("%02d:%02d %02d/%02d/%02d", 
    nextMeasurementTime.hour(), 
    nextMeasurementTime.minute(),
    nextMeasurementTime.day(),
    nextMeasurementTime.month(),
    nextMeasurementTime.year() % 100);
  display1.display();
  
  // Display 2: Last measurement and current time
  display2.clearDisplay();
  display2.setTextSize(1);
  display2.setCursor(0, 0);
  display2.println("LAST MEASUREMENT");
  display2.setTextSize(2);
  display2.setCursor(0, 12);
  if (lastMeasurement > 0) {
    display2.printf("%.3f", lastMeasurement);
  } else {
    display2.print("--");
  }
  display2.setTextSize(1);
  display2.setCursor(0, 35);
  display2.println("CURRENT TIME");
  display2.setCursor(0, 45);
  display2.printf("%02d:%02d:%02d %02d/%02d", 
    now.hour(), 
    now.minute(),
    now.second(),
    now.day(),
    now.month());
  
  // Show measurement status
  display2.setCursor(0, 55);
  if (isMeasuring) {
    display2.print("MEASURING...");
  } else {
    display2.print("READY");
  }
  
  display2.display();
}

void saveMeasurementData(float density, DateTime timestamp) {
  String filename = "/data_" + String(timestamp.year()) + 
                   String(timestamp.month()) + 
                   String(timestamp.day()) + ".csv";
  
  File file = LittleFS.open(filename, "a");
  if (file) {
    file.printf("%04d-%02d-%02d %02d:%02d:%02d,%.4f,%.2f\n",
      timestamp.year(), timestamp.month(), timestamp.day(),
      timestamp.hour(), timestamp.minute(), timestamp.second(),
      density, currentAngle);
    file.close();
  }
}

String getMeasurementData() {
  String data = "Timestamp,Density,Angle\n";
  
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  
  while (file) {
    String filename = file.name();
    if (filename.startsWith("/data_") && filename.endsWith(".csv")) {
      File dataFile = LittleFS.open(filename, "r");
      if (dataFile) {
        while (dataFile.available()) {
          data += dataFile.readString();
        }
        dataFile.close();
      }
    }
    file = root.openNextFile();
  }
  
  return data;
}

void deleteMeasurementData() {
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  
  while (file) {
    String filename = file.name();
    if (filename.startsWith("/data_") && filename.endsWith(".csv")) {
      LittleFS.remove(filename);
    }
    file = root.openNextFile();
  }
}

float angleToDensity(float angle) {
  // Convert angle to density using calibration
  // This function converts the MPU6050 tilt angle to Claybath density
  // You'll need to calibrate this based on your specific probe setup
  
  // Apply calibration offset and scale
  float calibratedAngle = (angle + config.calibrationOffset) * config.calibrationScale;
  
  // Example conversion formula - customize based on your probe design
  // This assumes a linear relationship, but you might need a more complex formula
  float density = 1.000 + (calibratedAngle / 45.0) * 0.050; // 45° = 0.05 density units
  
  // Ensure reasonable bounds
  if (density < 0.900) density = 0.900;
  if (density > 1.200) density = 1.200;
  
  return density;
}

void scanI2CDevices() {
  Serial.println("Scanning I2C Bus 1 (GPIO21/22) - MPU6050 & OLED1...");
  int deviceCount1 = 0;
  
  for (byte address = 1; address < 127; address++) {
    I2C_1.beginTransmission(address);
    byte error = I2C_1.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found on Bus 1 at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify known devices
      if (address == 0x68) {
        Serial.print(" (MPU6050)");
      } else if (address == 0x3C) {
        Serial.print(" (OLED Display 1)");
      }
      Serial.println();
      deviceCount1++;
    }
  }
  
  Serial.println("Scanning I2C Bus 2 (GPIO18/19) - DS3231 & OLED2...");
  int deviceCount2 = 0;
  
  for (byte address = 1; address < 127; address++) {
    I2C_2.beginTransmission(address);
    byte error = I2C_2.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found on Bus 2 at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify known devices
      if (address == 0x68) {
        Serial.print(" (DS3231)");
      } else if (address == 0x3C) {
        Serial.print(" (OLED Display 2)");
      }
      Serial.println();
      deviceCount2++;
    }
  }
  
  Serial.print("Total devices found: Bus 1: ");
  Serial.print(deviceCount1);
  Serial.print(", Bus 2: ");
  Serial.println(deviceCount2);
}

bool checkRTCConnection() {
  I2C_2.beginTransmission(DS3231_ADDRESS);
  byte error = I2C_2.endTransmission();
  return (error == 0);
}

bool checkMPUConnection() {
  I2C_1.beginTransmission(MPU6050_ADDRESS);
  byte error = I2C_1.endTransmission();
  return (error == 0);
}

void setDateTime(int year, int month, int day, int hour, int minute, int second) {
  rtc.adjust(DateTime(year, month, day, hour, minute, second));
}