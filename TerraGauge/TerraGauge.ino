#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h> 
#include <math.h> 

// --- NEW LIBRARIES FOR WEB SERVER / HOTSPOT ---
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h> 

// --- AP (Access Point) Configuration ---
// *** IMPORTANT: CHANGE THESE CREDENTIALS ***
const char* AP_SSID = "GPSHacker";     
const char* AP_PASSWORD = "password"; 

// Create WebServer object on port 80
ESP8266WebServer server(80); 
// ---------------------------------------------

// --- OLED Display Configuration ---
#define SCREEN_WIDTH 128    
#define SCREEN_HEIGHT 64    

#define OLED_ADDR   0x3C    
#define OLED_RESET  -1      

// Initialize the display objects (OLED and MPU)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu; 

// --- Conversion Constants ---
const float KM_TO_MILES = 0.621371;
const float METER_TO_FEET = 3.28084;
const float MILES_TO_FEET = 5280.0; 
const float M_TO_MILES = 0.000621371; // Meters to miles
const float R_EARTH_KM = 6371.0; 

// --- Mock Data Variables ---
const char* MOCK_DATE = "11/21/25"; 
const char* MOCK_TIME = "12:09:45"; 
const float MOCK_CURRENT_LAT = 37.425123; // Current GPS position (Mock)
const float MOCK_CURRENT_LNG = -122.086456;
int MOCK_SAT_COUNT = 0; 
const float MOCK_ALTITUDE_M = 30.5; 
const float MOCK_SPEED_KPH = 0.5; 

float currentPitch = 0.0; 

const float MOCK_HEADING = 90.0; 

// --- HOME Location (Reference Point for Distance Calculation) ---
// Variables are NOT const so they can be updated by the web server
float HOME_LAT = 37.421999;
float HOME_LNG = -122.084058;

// --- Global State Variables ---
unsigned long lastLockStateChangeTime = 0;
const unsigned long lockStateChangeInterval = 15000; 

// MPU offsets and status
float gyroXoffset = 0.0;
float gyroYoffset = 0.0;
float gyroZoffset = 0.0;

unsigned long lastMpuUpdate = 0;
const float alpha = 0.98; 
bool mpuIsPresent = false; 

// --- FUNCTION PROTOTYPES ---
void initializeMPU();
void performGyroCalibration();
void updateAttitude();
float calculateDistance(float lat1, float lon1, float lat2, float lon2);
float calculateBearingToHome(float lat1, float lon1, float lat2, float lon2);
void showBootSplash();
void drawClosedLock(int cx, int cy);
void drawDirectionShape(int cx, int cy, int size, float angle, bool isNorth); 

// --- WEB HANDLER PROTOTYPES ---
void handleRoot();
void handleData();
void handleSetHome();
void handleSetCurrentAsHome(); 
void handleNotFound();
void displayHomeUpdatedOLED(); 
// --- END PROTOTYPES ---


// ===============================================
// CORE NAVIGATION & SENSOR FUNCTIONS
// ===============================================

/**
 * @brief Calculates the distance between two GPS coordinates using the Haversine formula.
 * @return Distance in Kilometers.
 */
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  float dLat = (lat2 - lat1) * (M_PI / 180.0);
  float dLon = (lon2 - lon1) * (M_PI / 180.0);

  lat1 = lat1 * (M_PI / 180.0);
  lat2 = lat2 * (M_PI / 180.0);

  float a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  float distanceKm = R_EARTH_KM * c; 
  return distanceKm;
}

float calculateBearingToHome(float lat1, float lon1, float lat2, float lon2) {
  lat1 = lat1 * (M_PI / 180.0);
  lon1 = lon1 * (M_PI / 180.0);
  lat2 = lat2 * (M_PI / 180.0);
  lon2 = lon2 * (M_PI / 180.0);
  
  float dLon = lon2 - lon1;

  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  float bearingRad = atan2(y, x);
  
  float bearingDeg = bearingRad * (180.0 / M_PI);
  return fmod(bearingDeg + 360.0, 360.0);
}

void drawClosedLock(int cx, int cy) {
    int bodyW = 20;
    int bodyH = 15;
    display.fillRect(cx - bodyW/2, cy, bodyW, bodyH, SSD1306_WHITE);

    int shackleH = 10;
    int shackleW = 10;
    int shackleY = cy - shackleH;
    
    display.drawFastVLine(cx - shackleW/2, shackleY, shackleH, SSD1306_WHITE);
    display.drawFastVLine(cx + shackleW/2, shackleY, shackleH, SSD1306_WHITE);
    display.drawFastHLine(cx - shackleW/2, shackleY, shackleW + 1, SSD1306_WHITE); 
}

void showBootSplash() {
  MOCK_SAT_COUNT = 0; 
    
  const char* welcomeMessage = "AQUIRING GPS LOCK..."; 
  const int msgLength = strlen(welcomeMessage);
  const int charWidth = 6; 
  const int scrollDurationMs = 2000; 
  
  display.setTextSize(1); 
  display.setTextColor(SSD1306_WHITE);
  const int textY = (SCREEN_HEIGHT / 2) - 4; 

  long startTime = millis();
  long lastSatUpdate = millis();
  const int satUpdateInterval = 500; 
  
  while (MOCK_SAT_COUNT <= 3) {
    display.clearDisplay();
    
    if (millis() - lastSatUpdate > satUpdateInterval) { 
        if (MOCK_SAT_COUNT < 4) { 
            MOCK_SAT_COUNT++;
        }
        lastSatUpdate = millis();
    }
    
    if (millis() - startTime > scrollDurationMs) {
        startTime = millis();
    }
    
    float progress = (float)(millis() - startTime) / scrollDurationMs;
    int scrollDistance = 128 + (msgLength * charWidth); 
    int currentScrollX = (int)(progress * scrollDistance); 
    
    int startX = 128 - currentScrollX;
    
    display.setCursor(startX, textY);
    display.print(welcomeMessage);
    
    display.drawRect(10, 50, 108, 10, SSD1306_WHITE);
    
    int maxSatForBar = 4; 
    int fillWidth = map(MOCK_SAT_COUNT, 0, maxSatForBar, 0, 106); 
    if (fillWidth > 106) fillWidth = 106; 
    
    display.fillRect(11, 51, fillWidth, 8, SSD1306_WHITE); 
    
    display.setTextSize(1);
    display.setCursor(11, 52); 
    display.print("GPS LOCK: ");
    display.print(MOCK_SAT_COUNT);
    display.print(" of 4");
    
    display.display();
    delay(20); 
  }
  
  display.clearDisplay();
  drawClosedLock(SCREEN_WIDTH / 2, 20); 
  display.setTextSize(1);
  display.setCursor(1, 45); 
  display.print("3D GPS LOCK ACQUIRED!");

  display.display();
  delay(1500); 
  
  MOCK_SAT_COUNT = 4; 
}

void performGyroCalibration() {
    Serial.print(F("Calibrating Gyro... DO NOT MOVE MPU!"));
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("CALIBRATING GYRO...");
    display.setCursor(0, 10);
    display.print("DO NOT MOVE!");
    display.display();
    
    long sumX = 0, sumY = 0, sumZ = 0;
    const int numSamples = 2000;
    
    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        sumX += g.gyro.x * 1000.0; 
        sumY += g.gyro.y * 1000.0;
        sumZ += g.gyro.z * 1000.0;
        
        delay(1); 
        
        if (i % (numSamples / 10) == 0) {
            display.drawFastVLine(0 + (i / (numSamples / 10)) * 10, 20, 10, SSD1306_WHITE);
            display.display();
        }
    }
    
    gyroXoffset = (float)sumX / numSamples / 1000.0;
    gyroYoffset = (float)sumY / numSamples / 1000.0;
    gyroZoffset = (float)sumZ / numSamples / 1000.0;
    
    Serial.println(F("Done."));

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("CALIBRATION COMPLETE!");
    display.display();
    delay(1000); 
}

void initializeMPU() {
    Serial.print(F("Initializing MPU... "));

    if (!mpu.begin()) {
        Serial.println(F("Failed to find MPU6050 chip. Proceeding without MPU data."));
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.print("MPU NOT FOUND!"); 
        display.display();
        delay(1500);
        mpuIsPresent = false; 
        return; 
    }
    
    Serial.println(F("Done."));
    mpuIsPresent = true; 

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    performGyroCalibration(); 
}

void updateAttitude() {
    if (!mpuIsPresent) {
        return;
    }
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = millis();
    float dt = (now - lastMpuUpdate) / 1000.0; 
    lastMpuUpdate = now;

    float correctedGyroY = g.gyro.y - gyroYoffset;
    float accPitch = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / M_PI;
    float gyroChange = correctedGyroY * dt;

    currentPitch = alpha * (currentPitch + gyroChange) + (1.0 - alpha) * accPitch;
    
    currentPitch = constrain(currentPitch, -90.0, 90.0);
}

/**
 * @brief Draws a simple shape (triangle or 'N') rotated by an angle.
 * @param cx Center X position.
 * @param cy Center Y position.
 * @param size Length from center to tip (for triangle) or height (for 'N').
 * @param angle Angle in degrees (0 = straight up/North).
 * @param isNorth If true, draws a rotating 'N'; otherwise, draws a triangle.
 */
void drawDirectionShape(int cx, int cy, int size, float angle, bool isNorth) {
  float radians = (90.0 - angle) * (M_PI / 180.0);
  
  int points[4][2]; 
  int numPoints = 0;

  if (isNorth) {
    // Rotating 'N' using lines defined by 3 points
    int halfSize = size / 2;

    // Define the 3 points of the 'N' shape (relative to center)
    points[0][0] = -halfSize; points[0][1] = -halfSize; // Top Left (Start of N)
    points[1][0] = -halfSize; points[1][1] = halfSize; // Bottom Left 
    points[2][0] = halfSize; points[2][1] = -halfSize; // Top Right (Diagonal end point) 
    
    numPoints = 3;

    // Perform rotation and translation
    for (int i = 0; i < numPoints; i++) {
        int px = points[i][0];
        int py = points[i][1];
        
        points[i][0] = (int)(px * cos(radians) - py * sin(radians) + cx);
        points[i][1] = (int)(px * sin(radians) + py * cos(radians) + cy); 
    }
    
    // Draw the rotated shape (Lines for 'N')
    // Line 1 (Left Stroke)
    display.drawLine(points[0][0], points[0][1], points[1][0], points[1][1], SSD1306_WHITE); 
    // Line 2 (Diagonal Stroke) - from top left to bottom right (calculated point)
    int p3x = (int)(halfSize * cos(radians) - (-halfSize) * sin(radians) + cx);
    int p3y = (int)(halfSize * sin(radians) + (-halfSize) * cos(radians) + cy);
    display.drawLine(points[0][0], points[0][1], p3x, p3y, SSD1306_WHITE); 
    // Line 3 (Right Stroke)
    display.drawLine(p3x, p3y, points[1][0], points[1][1], SSD1306_WHITE); // Simplified connection to bottom left for a closed look


  } else {
    // Triangle (Home Arrow)
    points[0][0] = 0;          points[0][1] = -size;  // Tip
    points[1][0] = -size / 2;  points[1][1] = size / 2; // Bottom left
    points[2][0] = size / 2;   points[2][1] = size / 2; // Bottom right
    
    numPoints = 3;

    // Perform rotation and translation
    for (int i = 0; i < numPoints; i++) {
        int px = points[i][0];
        int py = points[i][1];
        
        points[i][0] = (int)(px * cos(radians) - py * sin(radians) + cx);
        points[i][1] = (int)(px * sin(radians) + py * cos(radians) + cy); 

    }
    
    // Draw the rotated triangle
    display.fillTriangle(
      points[0][0], points[0][1],
      points[1][0], points[1][1],
      points[2][0], points[2][1],
      SSD1306_WHITE
    );
  }
}


// ===============================================
// WEB SERVER HANDLERS
// ===============================================

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP8266 Nav Data</title>
<style>
  body{font-family: sans-serif; text-align: center; margin-top: 20px; background-color: #f0f0f0;}
  .container {max-width: 400px; margin: auto; padding: 20px; background: white; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1);}
  .data-box{border: 1px solid #ccc; padding: 15px; margin: 15px 0; text-align: left; background-color: #e9e9e9; border-radius: 5px;}
  .set-form {margin-top: 20px; border: 1px solid #007bff; padding: 15px; background-color: #e0f7ff; border-radius: 5px;}
  input[type="text"], input[type="submit"], button {width: 100%; padding: 8px; margin: 5px 0; box-sizing: border-box; border-radius: 4px; border: 1px solid #ccc;}
  input[type="submit"], button {background-color: #007bff; color: white; cursor: pointer; border: none;}
  input[type="submit"]:hover, button:hover {background-color: #0056b3;}
  strong {font-weight: bold; color: #333;}
</style>
</head>
<body>
<div class="container">
<h1>ESP Nav Data Viewer</h1>
<p>Hotspot IP: <strong>192.168.4.1</strong></p>

<div class="data-box">
  <h2>Live Readings</h2>
  <p><strong>Speed:</strong> <span id="speed">N/A</span> mph</p>
  <p><strong>Altitude:</strong> <span id="alt">N/A</span> ft</p>
  <p><strong>Pitch (Attitude):</strong> <span id="pitch">N/A</span> &deg;</p>
  <p><strong>Time to Home (TTH):</strong> <span id="tth">N/A</span></p>
  <p><strong>Satellites:</strong> <span id="sats">N/A</span></p>
  <p><strong>Home Lat/Lng:</strong> <span id="home">N/A</span></p>
</div>

<div class="set-form">
  <h2>Set Home Location</h2>
  
  <p id="success-message" style="color: green; font-weight: bold; display:none;">Home Point Updated!</p>

  <p>Current Mock GPS: <br>Lat: <strong>)rawliteral" + String(MOCK_CURRENT_LAT, 6) + R"rawliteral(</strong>, Lng: <strong>)rawliteral" + String(MOCK_CURRENT_LNG, 6) + R"rawliteral(</strong></p>
  
  <button onclick="setCurrentAsHome()">Set Current Location to Home Point</button>
  
  <p style="margin-top: 20px; margin-bottom: 0;">Or set manually:</p>
  <form action="/sethome" method="get">
    <label for="lat">Latitude:</label>
    <input type="text" id="lat" name="lat" required placeholder="e.g., 37.425123"><br>
    <label for="lng">Longitude:</label>
    <input type="text" id="lng" name="lng" required placeholder="e.g., -122.086456"><br>
    <input type="submit" value="Set Custom Home Point">
  </form>
</div>

<script>
function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var data = JSON.parse(this.responseText);
      document.getElementById("speed").innerHTML = parseFloat(data.speedMph).toFixed(1);
      document.getElementById("alt").innerHTML = parseFloat(data.altitudeFt).toFixed(0);
      
      // Display TTH value
      document.getElementById("tth").innerHTML = data.timeToHome;

      document.getElementById("pitch").innerHTML = data.pitchDeg;
      document.getElementById("sats").innerHTML = data.satellites;
      document.getElementById("home").innerHTML = parseFloat(data.homeLat).toFixed(6) + ", " + parseFloat(data.homeLng).toFixed(6);
    }
  };
  xhttp.open("GET", "/data", true);
  xhttp.send();
}

// *** NEW JAVASCRIPT FUNCTION ***
function setCurrentAsHome() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      // Show success message on the web page
      var msg = document.getElementById("success-message");
      msg.style.display = "block";
      
      // Hide the message after 3 seconds
      setTimeout(function() {
        msg.style.display = "none";
      }, 3000); 
      
      // Force a data update to reflect the new home point immediately
      getData();
    }
  };
  xhttp.open("GET", "/setcurrentashome", true);
  xhttp.send();
}

getData();
setInterval(getData, 2000); 
</script>

</div>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleData() {
  float distanceKm = calculateDistance(MOCK_CURRENT_LAT, MOCK_CURRENT_LNG, HOME_LAT, HOME_LNG);
  float altitudeFeet = MOCK_ALTITUDE_M * METER_TO_FEET;
  float speedMph = MOCK_SPEED_KPH * KM_TO_MILES;
  
  float distanceMeters = distanceKm * 1000.0;
  float speedMetersPerSecond = MOCK_SPEED_KPH / 3.6; 
  
  String timeToHomeString;

  if (speedMetersPerSecond < 0.2 || distanceMeters < 5.0) { 
      timeToHomeString = "ARRIVED"; 
  } else {
      int tthTotalSeconds = round(distanceMeters / speedMetersPerSecond);
      
      int tthHours = tthTotalSeconds / 3600;
      int tthMinutes = (tthTotalSeconds % 3600) / 60;
      int tthRemainingSeconds = tthTotalSeconds % 60;
      
      if (tthHours > 0) {
          // Format as HH:MM:SS
          timeToHomeString = String(tthHours) + ":" + 
                             (tthMinutes < 10 ? "0" : "") + String(tthMinutes) + ":" + 
                             (tthRemainingSeconds < 10 ? "0" : "") + String(tthRemainingSeconds);
      } else {
          // Format as MM:SS (for times under 1 hour)
          timeToHomeString = (tthMinutes < 10 && tthTotalSeconds >= 60 ? "0" : "") + String(tthMinutes) + ":" + 
                             (tthRemainingSeconds < 10 ? "0" : "") + String(tthRemainingSeconds);
      }
  }

  // Use the integer pitch value (no decimals) as requested for simplicity on the OLED.
  String pitchString = (mpuIsPresent ? String((int)currentPitch) : "N/A");

  String json = "{";
  json += "\"speedMph\":\"" + String(speedMph, 1) + "\",";
  json += "\"altitudeFt\":\"" + String(altitudeFeet, 0) + "\",";
  // Send the calculated TTH string
  json += "\"timeToHome\":\"" + timeToHomeString + "\","; 
  
  json += "\"pitchDeg\":\"" + pitchString + "\","; 
  json += "\"satellites\":\"" + String(MOCK_SAT_COUNT) + "\",";
  json += "\"homeLat\":\"" + String(HOME_LAT, 6) + "\",";
  json += "\"homeLng\":\"" + String(HOME_LNG, 6) + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleSetHome() {
  if (server.hasArg("lat") && server.hasArg("lng")) {
    HOME_LAT = server.arg("lat").toFloat();
    HOME_LNG = server.arg("lng").toFloat();

    Serial.print("New HOME set manually to: ");
    Serial.print(HOME_LAT, 6);
    Serial.print(", ");
    Serial.println(HOME_LNG, 6);

    displayHomeUpdatedOLED(); 

    server.send(200, "application/json", "{\"status\":\"success\"}");

  } else {
    server.send(400, "text/plain", "Missing Latitude or Longitude parameters.");
  }
}

/**
 * @brief Sets the global HOME_LAT/HOME_LNG to the current MOCK GPS location.
 */
void handleSetCurrentAsHome() {
  HOME_LAT = MOCK_CURRENT_LAT;
  HOME_LNG = MOCK_CURRENT_LNG;

  Serial.print("Current GPS location set as HOME: ");
  Serial.print(HOME_LAT, 6);
  Serial.print(", ");
  Serial.println(HOME_LNG, 6);
  
  // Update OLED display with success message
  displayHomeUpdatedOLED();

  server.send(200, "application/json", "{\"status\":\"success\"}");
}


void handleNotFound(){
  server.send(404, "text/plain", "404: Not Found");
}

/**
 * @brief Displays a brief 'Home Point Updated' message on the OLED.
 */
void displayHomeUpdatedOLED() {
    display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_BLACK); 
    
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(5, 18);
    display.print("HOME POINT");
    display.setCursor(5, 38);
    display.print("UPDATED!");
    
    display.display();
    delay(1500); 
}


// ===============================================
// SETUP & LOOP
// ===============================================

void setup() {
  Serial.begin(115200);

  // Initialize I2C bus on GPIO14 (D5) and GPIO12 (D6)
  Wire.begin(14, 12); 

  // Initialize I2C Bus and Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed. Check I2C wiring and address (0x3C)."));
    for (;;); 
  }

  // --- Access Point Setup ---
  Serial.print("Setting up Access Point: ");
  Serial.println(AP_SSID);
  
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  IPAddress apIP = WiFi.softAPIP();
  Serial.print("AP IP Address: ");
  Serial.println(apIP);

  // --- Web Server Routing Setup ---
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);
  server.on("/sethome", HTTP_GET, handleSetHome);
  server.on("/setcurrentashome", HTTP_GET, handleSetCurrentAsHome); 
  server.onNotFound(handleNotFound);

  // Start the server
  server.begin();
  Serial.println("HTTP server started.");
  
  // --- STARTUP PROCESS ---
  showBootSplash();
  initializeMPU(); 

  MOCK_SAT_COUNT = 8; 
  lastLockStateChangeTime = millis(); 
  lastMpuUpdate = millis(); 

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
}


void loop() {
  // Handle incoming web server requests constantly
  server.handleClient();
  
  updateAttitude(); 
  
  // --- MOCK LOCK LOSS LOGIC ---
  if (millis() - lastLockStateChangeTime > lockStateChangeInterval) {
    if (MOCK_SAT_COUNT >= 4) {
      Serial.println("--- MOCK: Losing GPS Lock (Sat count dropped to 3) ---");
      MOCK_SAT_COUNT = 3; 
    } 
    lastLockStateChangeTime = millis();
  }
  
  // --- CRITICAL LOCK CHECK AND REVERSION ---
  if (MOCK_SAT_COUNT < 4) {
    showBootSplash(); 
    MOCK_SAT_COUNT = 8; 
    lastLockStateChangeTime = millis(); 
    lastMpuUpdate = millis(); 
    return; 
  }

  // 1. Calculate the Distance (in Kilometers) and Bearing
  float distanceKm = calculateDistance(MOCK_CURRENT_LAT, MOCK_CURRENT_LNG, HOME_LAT, HOME_LNG);
  float bearingToHome = calculateBearingToHome(MOCK_CURRENT_LAT, MOCK_CURRENT_LNG, HOME_LAT, HOME_LNG);
  
  // 2. Determine Relative Angles
  float relativeAngle = bearingToHome - MOCK_HEADING;
  relativeAngle = fmod(relativeAngle + 360.0, 360.0);
  
  float northRelativeAngle = fmod(-MOCK_HEADING + 360.0, 360.0);
  
  // 3. Convert Altitude (feet) and Speed (mph)
  float altitudeFeet = MOCK_ALTITUDE_M * METER_TO_FEET;
  float speedMph = MOCK_SPEED_KPH * KM_TO_MILES;

  // ===========================================
  // TTH CALCULATION (HH:MM:SS or MM:SS)
  // ===========================================
  float distanceMeters = distanceKm * 1000.0;
  float speedMetersPerSecond = MOCK_SPEED_KPH / 3.6; // Convert KPH to M/S
  
  String timeString;
  
  if (speedMetersPerSecond < 0.2 || distanceMeters < 5.0) { // If nearly stopped or very close
      timeString = "ARRIVED"; 
  } else {
      // TTH calculation in seconds
      int tthTotalSeconds = round(distanceMeters / speedMetersPerSecond);
      
      int tthHours = tthTotalSeconds / 3600;
      int tthMinutes = (tthTotalSeconds % 3600) / 60;
      int tthRemainingSeconds = tthTotalSeconds % 60;
      
      if (tthHours > 0) {
          // Format as HH:MM:SS
          timeString = String(tthHours) + ":" + 
                       (tthMinutes < 10 ? "0" : "") + String(tthMinutes) + ":" + 
                       (tthRemainingSeconds < 10 ? "0" : "") + String(tthRemainingSeconds);
      } else {
          // Format as MM:SS (for times under 1 hour)
          timeString = (tthMinutes < 10 && tthTotalSeconds >= 60 ? "0" : "") + String(tthMinutes) + ":" + 
                       (tthRemainingSeconds < 10 ? "0" : "") + String(tthRemainingSeconds);
      }
  }
  // ===========================================
  
  // Clear the buffer for a fresh screen update
  display.clearDisplay();
  
  // Set default text size and color
  display.setTextColor(SSD1306_WHITE);
  
  // ===========================================
  // ROW 1: CRITICAL DATA (Speed, Alt, Pitch)
  // ===========================================
  display.setTextSize(1); 
  int yTop = 0;
  
  // 1. Speed (X.X mph) - Left
  display.setCursor(0, yTop); 
  display.print("S:");
  display.print(speedMph, 1);
  display.print("mph"); 
  
  // 2. Altitude (X ft) - Middle
  display.setCursor(50, yTop); 
  display.print("A:");
  display.print(altitudeFeet, 0); 
  display.print("ft"); 
  
  // 3. Pitch (X°) - Right
  display.setCursor(95, yTop); 
  display.print("P:");
  if (mpuIsPresent) {
      display.print(currentPitch, 0); 
      display.print(F("\xF8")); // Degree symbol
  } else {
      display.print("N/A");
  }
  
  // Draw the top horizontal divider line
  display.drawFastHLine(0, 8, SCREEN_WIDTH, SSD1306_WHITE);


  // ===========================================
  // ROW 2: PRIMARY NAVIGATION (Time to Home)
  // ===========================================
  display.setTextSize(3); 
  int yMiddle = 22; // Center Y

  // Center the large text
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(timeString, 0, yMiddle, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, yMiddle);
  display.print(timeString);
  
  display.drawFastHLine(0, 53, SCREEN_WIDTH, SSD1306_WHITE);

  // ===========================================
  // ROW 3: NAVIGATION HELPERS (Arrows, Heading, Sats)
  // ===========================================
  display.setTextSize(1);
  int yBottom = 55;
  int arrowCenterY = yBottom + 4;

  // 1. Home Arrow (Larger, Left) - isNorth = false (Draws a Triangle)
  drawDirectionShape(8, arrowCenterY, 8, relativeAngle, false); 
  
  // 2. North Indicator (Smaller, Center Left) - isNorth = true (Draws a rotating 'N')
  drawDirectionShape(25, arrowCenterY, 6, northRelativeAngle, true);
  
  // 3. Heading (X.X°) - Center
  display.setCursor(45, yBottom); 
  display.print("H:");
  display.print(MOCK_HEADING, 0); 
  display.print(F("\xF8")); // Degree symbol
  
  // 4. Satellites - Right
  display.setCursor(95, yBottom); 
  display.print(MOCK_SAT_COUNT); 
  display.print("sats");
  
  // Final screen update
  display.display();

  delay(500);
}
