#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h> 
#include <math.h> 

// --- OLED Display Configuration (Based on your specs) ---
#define SCREEN_WIDTH 128    // OLED display width, in pixels
#define SCREEN_HEIGHT 64    // OLED display height, in pixels

// PIN DEFINITIONS: Using the constants defined by the ESP8266 core for the NodeMCU board
// SDA is GPIO14 (D5), SCL is GPIO12 (D6)
#define OLED_ADDR   0x3C    // I2C Address 0x3C
#define OLED_RESET  -1      // Reset pin (-1 for software reset)

// Initialize the display objects (OLED and MPU)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu; 

// --- Conversion Constants ---
const float KM_TO_MILES = 0.621371;
const float METER_TO_FEET = 3.28084;
const float MILES_TO_FEET = 5280.0; // 1 mile = 5280 feet

// --- Mock Data Variables ---
const char* MOCK_DATE = "11/21/25"; 
const char* MOCK_TIME = "12:09:45"; 
const float MOCK_CURRENT_LAT = 37.425123; 
const float MOCK_CURRENT_LNG = -122.086456;
int MOCK_SAT_COUNT = 0; 
const float MOCK_ALTITUDE_M = 30.5; 
const float MOCK_SPEED_KPH = 0.5; 

// MOCK_PITCH is now the real-time calculated pitch
float currentPitch = 0.0; // Initialize to zero, will be updated by MPU

const float MOCK_HEADING = 90.0; // Direction of Travel (Course) in degrees (0=N, 90=E)

// --- HOME Location (Reference Point for Distance Calculation) ---
const float HOME_LAT = 37.421999;
const float HOME_LNG = -122.084058;
const float R_EARTH_KM = 6371.0; // Earth radius in kilometers

// --- Global Mock State for Cycling Lock Status ---
unsigned long lastLockStateChangeTime = 0;
const unsigned long lockStateChangeInterval = 15000; 

// --- Global MPU State Variables ---
// Offsets to be calculated during startup calibration
float gyroXoffset = 0.0;
float gyroYoffset = 0.0;
float gyroZoffset = 0.0;

// Time variables for calculating delta time (dt) for sensor fusion
unsigned long lastMpuUpdate = 0;
const float alpha = 0.98; // Complementary filter constant (0.98 for fast gyro tracking)

// --- FUNCTION PROTOTYPES (Required since functions are below setup/loop) ---
void initializeMPU();
void performGyroCalibration();
void updateAttitude();
// --- END PROTOTYPES ---


/**
 * @brief Calculates the distance between two GPS coordinates using the Haversine formula.
 * @param lat1 Latitude 1 (degrees)
 * @param lon1 Longitude 1 (degrees)
 * @param lat2 Latitude 2 (degrees)
 * @param lon2 Longitude 2 (degrees)
 * @return Distance in miles (mi).
 */
float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  // Convert degrees to radians
  float dLat = (lat2 - lat1) * (M_PI / 180.0);
  float dLon = (lon2 - lon1) * (M_PI / 180.0);

  // Convert to radians for Haversine calculation
  lat1 = lat1 * (M_PI / 180.0);
  lat2 = lat2 * (M_PI / 180.0);

  // Apply Haversine formula
  float a = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  // Calculate distance in km, then convert to miles
  float distanceKm = R_EARTH_KM * c; 
  return distanceKm * KM_TO_MILES;
}

/**
 * @brief Calculates the initial bearing (course) from point 1 to point 2.
 * @return Bearing in degrees (0 to 360).
 */
float calculateBearingToHome(float lat1, float lon1, float lat2, float lon2) {
  // Convert degrees to radians
  lat1 = lat1 * (M_PI / 180.0);
  lon1 = lon1 * (M_PI / 180.0);
  lat2 = lat2 * (M_PI / 180.0);
  lon2 = lon2 * (M_PI / 180.0);
  
  float dLon = lon2 - lon1;

  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  float bearingRad = atan2(y, x);
  
  // Convert bearing from radians to degrees and normalize to 0-360
  float bearingDeg = bearingRad * (180.0 / M_PI);
  return fmod(bearingDeg + 360.0, 360.0);
}

/**
 * @brief Draws a simple closed lock graphic.
 * @param cx Center X position of the lock.
 * @param cy Top Y position of the lock body.
 */
void drawClosedLock(int cx, int cy) {
    // Lock Body: 20x15 filled rectangle
    int bodyW = 20;
    int bodyH = 15;
    display.fillRect(cx - bodyW/2, cy, bodyW, bodyH, SSD1306_WHITE);

    // Lock Shackle (simplified U shape using lines)
    int shackleH = 10;
    int shackleW = 10;
    int shackleY = cy - shackleH;
    
    // Draw left vertical part
    display.drawFastVLine(cx - shackleW/2, shackleY, shackleH, SSD1306_WHITE);
    // Draw right vertical part
    display.drawFastVLine(cx + shackleW/2, shackleY, shackleH, SSD1306_WHITE);
    // Draw top horizontal part (connecting the shackle, making it 'closed')
    display.drawFastHLine(cx - shackleW/2, shackleY, shackleW + 1, SSD1306_WHITE); 
}


/**
 * @brief Displays a custom boot-up sequence and waits for a mock satellite lock (MOCK_SAT_COUNT > 3).
 * * NOTE: This function is blocking and will not return until MOCK_SAT_COUNT is 4 or greater.
 */
void showBootSplash() {
  // Reset the satellite count to 0 to restart the acquisition simulation
  MOCK_SAT_COUNT = 0; 
    
  const char* welcomeMessage = "AQUIRING GPS LOCK..."; 
  const int msgLength = strlen(welcomeMessage);
  const int charWidth = 6; 
  const int scrollDurationMs = 2000; // Time for one full scroll cycle
  
  display.setTextSize(1); 
  display.setTextColor(SSD1306_WHITE);
  const int textY = (SCREEN_HEIGHT / 2) - 4; // Center the text vertically 

  long startTime = millis();
  long lastSatUpdate = millis();
  const int satUpdateInterval = 500; // Update sat count every 500ms
  
  // Loop until MOCK_SAT_COUNT is greater than the required 3 (i.e., locked on 4)
  while (MOCK_SAT_COUNT <= 3) {
    display.clearDisplay();
    
    // --- Mock Satellite Acquisition ---
    if (millis() - lastSatUpdate > satUpdateInterval) { 
        // We only mock up to 4 satellites for the lock phase
        if (MOCK_SAT_COUNT < 4) { 
            MOCK_SAT_COUNT++;
        }
        lastSatUpdate = millis();
    }
    
    // Reset scroll timer for smooth looping scroll effect
    if (millis() - startTime > scrollDurationMs) {
        startTime = millis();
    }
    
    // --- SCROLLING LOGIC ---
    // Calculate scroll position based on time progress through one cycle
    float progress = (float)(millis() - startTime) / scrollDurationMs;
    // Total distance to scroll is screen width + message width
    int scrollDistance = 128 + (msgLength * charWidth); 
    int currentScrollX = (int)(progress * scrollDistance); 
    
    // Start drawing position (128 - scroll amount)
    int startX = 128 - currentScrollX;
    
    // Draw the message
    display.setCursor(startX, textY);
    display.print(welcomeMessage);
    
    // --- Satellite Status / Progress Bar ---
    display.drawRect(10, 50, 108, 10, SSD1306_WHITE);
    
    // Map the current MOCK_SAT_COUNT (0-4) to the progress bar width (0-106)
    int maxSatForBar = 4; 
    int fillWidth = map(MOCK_SAT_COUNT, 0, maxSatForBar, 0, 106); 
    if (fillWidth > 106) fillWidth = 106; // Cap the progress bar at max
    
    display.fillRect(11, 51, fillWidth, 8, SSD1306_WHITE); 
    
    // Draw satellite count text in the middle of the progress area
    // This text confirms the bar's purpose and shows the current locked count
    display.setTextSize(1);
    display.setCursor(11, 52); 
    display.print("GPS LOCK: ");
    display.print(MOCK_SAT_COUNT);
    display.print(" of 4");
    
    display.display();
    delay(20); // Small delay for smooth animation and loop control
  }
  
  // --- LOCK ACQUIRED GRAPHIC ---
  display.clearDisplay();
  
  // Draw the closed lock graphic, centered 
  drawClosedLock(SCREEN_WIDTH / 2, 20); 
  
  // Display Success Message below the lock
  display.setTextSize(1);
  display.setCursor(1, 45); 
  display.print("3D GPS LOCK ACQUIRED!");

  display.display();
  delay(1500); // Display the lock graphic for 1.5 seconds
  
  // Ensure the main loop knows the current count is the acquired value
  MOCK_SAT_COUNT = 4; 
}


/**
 * @brief Performs gyroscope bias calibration by reading static values and calculating offsets.
 * NOTE: The device MUST be kept perfectly still during this process.
 */
void performGyroCalibration() {
    Serial.print(F("Calibrating Gyro... DO NOT MOVE MPU!"));
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("CALIBRATING GYRO...");
    display.setCursor(0, 10);
    display.print("DO NOT MOVE!");
    display.display();
    
    // Use large variables to sum up thousands of readings
    long sumX = 0, sumY = 0, sumZ = 0;
    const int numSamples = 2000;
    
    // Read and sum up the raw gyroscope data
    for (int i = 0; i < numSamples; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        // Gyro data is in deg/s, but we use the raw reading here for summing
        sumX += g.gyro.x * 1000.0; // Multiply by 1000 to use integer math for stability
        sumY += g.gyro.y * 1000.0;
        sumZ += g.gyro.z * 1000.0;
        
        delay(1); // Small delay between readings
        
        // Show progress on the OLED
        if (i % (numSamples / 10) == 0) {
            display.drawFastVLine(0 + (i / (numSamples / 10)) * 10, 20, 10, SSD1306_WHITE);
            display.display();
        }
    }
    
    // Calculate the average offset and scale back down
    gyroXoffset = (float)sumX / numSamples / 1000.0;
    gyroYoffset = (float)sumY / numSamples / 1000.0;
    gyroZoffset = (float)sumZ / numSamples / 1000.0;
    
    Serial.println(F("Done."));
    Serial.print(F("Offsets: X=")); Serial.print(gyroXoffset, 3);
    Serial.print(F(", Y=")); Serial.print(gyroYoffset, 3);
    Serial.print(F(", Z=")); Serial.println(gyroZoffset, 3);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("CALIBRATION COMPLETE!");
    display.display();
    delay(1000); // Wait 1 second before continuing setup
}


/**
 * @brief Initializes the MPU-6050/6500 sensor on the I2C bus.
 * If initialization fails, it prints an error and halts.
 */
void initializeMPU() {
    Serial.print(F("Initializing MPU... "));

    // Attempt to initialize MPU. The MPU shares the I2C bus with the OLED.
    if (!mpu.begin()) {
        Serial.println(F("Failed to find MPU6050 chip. Check wiring on D5/D6."));
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.print("MPU ERROR!");
        display.display();
        while (1) { delay(10); } // Halt the program
    }
    
    // Set sensor ranges and filters (these are common optimal values)
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    Serial.println(F("Done."));

    // *** CALL THE NEW CALIBRATION FUNCTION HERE ***
    performGyroCalibration(); 
}

/**
 * @brief Reads MPU data and updates the global currentPitch variable using
 * a simple Complementary Filter (Sensor Fusion).
 */
void updateAttitude() {
    // Read the sensors
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate time elapsed (dt) since the last update in seconds
    unsigned long now = millis();
    float dt = (now - lastMpuUpdate) / 1000.0; 
    lastMpuUpdate = now;

    // --- Apply Gyro Calibration Offsets ---
    // Subtract the bias from the raw gyro readings to correct drift
    float correctedGyroY = g.gyro.y - gyroYoffset;

    // --- 1. Calculate Pitch from Accelerometer (Stable but noisy) ---
    // Pitch (rotation around Y-axis) = atan2(AccX, AccZ)
    float accPitch = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / M_PI;

    // --- 2. Calculate Pitch from Gyroscope (Fast but drifts) ---
    // Pitch_Gyro = Pitch_Prev + (Corrected GyroY * dt)
    float gyroChange = correctedGyroY * dt;

    // --- 3. Complementary Filter (Fusion) ---
    // Gyro provides the immediate change (98%), Acc provides the long-term stable anchor (2%).
    currentPitch = alpha * (currentPitch + gyroChange) + (1.0 - alpha) * accPitch;
    
    // Safety check to keep within bounds (-90 to 90 degrees)
    currentPitch = constrain(currentPitch, -90.0, 90.0);
}


void setup() {
  Serial.begin(115200);

  // EXPLICIT PIN INITIALIZATION: Force the I2C communication onto your specified D5 (SDA) and D6 (SCL) pins.
  Wire.begin(D5, D6); 

  // Initialize I2C Bus and Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed. Check I2C wiring and address (0x3C)."));
    for (;;); // Don't proceed if initialization fails
  }

  // --- STARTUP OPTIMIZATION: CUSTOM SPLASH SCREEN (Waits for Sat Lock) ---
  showBootSplash();
  
  // --- MPU SETUP ---
  initializeMPU(); 

  // After splash screen, set MOCK_SAT_COUNT to a strong lock value for the main loop
  MOCK_SAT_COUNT = 8; 
  lastLockStateChangeTime = millis(); // Initialize the lock state change timer
  lastMpuUpdate = millis(); // Initialize MPU update timer

  // Prepare for drawing the main loop content
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
}

/**
 * @brief Draws a simple triangle arrow rotated by an angle.
 * @param cx Center X position.
 * @param cy Center Y position.
 * @param size Length from center to tip.
 * @param angle Angle in degrees (0 = straight up/North).
 */
void drawDirectionArrow(int cx, int cy, int size, float angle) {
  // Convert angle (0-360, where 0 is up) to radians
  // We use 90 - angle because standard trig starts at 0 right (East) and goes counter-clockwise.
  // We want 0 to be up (North).
  float radians = (90.0 - angle) * (M_PI / 180.0);
  
  // Define arrow tip and base points relative to center (0,0)
  int x0 = 0;
  int y0 = -size; // Tip
  int x1 = -size / 2;
  int y1 = size / 2; // Bottom left
  int x2 = size / 2;
  int y2 = size / 2; // Bottom right

  // Points array for easier rotation
  int points[3][2] = {{x0, y0}, {x1, y1}, {x2, y2}};

  // Rotate and translate each point
  for (int i = 0; i < 3; i++) {
    int px = points[i][0];
    int py = points[i][1];
    
    // Rotation matrix: x' = x*cos(r) - y*sin(r)  and  y' = x*sin(r) + y*cos(r)
    points[i][0] = (int)(px * cos(radians) - py * sin(radians) + cx);
    points[i][1] = (int)(px * sin(radians) + py * cos(radians) + cy); 

  }

  // Draw the filled triangle arrow
  display.fillTriangle(
    points[0][0], points[0][1],
    points[1][0], points[1][1],
    points[2][0], points[2][1],
    SSD1306_WHITE
  );
}


void loop() {
  // *** NEW: Update the pitch based on the MPU readings ***
  updateAttitude(); 
  
  // --- MOCK LOCK LOSS LOGIC (Still using MOCK_SAT_COUNT for GPS) ---
  if (millis() - lastLockStateChangeTime > lockStateChangeInterval) {
    if (MOCK_SAT_COUNT >= 4) {
      // Lock held for 15s, simulate losing it (set to 3 or less)
      Serial.println("--- MOCK: Losing GPS Lock (Sat count dropped to 3) ---");
      MOCK_SAT_COUNT = 3; 
    } else {
      // Lock was lost, but we reset the timer later in showBootSplash()
    }
    lastLockStateChangeTime = millis();
  }
  
  // --- CRITICAL LOCK CHECK AND REVERSION ---
  if (MOCK_SAT_COUNT < 4) {
    showBootSplash(); 
    // Re-initialize state after acquisition
    MOCK_SAT_COUNT = 8; 
    lastLockStateChangeTime = millis(); 
    lastMpuUpdate = millis(); // Reset MPU timer
    return; 
  }

  // 1. Calculate the Distance and Bearing
  float distanceMiles = calculateDistance(MOCK_CURRENT_LAT, MOCK_CURRENT_LNG, HOME_LAT, HOME_LNG);
  float bearingToHome = calculateBearingToHome(MOCK_CURRENT_LAT, MOCK_CURRENT_LNG, HOME_LAT, HOME_LNG);
  
  // 2. Determine Relative Angle for Arrow
  float relativeAngle = bearingToHome - MOCK_HEADING;
  relativeAngle = fmod(relativeAngle + 360.0, 360.0);
  
  // 3. Convert Altitude (feet) and Speed (mph)
  float altitudeFeet = MOCK_ALTITUDE_M * METER_TO_FEET;
  float speedMph = MOCK_SPEED_KPH * KM_TO_MILES;

  
  // Clear the buffer for a fresh screen update
  display.clearDisplay();
  
  // Set text size to 1 (5x8 pixels per character) for maximum data density
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // The 'line' variable helps us manage vertical spacing easily
  int currentLine = 0; 
  const int lineHeight = 9; // 8 pixels for text + 1 pixel for spacing
  
  
  // ===========================================
  // 1. Speed (Line 1, Left) & Satellites (Line 1, Right)
  // ===========================================
  
  // --- Speed (Left Side, Size 2) ---
  display.setTextSize(2); // Set text size to 2 (10x16 pixels)
  display.setCursor(0, currentLine * lineHeight); // Y=0
  display.print(speedMph, 1);
  display.print("mph"); 
  
  // --- Satellite Count (Right Side, Size 1) ---
  display.setTextSize(1); // Set size back to 1 for smaller text
  
  // Calculate X position for right alignment (assuming max 2 digits, width 12 pixels total)
  const int SAT_NUM_WIDTH = 12; 
  const int SAT_NUM_X = SCREEN_WIDTH - SAT_NUM_WIDTH; // 128 - 12 = 116
  
  // Position the satellite number in the top right corner
  display.setCursor(SAT_NUM_X, currentLine * lineHeight + 1); 
  display.print(MOCK_SAT_COUNT); // Only print the number

  currentLine += 2; // currentLine = 2 (Y=18, moves to the next block of space)
  
  
  // ===========================================
  // 2. Altitude (Line 3) - VALUE ONLY, SIZE 2
  // ===========================================
  display.setTextSize(2); // Text size remains 2
  display.setCursor(0, currentLine * lineHeight); // Y=18
  display.print(altitudeFeet, 0); // Display whole number
  display.print("ft"); 
  
  currentLine += 2; // currentLine = 4 (Y=36)
  
  
  // ===========================================
  // 3. Direction Arrow + Distance (Line 5 Area) - NOW SIZE 2 (Two lines tall), MOVED LEFT
  // Logic: Switch to feet (ft) if distance is less than 1.0 mile.
  // ===========================================
  display.setTextSize(2); // Set text size to 2 (16 pixels high)
  
  const int distanceY = currentLine * lineHeight; // Y=36
  const int arrowCenterY = distanceY + 8; // Center of the 16 pixel height
  
  // Arrow size is 10. Center X moved from 25 to 15 (near the left edge.
  drawDirectionArrow(15, arrowCenterY, 10, relativeAngle); 
  
  // Distance text next to the arrow (Starts at Y=36). Start X moved from 40 to 30.
  display.setCursor(30, distanceY); // Y=36
  
  if (distanceMiles < 1.0) {
      // Less than 1 mile, display in feet (rounded to 0 decimal places)
      float distanceFeet = distanceMiles * MILES_TO_FEET;
      display.print(distanceFeet, 0); 
      display.print(" ft"); 
  } else {
      // 1 mile or more, display in miles (with 2 decimal places)
      display.print(distanceMiles, 2); 
      display.print(" mi"); 
  }
  
  // Note: currentLine remains 4 (Y=36) here. The next element (footer) uses a fixed index.
  
  
  // ===========================================
  // 4. Time and Date - BOTTOM FOOTER (Last line)
  // ===========================================
  // Y coordinate is 6 * 9 = 54 (the last line, index 6)
  const int footerLine = 6;
  const int footerX = 14; 
  display.setTextSize(1); // Set size back to 1 for the footer text
  display.setCursor(footerX, footerLine * lineHeight); // Y=54
  display.print(MOCK_TIME);
  display.print(" "); 
  display.print(MOCK_DATE);
  
  
  // ===========================================
  // PITCH BAR VISUALIZATION 
  // Bar starts at Y=18 (Line 3) and ends at Y=53 (above the footer)
  // ===========================================
  const int BAR_X = 117; 
  const int BAR_Y_TOP = 18; // STARTS ON THE 3RD VISUAL LINE (Y=18)
  const int BAR_Y_BOTTOM = 54;
  const int BAR_HEIGHT = BAR_Y_BOTTOM - BAR_Y_TOP; // 54 - 18 = 36 pixels high
  const int BAR_WIDTH = 10; 
  
  // 1. Normalize Pitch (using the new currentPitch variable)
  float pitchDeg = constrain(currentPitch, -90.0, 90.0);
  float normalizedPitch = (pitchDeg + 90.0) / 180.0;

  // 2. Map normalized value to bar fill height
  int fillHeight = (int)(normalizedPitch * BAR_HEIGHT);

  // 3. Determine where the fill starts (Y coordinate)
  int fillStartY = (BAR_Y_TOP + BAR_HEIGHT) - fillHeight; 
  
  // Draw the fill
  // Draw a center line for reference (0 degrees)
  display.drawFastHLine(BAR_X, BAR_Y_TOP + (BAR_HEIGHT / 2), BAR_WIDTH, SSD1306_WHITE); 
  // Draw the main fill
  display.fillRect(BAR_X + 1, fillStartY, BAR_WIDTH - 2, fillHeight, SSD1306_WHITE);

  // Draw the frame (last, so it overlaps the fill cleanly)
  display.drawRect(BAR_X, BAR_Y_TOP, BAR_WIDTH, BAR_HEIGHT, SSD1306_WHITE);

  // Push the contents of the buffer to the actual OLED screen
  display.display();

  // Wait before redrawing
  delay(500);
} // <--- This closing brace was likely missing in your local file!