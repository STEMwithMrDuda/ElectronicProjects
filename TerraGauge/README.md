TerraGauge: Precision Automotive Data Display
Project Overview
TerraGauge is a compact, open-source display unit built on the ESP8266 platform, designed to provide critical vehicle and environmental data to the driver. It combines essential navigation metrics (GPS) with real-time stability information (IMU), specifically focusing on vehicle pitch (incline/decline).
Usefulness
Off-Road & Safety:
The core utility of TerraGauge is its real-time Pitch Meter. This feature is vital for 4x4, off-road, and RV applications, where excessive inclination can lead to dangerous rollovers. By using a highly stabilized Complementary Filter for sensor fusion, the display provides accurate, instant feedback on the vehicle's angle during steep climbs, descents, and side-hilling (currently only pitch is shown, but the framework supports roll). It also aids in precision leveling for camping or construction.
On-Road & Navigation:
For daily driving, TerraGauge acts as a precise digital gauge, displaying current Speed (MPH) and Altitude (FT). The Dynamic Navigation system provides crucial information for managing specific routes, showing the exact distance and bearing to a user-defined "Home" or waypoint. The unit of distance automatically switches between miles and feet when nearing the target, providing navigational accuracy in crucial moments.
The current implementation uses mock GPS data for location, speed, and heading but integrates real-time readings from the MPU-6050 sensor for accurate pitch calculation.
Features
* Real-time Pitch Meter: Displays vehicle incline/decline using a vertical bar visualization derived from MPU-6050 sensor fusion (Accelerometer and Gyroscope).
* Responsive Speed & Altitude: Displays speed in miles per hour (mph) and altitude in feet (ft).
* Dynamic Navigation: Calculates and displays distance and bearing to a user-defined "Home" coordinate.
   * Distance switches dynamically from miles (mi) to feet (ft) when the user is within 1 mile of the target.
   * A directional arrow visualizes the relative bearing to the target location.
* GPS Status Simulation: Includes a boot sequence with a scrolling splash screen and a satellite lock acquisition simulation.
* Robust Sensor Initialization: Performs mandatory Gyroscope calibration on startup to ensure accurate pitch readings.
Hardware Requirements (Bill of Materials)
Component
	Description
	Notes
	Microcontroller
	ESP8266 NodeMCU (or similar ESP8266 board)
	Provides Wi-Fi capability (future expansion) and sufficient I/O.
	Display
	0.96 inch SSD1306 OLED (128x64)
	I2C communication; address is typically 0x3C or 0x3D.
	Sensor
	MPU-6050 or MPU-6500 IMU
	Integrates 3-axis accelerometer and 3-axis gyroscope. Used for pitch calculation.
	GPS Module
	NEO-6M (Recommended)
	Required for live GPS data (currently mocked). Communicates via UART/Serial.
	Wiring
	Jumper Wires
	

	Software Requirements
This project requires the following libraries installed via the Arduino Library Manager:
1. Adafruit GFX Library (Standard Graphics Library)
2. Adafruit SSD1306 (OLED Driver)
3. Adafruit MPU6050 (Sensor Driver)
4. Adafruit BusIO (Dependency for MPU6050)
5. TinyGPS++ (Recommended for parsing NMEA data from a real GPS module)
Wiring and Connections
1. I2C Bus (OLED and MPU)
The OLED and MPU-6050 share the I2C bus for data communication.
Device Pin
	ESP8266 NodeMCU Pin
	Function
	OLED/MPU VCC
	3.3V
	Power Supply
	OLED/MPU GND
	GND
	Ground
	OLED/MPU SDA
	D5 (GPIO14)
	I2C Data Line
	OLED/MPU SCL
	D6 (GPIO12)
	I2C Clock Line
	CRITICAL NOTE: If you encounter an "MPU ERROR!" message on the display, it means the microcontroller failed to initialize the MPU sensor. This is almost always due to incorrect wiring, a loose connection, or an incorrect I2C address for the sensor.
2. Real GPS Module Integration (Serial/UART)
To upgrade the project from mock data to live GPS tracking (using a module like the NEO-6M), you must wire the GPS module to the ESP8266's secondary serial port pins.
GPS Module Pin
	ESP8266 NodeMCU Pin
	Function
	GPS VCC
	3.3V
	Power Supply
	GPS GND
	GND
	Ground
	GPS TX (Transmit)
	D4 (GPIO2)
	Connects to ESP8266 RX (Receive)
	GPS RX (Receive)
	D8 (GPIO15)
	Connects to ESP8266 TX (Transmit)
	Note: The ESP8266's hardware serial pins (TX0/RX0) are typically reserved for uploading code and debugging. Using D4 and D8 allows for SoftwareSerial or the ESP's alternate hardware serial configuration.
Code Overview (mock_gps_display.ino)
The code is structured with dedicated functions for sensor initialization, attitude calculation, and display logic.
1. Sensor Fusion (updateAttitude())
This is the core logic for the pitch meter. It uses a Complementary Filter to combine the fast-reacting Gyroscope data (for high-frequency changes) and the drift-free Accelerometer data (for long-term stability).
float correctedGyroY = g.gyro.y - gyroYoffset;
float accPitch = atan2(a.acceleration.x, a.acceleration.z) * 180.0 / M_PI;

// Fusion: 98% Gyro movement + 2% Accelerometer correction
currentPitch = alpha * (currentPitch + gyroChange) + (1.0 - alpha) * accPitch;

2. Startup and Calibration (setup() and performGyroCalibration())
Upon boot, the display runs a simulated GPS lock sequence (showBootSplash). Once finished, it initiates the MPU and runs the performGyroCalibration() function, which captures 2000 samples of gyro data to calculate and store static bias offsets. The device must remain perfectly still during this 2-second calibration period.
3. Display Logic (loop())
The main loop updates data and redraws the screen every 500 milliseconds.
* Navigation Calculations: Uses the calculateDistance (Haversine formula) and calculateBearingToHome functions to determine the current distance and course to the hardcoded HOME_LAT and HOME_LNG.
* Relative Arrow: The drawDirectionArrow function rotates an arrow graphic based on the difference between the vehicle's heading (MOCK_HEADING) and the bearing to the target (bearingToHome).
* Pitch Bar: The pitch angle (currentPitch, from -90° to 90°) is mapped to the vertical fill of the bar on the right side of the screen, providing a visual gauge of the vehicle's incline.
4. Customizing GPS Data
To use this project with a real GPS module (e.g., NEO-6M), you would replace the hardcoded mock variables in the global scope with live data retrieval from your GPS module's serial output (often using the TinyGPS++ library).
Replace the following mock variables with your live parsed data:
Mock Variable
	Description
	MOCK_CURRENT_LAT
	Live Latitude from GPS
	MOCK_CURRENT_LNG
	Live Longitude from GPS
	MOCK_ALTITUDE_M
	Live Altitude in meters
	MOCK_SPEED_KPH
	Live Speed in km/h
	MOCK_HEADING
	Live Course/Heading in degrees
	MOCK_SAT_COUNT
	Live Satellite Count
	MOCK_DATE / MOCK_TIME
	Live Date/Time strings
	HOME_LAT / HOME_LNG
	(Keep) Hardcoded Target Location
