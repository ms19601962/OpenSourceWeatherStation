// Requires libraries
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define pin numbers for the 74HC165
const int LOAD_PIN = 7;   // Load (Latch) pin
const int CLOCK_PIN = 4;  // Clock pin
const int DATA_PIN = 5;   // Data pin

// Define variables to hold the input values and wind direction
int inputValues = 0;
String windDirection = "N";  // initialize the wind direction to North

// Define variables for BMP680 sensor

#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)
Adafruit_BMP280 bmp;

// Tipping Bcket Setup

LiquidCrystal_I2C lcd(0x27, 20, 4);                   // Initialize the LCD display
const float mmPerPulse = 0.173;                       // Rainfall in mm per bucket tip
float mmTotal = 0;                                    // Total rainfall in mm
unsigned long lastUpdateTime = 0;                     // Timestamp for the last update
const unsigned long updateInterval = 30 * 60 * 1000;  // Update every 30 minutes

int sensorPin = 9;      // Digital pin connected to the tipping bucket sensor
int previousState = 0;  // Previous state of the sensor


void setup() {
  // Tipping buckey
  pinMode(sensorPin, INPUT);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(4, 0);
  lcd.print("Rain Gauge");
  delay(1000);
  lcd.clear();

  // Set the pin modes for the 74HC165
  pinMode(LOAD_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, INPUT);

  // Set the initial state for the Load pin (active low)
  digitalWrite(LOAD_PIN, HIGH);

  // Initialize the serial communication
  Serial.begin(9600);

  // Initialize BMP280
  unsigned status;
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                     "try a different address!"));
  }
}
void loop() {
  //ipping Bucket
   int currentState = digitalRead(sensorPin);
  if (currentState != previousState) {
    mmTotal += mmPerPulse; // Increment total rainfall
  }
  previousState = currentState;

  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;
    // Display total rainfall on the LCD
    lcd.setCursor(2, 2);
    lcd.print("Total Rainfall:");
    lcd.setCursor(2, 3);
    lcd.print(mmTotal);
    lcd.print(" mm");
  } 

  // Trigger the shift register to read the input values
  digitalWrite(LOAD_PIN, LOW);
  delayMicroseconds(1);
  digitalWrite(LOAD_PIN, HIGH);

  // Read the input values from the shift register
  inputValues = 0;
  for (int i = 0; i < 8; i++) {
    inputValues |= digitalRead(DATA_PIN) << i;
    digitalWrite(CLOCK_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(CLOCK_PIN, LOW);
  }

  // Determine the wind direction based on the input values
  switch (inputValues) {
    case 0b01111111: windDirection = "N"; break;
    case 0b00111111: windDirection = "NNE"; break;
    case 0b10111111: windDirection = "NE"; break;
    case 0b10011111: windDirection = "ENE"; break;
    case 0b11011111: windDirection = "E"; break;
    case 0b11001111: windDirection = "ESE"; break;
    case 0b11101111: windDirection = "SE"; break;
    case 0b11100111: windDirection = "SSE"; break;
    case 0b11110111: windDirection = "S"; break;
    case 0b11110011: windDirection = "SSW"; break;
    case 0b11111011: windDirection = "SW"; break;
    case 0b11111001: windDirection = "WSW"; break;
    case 0b11111101: windDirection = "W"; break;
    case 0b11111100: windDirection = "WNW"; break;
    case 0b11111110: windDirection = "NW"; break;
    case 0b01111110: windDirection = "NNW"; break;
    default: windDirection = "Invalid"; break;
  }

  // Print the input values and wind direction to the serial monitor
  Serial.print("Input Values: ");
  Serial.print(inputValues);
  Serial.print(", Wind Direction: ");
  Serial.println(windDirection);

  // BMP280 readings
  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  Serial.println(" m");
  // Wait for a short time before reading again
  delay(100);
}
