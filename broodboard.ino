#include <LiquidCrystal.h>
#include <Servo.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin definitions
#define MOISTURE_PIN A1       // Analog pin for moisture sensor
#define LDR_PIN A2            // Analog pin for LDR
#define LED1_PIN 6            // Digital pin for LED 1
#define LED2_PIN 7            // Digital pin for LED 2
#define LED3_PIN 8            // Digital pin for LED 3
#define SERVO_PIN 9           // PWM pin for servo motor
#define TEMPERATURE_PIN 10    // Digital pin for DS18B20 temperature sensor
#define WATER_SENSOR_PIN A5   // Analog pin for water sensor
#define ULTRASONIC_TRIG_PIN A3 // Digital pin for ultrasonic sensor trigger
#define ULTRASONIC_ECHO_PIN A4 // Digital pin for ultrasonic sensor echo

// LCD pin definitions
LiquidCrystal lcd_1(12, 11, 5, 4, 3, 2); // RS, Enable, D4, D5, D6, D7

// Servo motor object
Servo servoMotor;

// Setup a oneWire instance to communicate with DS18B20 temperature sensor
OneWire oneWire(TEMPERATURE_PIN);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

unsigned long previousMillis = 0;
const long interval = 10000; // Interval in milliseconds to switch between sensor readings (10 seconds)
bool displayTemperatureMoisture = true; // Flag to toggle display between temperature/moisture and ultrasonic/water

void setup() {
  Serial.begin(9600);   // Initialize serial communication
  lcd_1.begin(16, 2);    // Set up the number of columns and rows on the LCD.
  lcd_1.print("Temp:     C");

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(WATER_SENSOR_PIN, INPUT);    // Set water sensor pin as input
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); // Set ultrasonic sensor trigger pin as output
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);  // Set ultrasonic sensor echo pin as input

  // Initialize servo motor
  servoMotor.attach(SERVO_PIN);

  // Start up the DallasTemperature library
  sensors.begin();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    displayTemperatureMoisture = !displayTemperatureMoisture; // Toggle display flag
  }

  if (displayTemperatureMoisture) {
    displayTemperatureMoistureValues();
  } else {
    displayUltrasonicWaterValues();
  }

  // Output temperature to Serial Monitor (for continuous monitoring)
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  delay(100); // Adjust delay as needed to ensure responsiveness
}

void displayTemperatureMoistureValues() {
  // Request temperature from DS18B20 sensor
  sensors.requestTemperatures();

  // Read temperature in Celsius
  float temperatureC = sensors.getTempCByIndex(0);

  // Read other sensor values
  int moistureSensorValue = analogRead(MOISTURE_PIN);
  int ldrValue = analogRead(LDR_PIN);

  // Print temperature to LCD
  lcd_1.setCursor(0, 0);
  lcd_1.print("Temp:     "); // Ensure it always prints "Temp:" to reset the display
  lcd_1.print(temperatureC, 1); // Print temperature with one decimal place
  lcd_1.setCursor(0, 1);
  lcd_1.print("Moisture: ");
  lcd_1.print(moistureSensorValue);
  lcd_1.print("   "); // Clear any remaining characters

  // Control servo motor based on temperature
  if (temperatureC > 25.0) {
    moveServo(180); // Move servo motor to the right (clockwise)
  } else if (temperatureC < 19.0) {
    moveServo(0);   // Move servo motor to the left (counterclockwise)
  } else {
    moveServo(90);  // Move servo motor to a neutral position
  }

  // Control LEDs based on LDR value
  if (ldrValue < 20) {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
  } else {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
  }
}

void displayUltrasonicWaterValues() {
  // Clear previous temperature and moisture values on LCD
  lcd_1.setCursor(0, 0);
  lcd_1.print("                "); // Clear temperature display
  lcd_1.setCursor(0, 1);
  lcd_1.print("                "); // Clear moisture display

  // Read water sensor value
  int waterSensorValue = analogRead(WATER_SENSOR_PIN);

  // Read ultrasonic sensor distance
  float ultrasonicDistance = readUltrasonicDistance();

  // Print ultrasonic and water sensor values to LCD
  lcd_1.setCursor(0, 0);
  lcd_1.print("Water: ");
  lcd_1.print(waterSensorValue);
  lcd_1.setCursor(0, 1);
  lcd_1.print("Distance: ");
  lcd_1.print(ultrasonicDistance);
  lcd_1.print(" cm   "); // Clear any remaining characters
}

void moveServo(int degrees) {
  // Map degrees to servo pulse width (0 to 180)
  servoMotor.write(degrees);
  delay(500); // Allow time for the servo to reach the position
}

float readUltrasonicDistance() {
  // Trigger pulse to the ultrasonic sensor
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // Measure the duration of the pulse from the echo pin
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);

  // Calculate distance in centimeters
  float distance = duration * 0.034 / 2; // Speed of sound is 34 cm per millisecond

  return distance;
}
