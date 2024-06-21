#include <LiquidCrystal.h>
#include <Servo.h>

// Pin definitions
#define MOISTURE_PIN A1       // Analog pin for moisture sensor
#define LDR_PIN A2            // Analog pin for LDR
#define LED1_PIN 6            // Digital pin for LED 1
#define LED2_PIN 7            // Digital pin for LED 2
#define LED3_PIN 8            // Digital pin for LED 3
#define SERVO_PIN 9           // PWM pin for servo motor
#define TEMPERATURE_PIN A0    // Analog pin for temperature sensor
#define ULTRASONIC_TRIG_PIN A3  // Digital pin for ultrasonic sensor trigger
#define ULTRASONIC_ECHO_PIN A4  // Digital pin for ultrasonic sensor echo
#define WATER_SENSOR_PIN 13    // Digital pin for water sensor

// LCD pin definitions
LiquidCrystal lcd_1(12, 11, 5, 4, 3, 2); // RS, Enable, D4, D5, D6, D7

// Servo motor object
Servo servoMotor;

void setup() {
  Serial.begin(9600);   // Initialize serial communication
  lcd_1.begin(16, 2);    // Set up the number of columns and rows on the LCD.
  lcd_1.print("Temp:     C");

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); // Set ultrasonic trigger pin as output
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);  // Set ultrasonic echo pin as input
  pinMode(WATER_SENSOR_PIN, INPUT);     // Set water sensor pin as input

  // Initialize servo motor
  servoMotor.attach(SERVO_PIN);
}

void loop() {
  // Read temperature from analog pin A0
  int tempSensorValue = analogRead(TEMPERATURE_PIN);
  
  // Convert analog reading to temperature in Celsius (example conversion)
  float temperatureC = map(tempSensorValue, 0, 1023, 0, 100); // Adjust the mapping according to your sensor characteristics

  // Read other sensor values
  int moistureSensorValue = analogRead(MOISTURE_PIN);
  int ldrValue = analogRead(LDR_PIN);

  // Read ultrasonic sensor distance
  float distance = readUltrasonicDistance();

  // Read water sensor value
  int waterSensorValue = digitalRead(WATER_SENSOR_PIN);

  // Print temperature to LCD
  lcd_1.setCursor(5, 0);
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

  // Output ultrasonic sensor distance to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Output water sensor value to Serial Monitor
  Serial.print("Water Sensor Value: ");
  Serial.println(waterSensorValue);

  // Output temperature to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");

  delay(1000); // Wait for 1 second
}

void moveServo(int degrees) {
  // Map degrees to servo pulse width (0 to 180)
  servoMotor.write(degrees);
  delay(500); // Allow time for the servo to reach the position
}

float readUltrasonicDistance() {
  // Trigger ultrasonic sensor
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);

  // Calculate distance in centimeters (speed of sound is 343 m/s or 0.0343 cm/microsecond)
  float distance = duration * 0.0343 / 2;

  return distance;
}
