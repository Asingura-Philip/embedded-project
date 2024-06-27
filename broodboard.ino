#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <Servo.h>

// Pin definitions
#define MOISTURE_PIN A1         // Analog pin for moisture sensor
#define LDR_PIN A2              // Analog pin for LDR
#define LED1_PIN 6              // Digital pin for LED 1
#define LED2_PIN 7              // Digital pin for LED 2
#define LED3_PIN 8              // Digital pin for LED 3
#define SERVO_PIN 9             // PWM pin for servo motor
#define TEMPERATURE_PIN 10      // Digital pin for temperature sensor
#define WATER_SENSOR_PIN A5     // Analog pin for water sensor
#define ULTRASONIC_TRIG_PIN A3  // Digital pin for ultrasonic sensor trigger
#define ULTRASONIC_ECHO_PIN A4  // Digital pin for ultrasonic sensor echo
#define BUZZER_PIN A5            // Digital pin for buzzer (changed to pin A5)
#define PUSH_BUTTON_PIN 13      // Digital pin for push button

// LCD pin definitions
LiquidCrystal lcd_1(12, 11, 5, 4, 3, 2); // RS, Enable, D4, D5, D6, D7

// Servo motor object
Servo servoMotor;

// OneWire instance for DallasTemperature sensor
OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature sensors(&oneWire);

// Enum to define display modes
enum DisplayMode {
  TEMPERATURE_MODE,
  SENSOR_DATA_MODE
};

DisplayMode currentMode = TEMPERATURE_MODE; // Initial display mode

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  lcd_1.begin(16, 2);   // Set up the number of columns and rows on the LCD
  lcd_1.print("Temp:     C");

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(WATER_SENSOR_PIN, INPUT);     // Set water sensor pin as input
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT); // Set ultrasonic sensor trigger pin as output
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);  // Set ultrasonic sensor echo pin as input
  pinMode(BUZZER_PIN, OUTPUT);          // Set buzzer pin as output
  pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP); // Set push button pin as input with internal pull-up resistor

  // Initialize servo motor
  servoMotor.attach(SERVO_PIN);

  // Start up the DallasTemperature library
  sensors.begin();
}

void loop() {
  int ldrValue = analogRead(LDR_PIN);
  Serial.print("LDR Value: ");
  Serial.println(ldrValue);
  // Check if the push button is pressed to toggle display
  if (digitalRead(PUSH_BUTTON_PIN) == LOW) {
    toggleDisplayMode(); // Toggle display mode immediately
    delay(200); // Button debounce delay
    lcd_1.clear(); // Clear LCD when switching display
    if (currentMode == TEMPERATURE_MODE) {
      lcd_1.print("Temp:     C"); // Print temperature label after clearing
    }
  }

  // Update display based on current mode
  if (currentMode == TEMPERATURE_MODE) {
    updateTemperatureDisplay();
  } else if (currentMode == SENSOR_DATA_MODE) {
    updateSensorDataDisplay();
  }

  delay(200); // Delay between display updates (adjust as needed)
}

void toggleDisplayMode() {
  if (currentMode == TEMPERATURE_MODE) {
    currentMode = SENSOR_DATA_MODE;
  } else {
    currentMode = TEMPERATURE_MODE;
  }
}

void updateTemperatureDisplay() {
  // Request temperature reading
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0); // Read temperature in Celsius

  // Read moisture sensor value
  int moistureSensorValue = analogRead(MOISTURE_PIN);

  // Print temperature to LCD
  lcd_1.setCursor(5, 0);
  lcd_1.print(temperatureC, 1); // Print temperature with one decimal place
  lcd_1.setCursor(0, 1);
  lcd_1.print("Moisture: ");
  lcd_1.print(moistureSensorValue);
  lcd_1.print("   "); // Clear any remaining characters

  // Control servo motor based on temperature
  if (temperatureC > 28.0) {
    moveServo(180); // Move servo motor to the right (clockwise)
  } else if (temperatureC < 26.0) {
    moveServo(0);   // Move servo motor to the left (counterclockwise)
  } else {
    moveServo(90);  // Move servo motor to a neutral position
  }

  // Control LEDs based on LDR value (example)
  int ldrValue = analogRead(LDR_PIN);
  if (ldrValue > 40) {
    // When ldrValue is greater than 40, turn off all LEDs
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
    } else if (ldrValue >= 20 && ldrValue <= 40) {
    // For ldrValue between 20 and 40 (inclusive), turn on only LED3
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, HIGH);
    } else {
    // When ldrValue is less than 20, turn on all LEDs
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
    }



  // Output temperature to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println(" °C");
}

void updateSensorDataDisplay() {
  // Read water sensor value
  int waterSensorValue = analogRead(WATER_SENSOR_PIN);

  // Read ultrasonic sensor value (example)
  long duration, distance;
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  // Print water and ultrasonic sensor values to LCD
  lcd_1.setCursor(0, 0);
  lcd_1.print("Water: ");
  lcd_1.print(waterSensorValue);
  lcd_1.setCursor(0, 1);
  lcd_1.print("Distance: ");
  lcd_1.print(distance);
  lcd_1.print(" cm  ");

  // Activate buzzer if distance is above 10cm
  if (distance > 10) {
    digitalWrite(BUZZER_PIN, HIGH);
    
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    
  }

  // Output ultrasonic sensor distance to Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void moveServo(int degrees) {
  // Map degrees to servo pulse width (0 to 180)
  servoMotor.write(degrees);
  delay(500); // Allow time for the servo to reach the position
}
