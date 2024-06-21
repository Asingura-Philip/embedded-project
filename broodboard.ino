#include <LiquidCrystal.h>
#include <Servo.h>

//define pins
#define LDR_PIN A2           // Analog pin for LDR
#define LED1_PIN 6           // Digital pin for LED 1
#define LED2_PIN 7           // Digital pin for LED 2
#define LED3_PIN 8           // Digital pin for LED 3
#define SERVO_PIN 9          // PWM pin for servo motor



Servo servoMotor;

void setup() {
    Serial.begin(9600);  // Initialize serial communication
    lcd_1.begin(16, 2);   // Set up the number of columns and rows on the LCD.
    lcd_1.print("Temp:     C");

    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);

    servoMotor.attach(SERVO_PIN);


}

void loop() {
  // Control LEDs based on LDR value (example)
  if (ldrValue < 60) {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
  } else {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
  }

}
