#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define IR sensor and servo motor pins for entry and exit
int entryIRPin = A2; // Analog input pin for entry IR sensor
int exitIRPin = A3;  // Analog input pin for exit IR sensor
int entryServoPin = 11; // Control pin for entry servo motor
int exitServoPin = 12; // Control pin for exit servo motor
unsigned long entryDetectionTime = 0; // Variable to store the time when entry IR sensor detects a vehicle
unsigned long exitDetectionTime = 0; // Variable to store the time when exit IR sensor detects a vehicle
const int IR_DISTANCE_THRESHOLD = 800; // Set your desired distance threshold (calibrated in your scenario)

// Define ultrasonic sensor and buzzer pins
const int frontTrigPin = 9; // Trigger pin for the front ultrasonic sensor
const int frontEchoPin = 10; // Echo pin for the front ultrasonic sensor
const int frontBuzzerPin = 5; // Pin for the front buzzer
const int rearTrigPin = 7; // Trigger pin for the rear ultrasonic sensor
const int rearEchoPin = 8; // Echo pin for the rear ultrasonic sensor
const int rearBuzzerPin = 6; // Pin for the rear buzzer

// Define IR sensor pins for parking slot availability
const int irSensorPin1 = 13; // IR proximity sensor pin for slot 3
const int irSensorPin2 = 3; // IR proximity sensor pin for slot 2
const int irSensorPin3 = 4; // IR proximity sensor pin for slot 1

// Define LCD I2C address and pins
LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C address 0x27, 16x2 display
const int potentiometerPin = A0; // Potentiometer pin for contrast adjustment

Servo entryServo; // Create entry servo object
Servo exitServo; // Create exit servo object

bool isSlotEmpty1 = true;
bool isSlotEmpty2 = true;
bool isSlotEmpty3 = true;

void setup() {
  entryServo.attach(entryServoPin); // Attach entry servo to the specified pin
  exitServo.attach(exitServoPin); // Attach exit servo to the specified pin

  pinMode(entryIRPin, INPUT); // Set entry IR sensor pin as an input
  pinMode(exitIRPin, INPUT); // Set exit IR sensor pin as an input

  pinMode(frontTrigPin, OUTPUT);
  pinMode(frontEchoPin, INPUT);
  pinMode(frontBuzzerPin, OUTPUT);

  pinMode(rearTrigPin, OUTPUT);
  pinMode(rearEchoPin, INPUT);
  pinMode(rearBuzzerPin, OUTPUT);

  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the backlight
  pinMode(irSensorPin1, INPUT); // Set the IR sensor pin for slot 1 as an input
  pinMode(irSensorPin2, INPUT); // Set the IR sensor pin for slot 2 as an input
  pinMode(irSensorPin3, INPUT); // Set the IR sensor pin for slot 3 as an input

  Serial.begin(9600); // Initialize the serial monitor for debugging
}

void loop() {
  // IR sensors for entry and exit control
  int entryIRValue = digitalRead(entryIRPin);
  int exitIRValue = digitalRead(exitIRPin);

  if (entryIRValue == LOW || analogRead(entryIRPin) < IR_DISTANCE_THRESHOLD) {
    if (entryDetectionTime == 0) {
      entryDetectionTime = millis(); // Record the time when the vehicle is detected at the entry
    }
    // Open the entry gate
    entryServo.write(90); // Vehicle detected at entry, open entry gate (adjust angle if needed)
  } else {
    entryDetectionTime = 0; // Reset the time when no vehicle is detected
    entryServo.write(0); // No vehicle at entry, close entry gate (adjust angle if needed)
  }

  // Close the entry gate 5 seconds after detection
  if (entryDetectionTime > 0 && (millis() - entryDetectionTime > 5000)) {
    entryServo.write(0); // Close the entry gate after 5 seconds
    entryDetectionTime = 0; // Reset the time after closing the gate
  }

  if (exitIRValue == LOW || analogRead(exitIRPin) < IR_DISTANCE_THRESHOLD) {
    exitServo.write(90); // Vehicle detected at exit, open exit gate (adjust angle if needed)
  } else {
    exitServo.write(0); // No vehicle at exit, close exit gate (adjust angle if needed)
  }

  // Ultrasonic sensors for object detection and buzzer activation
  detectAndAlert(frontTrigPin, frontEchoPin, frontBuzzerPin, "Front");
  detectAndAlert(rearTrigPin, rearEchoPin, rearBuzzerPin, "Rear");

  // IR sensors for parking slot availability on the LCD display
  int sensorValue1 = digitalRead(irSensorPin1);
  int sensorValue2 = digitalRead(irSensorPin2);
  int sensorValue3 = digitalRead(irSensorPin3);
  int contrastValue = analogRead(potentiometerPin);

  int contrast = map(contrastValue, 0, 1023, 0, 255);
  lcd.setContrast(contrast);

  isSlotEmpty1 = (sensorValue1 == HIGH);
  isSlotEmpty2 = (sensorValue2 == HIGH);
  isSlotEmpty3 = (sensorValue3 == HIGH);

  lcd.setCursor(0, 0);
  // lcd.print("S1: ");
  lcd.print(isSlotEmpty1 ? "S1:Free" : "S1:Fill");
  lcd.setCursor(9, 0);
  // lcd.print("S2: ");
  lcd.print(isSlotEmpty2 ? "S2:Free" : "S2:Fill");
  lcd.setCursor(0, 1);
  // lcd.print("S3: ");
  lcd.print(isSlotEmpty3 ? "S3:Free" : "S3:Fill");
  lcd.setCursor(9, 1);
  // lcd.print("S3: ");
  lcd.print("BE-IOT");

  delay(1000); // Add a small delay for stability
}

void detectAndAlert(int trigPin, int echoPin, int buzzerPin, String location) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration / 58.2;

  Serial.print("Distance (");
  Serial.print(location);
  Serial.print("): ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= 5) {
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  delay(100);
}
