#include <Servo.h>

// Define the pins for the ultrasonic sensor
#define TRIGGER_PIN A2
#define ECHO_PIN A3

// Create a servo object
Servo myServo;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Attach the servo to pin 9
  myServo.attach(9);
  
  // Set the trigger pin as an output and the echo pin as an input
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Rotate the servo to 150 degrees and measure distance
  myServo.write(150);
  delay(1000); // Wait for the servo to reach the position
  measureDistance(150);

  // Rotate the servo to 90 degrees and measure distance
  myServo.write(90);
  delay(1000); // Wait for the servo to reach the position
  measureDistance(90);

  // Rotate the servo to 30 degrees and measure distance
  myServo.write(30);
  delay(1000); // Wait for the servo to reach the position
  measureDistance(30);

  // Add a small delay before the next cycle
  delay(2000); // Wait for 2 seconds before the next measurement cycle
}

void measureDistance(int angle) {
  // Clear the trigger pin
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin high for 10 microseconds
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  // Read the echo pin and calculate the duration of the pulse
  long duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate the distance in centimeters
  long distance = duration * 0.034 / 2;
  
  // Print the distance to the Serial Monitor
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" degrees | Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}
