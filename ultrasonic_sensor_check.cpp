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
  // Rotate the servo to 90 degrees
  myServo.write(90);
  delay(1000); // Wait for the servo to reach the position
  
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
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Add a small delay before the next reading
  delay(2000); // Wait for 2 seconds before the next measurement
}
