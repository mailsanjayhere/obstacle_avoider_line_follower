#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

// hc-sr04 sensor
#define TRIGGER_PIN A2
#define ECHO_PIN A3
#define max_distance 50

// ir sensor
#define irLeft A0
#define irRight A1

// motor
#define MOTOR_SPEED 120  // Single speed for all operations

Servo servo;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

int distance = 0;
int leftDistance;
int rightDistance;
boolean object = false;
bool isStopped = false;  // Flag to check if the robot is stopped

// IR sensor thresholds
int irThreshold = 500;  // Adjust based on your environment

void setup() {
  Serial.begin(9600);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  servo.attach(10);
  servo.write(90); // Center the servo

  // Set the speed for all motors
  motor1.setSpeed(MOTOR_SPEED);
  motor2.setSpeed(MOTOR_SPEED);
  motor3.setSpeed(MOTOR_SPEED);
  motor4.setSpeed(MOTOR_SPEED);
}

void loop() {
  // Read IR sensor values as analog
  int leftIR = analogRead(irLeft);
  int rightIR = analogRead(irRight);

  // Print the IR sensor values for debugging
  Serial.print("Left IR: ");
  Serial.print(leftIR);
  Serial.print(" - Right IR: ");
  Serial.println(rightIR);

  // If both sensors detect black, resume normal operation
  if (leftIR > irThreshold && rightIR > irThreshold) {
    isStopped = false;  // Reset the stopped flag when back on track
    moveForward();      // Move forward
  }
  // If the robot is off-track (both sensors detect white) and not already stopped
  else if (leftIR <= irThreshold && rightIR <= irThreshold && !isStopped) {
    StopAndRealign();
  }
  // Turn left or right if one sensor detects black and the other detects white
  else if (leftIR > irThreshold && rightIR <= irThreshold && !isStopped) {
    turnLeft();
  } else if (leftIR <= irThreshold && rightIR > irThreshold && !isStopped) {
    turnRight();
  }

  delay(100);  // Small delay for stability
}

void StopAndRealign() {
  // First, stop the robot completely
  Stop();

  // Try realigning a few times, if still off-track, stop completely
  for (int i = 0; i < 3; i++) {
    realign();
    
    // After realigning, check if it's back on the track
    int leftIR = analogRead(irLeft);
    int rightIR = analogRead(irRight);
    
    // If back on track, break out of the loop and resume operation
    if (leftIR > irThreshold || rightIR > irThreshold) {
      isStopped = false;  // Clear stopped flag and resume
      return;  // Exit function if track is found
    }
  }
  
  // If still off-track after 3 attempts, stop completely and set the flag
  isStopped = true;
  Stop();
}

void realign() {
  // Move backward and check if back on track
  moveBackward();
  delay(300);  // Move backward for 300ms

  // Check if back on track after moving backward
  int leftIR = analogRead(irLeft);
  int rightIR = analogRead(irRight);

  if (leftIR > irThreshold || rightIR > irThreshold) {
    // If either sensor detects black, we're back on track
    return;  // Exit the realign function immediately
  }

  // If not back on track, move forward to try to realign
  moveForward();
  delay(300);  // Move forward for 300ms

  // Check again if back on track after moving forward
  leftIR = analogRead(irLeft);
  rightIR = analogRead(irRight);

  if (leftIR > irThreshold || rightIR > irThreshold) {
    // If either sensor detects black, we're back on track
    return;  // Exit the realign function
  }
}


void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward() {
  // Move all motors forward at MOTOR_SPEED
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveBackward() {
  // Move all motors backward at MOTOR_SPEED
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void turnLeft() {
  motor1.setSpeed(MOTOR_SPEED - 20); // Slow down slightly during the turn
  motor2.setSpeed(MOTOR_SPEED - 20);
  motor3.setSpeed(MOTOR_SPEED - 20);
  motor4.setSpeed(MOTOR_SPEED - 20);
  
  // Start turning left
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  // Keep turning left until the right sensor detects black
  while (analogRead(irRight) <= irThreshold) {
    // Keep turning until the right IR detects the line again (black)
  }

  motor1.setSpeed(MOTOR_SPEED);
  motor2.setSpeed(MOTOR_SPEED);
  motor3.setSpeed(MOTOR_SPEED);
  motor4.setSpeed(MOTOR_SPEED);

  moveForward();  // Stabilize by moving forward after turning
  delay(100);
}

void turnRight() {
  motor1.setSpeed(MOTOR_SPEED - 20); // Slow down slightly during the turn
  motor2.setSpeed(MOTOR_SPEED - 20);
  motor3.setSpeed(MOTOR_SPEED - 20);
  motor4.setSpeed(MOTOR_SPEED - 20);
  
  // Start turning right
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  // Keep turning right until the left sensor detects black
  while (analogRead(irLeft) <= irThreshold) {
    // Keep turning until the left IR detects the line again (black)
  }
  

  motor1.setSpeed(MOTOR_SPEED);
  motor2.setSpeed(MOTOR_SPEED);
  motor3.setSpeed(MOTOR_SPEED);
  motor4.setSpeed(MOTOR_SPEED);

  moveForward();  // Stabilize by moving forward after turning
  delay(100);
}
