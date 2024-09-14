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

  // Move forward when both sensors detect black (within the threshold)
  if (leftIR > irThreshold && rightIR > irThreshold) {
    moveForward();  // Both sensors detect black, move forward
  }
  // Turn left if left detects black and right detects white
  else if (leftIR > irThreshold && rightIR <= irThreshold) {
    turnLeft();
  }
  // Turn right if right detects black and left detects white
  else if (leftIR <= irThreshold && rightIR > irThreshold) {
    turnRight();
  }
  // Stop and attempt to realign if both detect white (off the track)
  else if (leftIR <= irThreshold && rightIR <= irThreshold) {
    StopAndRealign();
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
    
    // If back on track, break out of the loop
    if (leftIR > irThreshold || rightIR > irThreshold) {
      return;  // Exit function if track is found
    }
  }
  
  // If still off-track after 3 attempts, stop completely
  Stop();
}

void realign() {
  moveBackward();
  delay(300);  // Adjust this delay based on your track
  moveForward();
  delay(300);  // Move forward to try to get back on track
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
}
