#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

// Sensors and motors setup
#define TRIGGER_PIN A2
#define ECHO_PIN A3
#define max_distance 50
#define irLeft A0
#define irRight A1

Servo servo;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);
AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

int distance = 0;
int leftDistance = 0;  // Declare left distance
int rightDistance = 0;  // Declare right distance
int leftIR, rightIR;
int irThreshold = 500;  // Adjust based on your environment
boolean object = false;

void setup() {
  Serial.begin(9600);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  servo.attach(9);
  servo.write(90);  // Center servo

  motor1.setSpeed(120);
  motor2.setSpeed(120);
  motor3.setSpeed(120);
  motor4.setSpeed(120);
}

void loop() {
  // Read IR sensor values as analog
  leftIR = analogRead(irLeft);
  rightIR = analogRead(irRight);

  Serial.print("Left IR: ");
  Serial.print(leftIR);
  Serial.print(" - Right IR: ");
  Serial.println(rightIR);

  // Adjust logic based on analog values instead of digital
  if (leftIR > irThreshold && rightIR > irThreshold) {
    objectAvoid();  // Both sensors detect the line
  } else if (leftIR > irThreshold && rightIR <= irThreshold) {
    objectAvoid();
    Serial.println("TL");
    moveLeft();  // Left detects black, turn left
  } else if (leftIR <= irThreshold && rightIR > irThreshold) {
    objectAvoid();
    Serial.println("TR");
    moveRight();  // Right detects black, turn right
  } else {
    Stop();  // Both sensors off-track
  }
}

void objectAvoid() {
  distance = getDistance();
  if (distance <= 15) {
    Stop();
    Serial.println("Obstacle detected! Stopping...");
    lookLeft();
    lookRight();
    delay(100);
    if (rightDistance <= leftDistance) {
      object = true;  // Set object to true for left turn
      turn();
      Serial.println("moveLeft");
    } else {
      object = false;  // Set object to false for right turn
      turn();
      Serial.println("moveRight");
    }
  } else {
    moveForward();
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 100;
  }
  return cm;
}

int lookLeft() {
  servo.write(150);
  delay(500);
  leftDistance = getDistance();
  servo.write(90);
  return leftDistance;
}

int lookRight() {
  servo.write(30);
  delay(500);
  rightDistance = getDistance();
  servo.write(90);
  return rightDistance;
}

void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void turn() {
  if (object == false) {
    Serial.println("Turning Right");
    moveLeft();
    delay(700);
    moveForward();
    delay(800);
    moveRight();
    delay(700);
    realign();  // Try to realign after turning
  } else {
    Serial.println("Turning Left");
    moveRight();
    delay(700);
    moveForward();
    delay(800);
    moveLeft();
    delay(700);
    realign();  // Try to realign after turning
  }
}

void realign() {
  // Keep moving forward until the IR sensors detect the line again
  while (analogRead(irLeft) <= irThreshold && analogRead(irRight) <= irThreshold) {
    moveForward();
    delay(100);  // Small forward movement to realign
  }
  Serial.println("Back on track.");
}

void moveLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}
