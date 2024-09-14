#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

#define TRIGGER_PIN A2
#define ECHO_PIN A3
#define max_distance 50

#define irLeft A0
#define irRight A1

#define MOTOR_SPEED 120

Servo servo;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

int distance = 0;
int leftIR;
int rightIR;
boolean object = false;
boolean avoidingObstacle = false;  // State variable to prevent loops

int irThreshold = 500;

void setup() {
  Serial.begin(9600);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  servo.attach(9);
  servo.write(90);

  motor1.setSpeed(MOTOR_SPEED);
  motor2.setSpeed(MOTOR_SPEED);
  motor3.setSpeed(MOTOR_SPEED);
  motor4.setSpeed(MOTOR_SPEED);
}

void loop() {
  leftIR = analogRead(irLeft);
  rightIR = analogRead(irRight);

  Serial.print("Left IR: ");
  Serial.print(leftIR);
  Serial.print(" - Right IR: ");
  Serial.println(rightIR);

  if (!avoidingObstacle) {
    if (leftIR > irThreshold && rightIR > irThreshold) {
      objectAvoid();  // Check for obstacles
    } else if (leftIR > irThreshold && rightIR <= irThreshold) {
      turnLeft();
    } else if (leftIR <= irThreshold && rightIR > irThreshold) {
      turnRight();
    } else if (leftIR <= irThreshold && rightIR <= irThreshold) {
      Stop();
    }
  }

  delay(100);
}

void objectAvoid() {
  distance = getDistance();
  
  if (distance <= 15) {
    avoidingObstacle = true;  // Set the flag
    Serial.println("Obstacle detected! Stopping...");
    Stop();
    moveBackward();
    delay(500);

    turnLeft();
    delay(700);
    moveForward();
    delay(800);
    
    turnRight();
    delay(700);
    
    avoidingObstacle = false;  // Reset the flag after maneuver
  } else {
    moveForward();
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 100;  // Return large value if no object detected
  }
  return cm;
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

void turnLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}
