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
int leftIR;
int rightIR;
boolean object = false;

// IR sensor thresholds
int irThreshold = 500;  // Adjust based on your environment

void setup() {
  Serial.begin(9600);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  servo.attach(9);
  servo.write(90); // Center the servo

  // Set the speed for all motors
  motor1.setSpeed(MOTOR_SPEED);
  motor2.setSpeed(MOTOR_SPEED);
  motor3.setSpeed(MOTOR_SPEED);
  motor4.setSpeed(MOTOR_SPEED);
}

void loop() {
  // Read IR sensor values as analog
  leftIR = analogRead(irLeft);
  rightIR = analogRead(irRight);

  // Print the IR sensor values for debugging
  Serial.print("Left IR: ");
  Serial.print(leftIR);
  Serial.print(" - Right IR: ");
  Serial.println(rightIR);

  // Line-following logic
  if (leftIR > irThreshold && rightIR > irThreshold) {
    objectAvoid();  // Check for obstacles
  }
  else if (leftIR > irThreshold && rightIR <= irThreshold) {
    objectAvoid();
    Serial.println("TL");
    turnLeft();
  }
  else if (leftIR <= irThreshold && rightIR > irThreshold) {
    objectAvoid();
    Serial.println("TR");
    turnRight();
  }
  else if (leftIR <= irThreshold && rightIR <= irThreshold) {
    Stop();
  }

  delay(100);  // Small delay for stability
}

void objectAvoid() {
  distance = getDistance();  // Get distance from ultrasonic sensor
  
  Serial.print("Object distance: ");
  Serial.println(distance);  // Log the distance

  if (distance <= 15) {
    Stop();  // Stop when an obstacle is detected
    Serial.println("Obstacle detected! Stopping...");

    // Look left and right, then choose the direction with more space
    lookLeft();
    lookRight();

    Serial.print("Left Distance: ");
    Serial.println(leftDistance);
    Serial.print("Right Distance: ");
    Serial.println(rightDistance);
    
    delay(100);

    if (rightDistance <= leftDistance) {
      Serial.println("Turning left to avoid obstacle.");
      object = true;
      avoidObstacle();  // Turn left, move forward, then turn right
    } else {
      Serial.println("Turning right to avoid obstacle.");
      object = false;
      avoidObstacle();  // Turn right, move forward, then turn left
    }

    delay(100);  // Small delay after the turn
  } else {
    Serial.println("No obstacle, moving forward...");
    moveForward();  // No obstacle, keep moving forward
  }
}

int getDistance() {
  delay(50);  // Short delay before taking a reading
  int cm = sonar.ping_cm();  // Get distance in centimeters
  if (cm == 0) {
    cm = 100;  // Return a large value if no object is detected
  }
  return cm;
}

int lookLeft() {
  servo.write(150);  // Turn the servo to look left
  delay(500);  // Allow time for the servo to move and measure distance
  leftDistance = getDistance();  // Get distance on the left
  servo.write(90);  // Return the servo to center
  return leftDistance;
}

int lookRight() {
  servo.write(30);  // Turn the servo to look right
  delay(500);  // Allow time for the servo to move and measure distance
  rightDistance = getDistance();  // Get distance on the right
  servo.write(90);  // Return the servo to center
  return rightDistance;
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
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

// Avoidance function: Move far enough outside the track to bypass the obstacle
void avoidObstacle() {
  if (object == true) {
    // Turn left to go outside the track
    Serial.println("Turning left to avoid obstacle.");
    moveLeft();
    delay(1000);  // Move left for 1000ms to go outside the track

    // Move forward outside the track to bypass the obstacle
    Serial.println("Moving forward to bypass the obstacle.");
    moveForward();
    delay(1500);  // Move forward for 1500ms to pass the obstacle

    // Turn right to return to the track
    Serial.println("Turning right to return to track.");
    moveRight();
    delay(1000);  // Move right for 1000ms to return to the track

    moveForward();  // Continue moving forward after avoiding the obstacle
  } else {
    // Turn right to go outside the track
    Serial.println("Turning right to avoid obstacle.");
    moveRight();
    delay(1000);  // Move right for 1000ms to go outside the track

    // Move forward outside the track to bypass the obstacle
    Serial.println("Moving forward to bypass the obstacle.");
    moveForward();
    delay(1500);  // Move forward for 1500ms to pass the obstacle

    // Turn left to return to the track
    Serial.println("Turning left to return to track.");
    moveLeft();
    delay(1000);  // Move left for 1000ms to return to the track

    moveForward();  // Continue moving forward after avoiding the obstacle
  }
}

// Continuous turning logic for sharp turns (used for normal line-following)
void turnLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  while (analogRead(irRight) <= irThreshold) {
    // Keep turning until the right IR detects the line again (black)
  }
  moveForward();  // Stabilize by moving forward after turning
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);

  while (analogRead(irLeft) <= irThreshold) {
    // Keep turning until the left IR detects the line again (black)
  }
  moveForward();  // Stabilize by moving forward after turning
}

void moveRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void moveLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}
