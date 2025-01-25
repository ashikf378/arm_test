#include <math.h>
#include <Servo.h>
#include <string.h>

// Arm dimensions in centimeters
const float a1 = 6.5, a2 = 10.5, a3 = 15;
const int ANGLE_CORRECTIONS[] = {-5, 0, -8};
Servo servo1, servo2, servo3, servo4;

// Structure to track servo movement
struct ServoMove {
  Servo *servo;
  int targetAngle;
  int stepDelay;
  unsigned long lastMoveTime;
  int currentAngle;
  bool moving;
};

ServoMove servoMoves[3];

void initializeServoMoves() {
  servoMoves[0] = {&servo1, 0, 15, 0, 0, false};
  servoMoves[1] = {&servo2, 0, 15, 0, 0, false};
  servoMoves[2] = {&servo3, 0, 15, 0, 0, false};
}

void smoothMove(ServoMove &servoMove) {
  unsigned long currentTime = millis();
  if (!servoMove.moving) return;

  if (currentTime - servoMove.lastMoveTime >= servoMove.stepDelay) {
    servoMove.lastMoveTime = currentTime;
    if (servoMove.currentAngle != servoMove.targetAngle) {
      servoMove.currentAngle += (servoMove.currentAngle < servoMove.targetAngle) ? 1 : -1;
      servoMove.servo->write(servoMove.currentAngle);
    } else {
      servoMove.moving = false; // Stop moving when target is reached
    }
  }
}

void startSmoothMove(Servo &servo, int targetAngle, int stepDelay, ServoMove &servoMove) {
  servoMove.servo = &servo;
  servoMove.targetAngle = targetAngle;
  servoMove.stepDelay = stepDelay;
  servoMove.currentAngle = servo.read();
  servoMove.moving = true;
  servoMove.lastMoveTime = millis();
}

// Map angle function
int mapAngle(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Split coordinate string into x and y components
String getCoordinatePart(String s, bool isFirst) {
  int commaIndex = s.indexOf(',');
  return isFirst ? s.substring(0, commaIndex) : s.substring(commaIndex + 1);
}

// Calculate arm angles using inverse kinematics
float* armAngles(float x, float y, float z) {
  static float theta[4]; // Static to persist data after function exits

  float theta1 = atan2(y, x) * 180.0 / PI; // Base rotation angle
  float r = sqrt(x * x + y * y); // Horizontal distance
  float d = sqrt(r * r + (z - a1) * (z - a1)); // 3D distance

  if (d > (a2 + a3)) {
    Serial.println("Target out of reach.");
    return nullptr; // Target not reachable
  }

  float angle_a = atan2(z - a1, r);
  float angle_b = acos((a2 * a2 + d * d - a3 * a3) / (2 * a2 * d));
  float theta2 = (angle_a + angle_b) * 180.0 / PI;

  float theta3 = acos((a2 * a2 + a3 * a3 - d * d) / (2 * a2 * a3)) * 180.0 / PI;

  theta[0] = 90 + theta1;
  theta[1] = 90 + theta2;
  theta[2] = theta3;

  return theta;
}

// Move arm to specific coordinates
void moveToPosition(float x, float y, float z) {
  float *angles = armAngles(x, y, z);

  if (angles == nullptr) {
    Serial.println("Position unreachable.");
    return;
  }

  startSmoothMove(servo1, mapAngle(angles[0] + ANGLE_CORRECTIONS[0], 0, 180, 0, 180), 15, servoMoves[0]);
  startSmoothMove(servo2, mapAngle(angles[1] + ANGLE_CORRECTIONS[1], 0, 180, 0, 160), 15, servoMoves[1]);
  startSmoothMove(servo3, mapAngle(angles[2] + ANGLE_CORRECTIONS[2], 0, 180, 0, 156), 15, servoMoves[2]);

  Serial.println(angles[0]);
  Serial.println(angles[1]);
  Serial.println(angles[2]);
}

// Setup function
void setup() {
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);

  Serial.begin(9600);
  Serial.println("Enter coordinates as x,y:");
  initializeServoMoves();
}

// Main loop
void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    String xStr = getCoordinatePart(input, true);
    String yStr = getCoordinatePart(input, false);

    float x = xStr.toFloat();
    float y = yStr.toFloat();

    float radius = sqrt(x * x + y * y);
    Serial.print("Radius: ");
    Serial.println(radius);

    if (radius >= 5.65 && radius < 7.08) {
      moveToPosition(x, y, -2.4);
    } else if (radius >= 8.45 && radius < 9.92) {
      moveToPosition(x, y, -2.2);
    } else if (radius >= 10.63 && radius < 14.50) {
      moveToPosition(x, y, -1.4);
    } else {
      Serial.println("Coordinates out of range.");
    }
  }

  // Continuously update servo movements
  for (int i = 0; i < 3; i++) {
    smoothMove(servoMoves[i]);
  }
}
