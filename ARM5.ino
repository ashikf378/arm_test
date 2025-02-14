#include <math.h>
#include <Servo.h>
#include <string.h>

// Arm dimensions in centimeters
const float a1 = 6.5, a2 = 10.5, a3 = 15.0;
const int ANGLE_CORRECTIONS[] = {-3,2, -10};
Servo servo1, servo2, servo3, servo4;

// Function to smoothly move servos
void smoothMove(Servo &servo, int targetAngle, int stepDelay = 15) {
  int currentAngle = servo.read();
  while (currentAngle != targetAngle) {
    currentAngle += (currentAngle < targetAngle) ? 1 : -1;
    servo.write(currentAngle);
    delay(stepDelay);
  }
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

void openGripper() {
  smoothMove(servo4, 90); // Use smoothMove for gripper too
}

void closeGripper() {
  smoothMove(servo4, 0);
}

// Move arm to specific coordinates
void moveToPosition(float x, float y, float z) {
  float *angles = armAngles(x, y, z);

  if (angles == nullptr) {
    Serial.println("Position unreachable.");
    return;
  }

  openGripper(); // Open gripper before moving
  smoothMove(servo1, mapAngle(angles[0] + ANGLE_CORRECTIONS[0], 0, 180, 0, 180));
  smoothMove(servo2, mapAngle(angles[1] + ANGLE_CORRECTIONS[1], 0, 180, 0, 172));
  smoothMove(servo3, mapAngle(angles[2] + ANGLE_CORRECTIONS[2], 0, 180, 0, 172));
  
  closeGripper(); // Close gripper after reaching position
  delay(1000); // Hold object
}

// Setup function
void setup() {
  servo1.attach(2);
  servo2.attach(3);
  servo3.attach(4);
  servo4.attach(5);
  closeGripper(); // Start with closed gripper
  Serial.begin(9600);
  Serial.println("Enter coordinates as x,y:");
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

    if (radius >= 5.65 && radius < 7.90) {
      moveToPosition(x, y, -2.2);
    }
    else if (radius >= 8.0 && radius < 10.50) {
      moveToPosition(x, y, -1.8);
    } else if (radius >= 10.63 && radius < 14.50) {
      moveToPosition(x, y, -1.1);
    } else {
      Serial.println("Coordinates out of range.");
    }
    
    // Return to home position after pickup
    smoothMove(servo1, 90);
    smoothMove(servo2, 90);
    smoothMove(servo3, 90);
    openGripper();
  }
}
