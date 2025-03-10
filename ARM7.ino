#include <math.h>
#include <Servo.h>
#include <string.h>
// Arm dimensions in centimeters (verify physically)
const float a1 = 6.5, a2 = 10.5, a3 = 15.0;
const int ANGLE_CORRECTIONS[] = {-10, 5, -3};  // Updated elbow correction
Servo servo1, servo2, servo3, servo4;
// Servo control variables
struct ServoState {
  int currentAngle;
  int targetAngle;
  unsigned long lastMoveTime;
};
ServoState s1 = {0, 0, 0};
ServoState s2 = {0, 0, 0};
ServoState s3 = {0, 0, 0};
ServoState s4 = {0, 0, 0};

void updateServo(Servo &servo, ServoState &state, int target, int stepDelay = 90) {
  state.targetAngle = target;
  if (state.currentAngle == state.targetAngle) return;

  unsigned long now = millis();
  if (now - state.lastMoveTime >= stepDelay) {
    // Easing effect for smoother movement
    int step = 1;
    int diff = abs(state.targetAngle - state.currentAngle);
    
    if (diff > 30) step = 3;
    else if (diff > 15) step = 2;
    
    state.currentAngle += (state.targetAngle > state.currentAngle) ? step : -step;
    state.currentAngle = constrain(state.currentAngle, 0, 180);
    servo.write(state.currentAngle);
    state.lastMoveTime = now;
  }
}

void smoothMove(Servo &servo, ServoState &state, int target) {
  while (state.currentAngle != target) {
    updateServo(servo, state, target);
    delay(1); // Allow time for servo movement
  }
}



int mapAngle(float x, float in_min, float in_max, float out_min, float out_max) {
  return constrain((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
}
String getCoordinatePart(String s, bool isFirst) {
    int commaIndex = s.indexOf(',');
    return isFirst ? s.substring(0, commaIndex) : s.substring(commaIndex + 1);
}

float* armAngles(float x, float y, float z) {
  static float theta[4];

  // Base angle calculation
  float theta1 = atan2(y, x) * 180.0 / PI;
  
  // Horizontal and 3D distances
  float r = hypot(x, y);
  float d = hypot(r, z - a1);

  // Enhanced reachability check
  if (d > (a2 + a3) || d < fabs(a2 - a3)) {
    Serial.println("Target out of reach.");
    return nullptr;
  }

  // Shoulder angle calculations
  float phi = atan2(z - a1, r);
  float cos_theta2 = (a2*a2 + d*d - a3*a3) / (2*a2*d);
  float theta2 = (phi + acos(cos_theta2)) * 180.0 / PI;

  // Corrected elbow angle calculation
  float cos_theta3 = (a2*a2 + a3*a3 - d*d) / (2*a2*a3);
  float theta3 = (180.0 - acos(cos_theta3) * 180.0 / PI);  // Supplementary angle

  theta[0] = 90 + theta1;   // Base
  theta[1] = theta2;        // Shoulder
  theta[2] =  theta3;   // Elbow

  Serial.print("Calculated angles (Base/Shoulder/Elbow): ");
  Serial.print(theta[0]); Serial.print(", ");
  Serial.print(theta[1]); Serial.print(", ");
  Serial.println(theta[2]);
  
  return theta;
}
void openGripper() {
  smoothMove(servo4, s4, 60);
}

void closeGripper() {
  smoothMove(servo4, s4, 85);
}


void moveToPosition(float x, float y, float z) {
  float *angles = armAngles(x, y, z);
  if (angles == nullptr) return;

  int targets[3] = {
    mapAngle(angles[0] + ANGLE_CORRECTIONS[0], 0, 180, 0, 180),
    mapAngle(angles[1] + ANGLE_CORRECTIONS[1], 0, 180, 0, 180),
    mapAngle(angles[2] + ANGLE_CORRECTIONS[2], 0, 180, 0, 180)
  };

  Serial.print("Target angles: ");
  Serial.print(targets[0]); Serial.print(", ");
  Serial.print(targets[1]); Serial.print(", ");
  Serial.println(targets[2]);

  openGripper();
  
  // Set all targets first
  s1.targetAngle = targets[0];
  s2.targetAngle = targets[1];
  s3.targetAngle = targets[2];

  // Smooth movement for all joints simultaneously
  while (s1.currentAngle != s1.targetAngle || 
         s2.currentAngle != s2.targetAngle || 
         s3.currentAngle != s3.targetAngle) {
    updateServo(servo1, s1, s1.targetAngle);
    updateServo(servo2, s2, s2.targetAngle);
    updateServo(servo3, s3, s3.targetAngle);
    delay(1);
  }

  closeGripper();
}

void setup() {
  servo1.attach(10);
  servo2.attach(9);
  servo3.attach(8);
  servo4.attach(5);
  
  // Initialize positions
  s1.currentAngle = 85 + ANGLE_CORRECTIONS[0];
  s2.currentAngle = 130 + ANGLE_CORRECTIONS[1];
  s3.currentAngle = 110 + ANGLE_CORRECTIONS[2];
  
  servo1.write(s1.currentAngle);
  servo2.write(s2.currentAngle);
  servo3.write(s3.currentAngle);
  closeGripper();

  Serial.begin(9600);
  Serial.println("Enter coordinates as x,y:");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(input);
    
    String xStr = getCoordinatePart(input, true);
    String yStr = getCoordinatePart(input, false);

    float x = xStr.toFloat();
    float y = yStr.toFloat();
    Serial.print("Parsed coordinates (X/Y): ");
    Serial.print(x);
    Serial.print(", ");
    Serial.println(y);

    float radius = sqrt(x * x + y * y);
    Serial.print("Radius: ");
    Serial.println(radius);

    if (radius >= 5.65 && radius < 7.90) {
      moveToPosition(x, y, 1.5);
    }
    else if (radius >= 8.0 && radius < 10.50) {
      moveToPosition(x, y, 1.3);
    } else if (radius >= 10.63 && radius < 14.50) {
      moveToPosition(x, y, 1.0);
    } else {
      Serial.println("Coordinates out of range.");
    }
        s1.targetAngle = 85;
    s2.targetAngle = 130;
    s3.targetAngle = 110;
    
    while (s1.currentAngle != s1.targetAngle || 
           s2.currentAngle != s2.targetAngle || 
           s3.currentAngle != s3.targetAngle) {
      updateServo(servo1, s1, s1.targetAngle);
      updateServo(servo2, s2, s2.targetAngle);
      updateServo(servo3, s3, s3.targetAngle);
      delay(1);
    }
    openGripper();

  }
}
