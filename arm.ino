#include <math.h>
#include <Servo.h>
#include <string.h>

// Arm dimensions in centimeters
const float a1 = 6.5, a2 = 10.5, a3 = 15;
Servo servo1, servo2, servo3, servo4;
float coordinates[2];
unsigned long lastUpdateTime = 0;
const int updateInterval = 20; // Servo update interval in milliseconds

String first(String s)
{
    String ans="";
    for(int i=0; i<s.length(); i++)
    {
        if(s[i]==',') break;
        ans = ans + s[i];
    }
    return ans;
}
String second(String s)
{
    String ans="";
    for(int i=s.length()-1; i>=0; i--)
    {
        if(s[i]==',') break;
        ans=s[i]+ans;
    }
    return ans;
}




void smoothMove(Servo &servo, int targetAngle, int stepDelay = 15) {
  int currentAngle = servo.read();
  while (currentAngle != targetAngle) {
    currentAngle += (currentAngle < targetAngle) ? 1 : -1;
    servo.write(currentAngle);
    delay(stepDelay);
  }
}

//String getCoordinatePart(String s, bool isFirst) {
//    int commaIndex = s.indexOf(',');
//    return isFirst ? s.substring(0, commaIndex) : s.substring(commaIndex + 1);
//}

float* armAngles(float x, float y, float z) {
float theta1 = atan2(y, x) * 180.0 / PI; // Convert from radians to degrees

      // Project the target point onto the 2D plane of the arm
      float r = sqrt(x * x + y * y); // Distance to target in the horizontal plane
      float d = sqrt(r * r + (z - a1) * (z - a1)); // Distance from shoulder joint to target point

  // Check if the target is reachable
  if (d > (a2 + a3)) {
    Serial.println("Target out of reach.");
    return;
  }

  // Calculate the shoulder angle (theta2)
  float angle_a = atan2(z - a1, r);
  float angle_b = acos((a2 * a2 + d * d - a3 * a3) / (2 * a2 * d));
  float theta2 = (angle_a + angle_b) * 180.0 / PI; // Convert to degrees

  // Calculate the elbow angle (theta3)
   float theta3 = acos((a2 * a2 + a3 * a3 - d * d) / (2 * a2 * a3)) * 180.0 / PI;
    
    static float theta[4];
   
    theta[0]= 100+theta1;
    theta[1]= 90-theta2;
    theta[2]= 190-theta3;

       Serial.println(theta[0]);
    Serial.println(theta[1]);
    Serial.println(theta[2]);
    return theta;
}

void setup() {
    servo1.attach(2);
    servo2.attach(3);
    servo3.attach(4);
    servo4.attach(5);

    Serial.begin(9600);
    Serial.println("Enter coordinates as x,y:");
}


void moveToPosition(float x, float y, float z) {
  float *angles = armAngles(x, y, z);
  smoothMove(servo1, angles[0]);
  smoothMove(servo2, angles[1]);
  smoothMove(servo3, angles[2]);
    Serial.println("for Smooth move");
    Serial.println(angles[0]);
    Serial.println(angles[1]);
    Serial.println(angles[2]);

}

void loop() {
    if (Serial.available() > 0) {
        lastDelay:
        String input = Serial.readStringUntil('\n');
        String x = first(input);
        String y = second(input);
        float fx =atof(x.c_str());
        float fy =atof(y.c_str());
        coordinates[0]= fx;
        coordinates[1]= fy;

    float radius = sqrt(fx*fx + fy*fy);
    Serial.println(fx);
    Serial.println(fy);
    Serial.println(radius);
        if(fx<=0 && fy <=0){
       goto lastDelay;
        }

        else if (radius >= 7.07 && radius < 9.92) {
            moveToPosition(coordinates[0], coordinates[1], -3);

         } else if (radius >= 10.63  && radius < 14.50) {

            moveToPosition(coordinates[0], coordinates[1], 3);

        }
    }
}
