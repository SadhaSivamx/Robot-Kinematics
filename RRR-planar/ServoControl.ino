#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int minAngle[6] = {0, 0, 0, 0, 0, 0};
int maxAngle[6] = {180, 180, 180, 180, 180, 90};

int SERVOMIN = 150;
int SERVOMAX = 600;

int currentAngle[6] = {90, 90, 90, 90, 90, 45};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  for (int ch = 0; ch < 6; ch++)
  {
    int mid = ((minAngle[ch] + maxAngle[ch]) / 2) - 10;
    currentAngle[ch] = mid;
    setServoAngle(ch, mid);
  }

  Serial.println("✅ Interactive Servo Control Ready!");
  Serial.println("Type command like: M0:90");
  Serial.println("Servo ranges:");
  for (int i = 0; i < 6; i++) {
    Serial.print("M");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(minAngle[i]);
    Serial.print(" - ");
    Serial.println(maxAngle[i]);
  }
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("M")) {
      int servoIndex = input.substring(1, input.indexOf(':')).toInt();
      int targetAngle = input.substring(input.indexOf(':') + 1).toInt();

      if (servoIndex >= 0 && servoIndex < 6) {
        if (targetAngle >= minAngle[servoIndex] && targetAngle <= maxAngle[servoIndex]) {
          smoothMove(servoIndex, targetAngle);
          Serial.print("✅ M");
          Serial.print(servoIndex);
          Serial.print(" moved to ");
          Serial.println(targetAngle);
        } else {
          Serial.print("❌ Angle out of range for M");
          Serial.println(servoIndex);
        }
      } else {
        Serial.println("❌ Invalid motor index");
      }
    }
  }
}

void smoothMove(int channel, int target) {
  int step = (target > currentAngle[channel]) ? 1 : -1;

  for (int angle = currentAngle[channel]; angle != target; angle += step) {
    setServoAngle(channel, angle);
    delay(25);
  }

  currentAngle[channel] = target;
}

void setServoAngle(int channel, int angle) {
  int pulseLen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulseLen);
}
