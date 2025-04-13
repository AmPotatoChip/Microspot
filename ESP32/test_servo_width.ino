#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Calibration constants (adjust if needed)
const int MIN_PULSE = 102;  // ~1ms
const int MAX_PULSE = 612;  // ~2ms was 512

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);  // 60Hz is typical for servos
  delay(10);

  Serial.println("Enter angle (0-180) and channel (0-15) like: 90,0");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    input.trim();  // Remove whitespace and newline
    if (input.length() == 0) return;

    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      Serial.println("Invalid format. Use: 90,0");
      return;
    }

    String angleStr = input.substring(0, commaIndex);
    String channelStr = input.substring(commaIndex + 1);

    int angle = angleStr.toInt();
    int channel = channelStr.toInt();

    if (angle < 0 || angle > 180 || channel < 0 || channel > 15) {
      Serial.println("Invalid angle or channel");
      return;
    }

    int pulse = map(angle, 0, 180, MIN_PULSE, MAX_PULSE);
    pwm.setPWM(channel, 0, pulse);

    Serial.print("Set channel ");
    Serial.print(channel);
    Serial.print(" to angle ");
    Serial.print(angle);
    Serial.print(" (PWM = ");
    Serial.print(pulse);
    Serial.println(")");
  }
}