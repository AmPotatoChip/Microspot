#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create the PWM driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Define the number of servos
#define NUM_SERVOS 6 //the spot has 12 but 6 is being used to test out motion

// Array to hold the servo angles
int servo_angles[NUM_SERVOS];

// Define the I2C address for the PCA9685
//#define PCA9685_ADDR 0x40
const int MIN_PULSE = 102;  // ~1ms
const int MAX_PULSE = 612;

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Initialize I2C communication
  Wire.begin();

  // Initialize the PCA9685 PWM driver
  pwm.begin();

  // Set PWM frequency to 60 Hz (standard for servos)
  pwm.setPWMFreq(60);

  Serial.println("ESP32 PCA9685 Servo Controller");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');

    if (input.startsWith("ANG:")) {
      input.remove(0, 4); // Remove "ANG:"
      Serial.print("Received: ");
      Serial.println(input);

      int angleIndex = 0;
      int start = 0;
      while (angleIndex < NUM_SERVOS && start < input.length()) {
        int commaIndex = input.indexOf(',', start);
        if (commaIndex == -1) commaIndex = input.length();

        String angleStr = input.substring(start, commaIndex);
        int angle = angleStr.toInt();  // Or .toFloat() if you're using floats

        int pulse = map(angle, 0, 180, MIN_PULSE, MAX_PULSE);  // Map to microsecond range
        pwm.setPWM(angleIndex, 0, pulse);

        Serial.print("Servo ");
        Serial.print(angleIndex);
        Serial.print(" = ");
        Serial.print(angle);
        Serial.print("°, PWM: ");
        Serial.println(pulse);

        angleIndex++;
        start = commaIndex + 1;
      }
    }
  }
}
