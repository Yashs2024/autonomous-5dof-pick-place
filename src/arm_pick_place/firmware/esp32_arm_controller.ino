/*
 * 5-DOF Robotic Arm + Gripper Serial Firmware (Arduino Uno / Mega / ESP32)
 * Receives packet format: <S1,S2,S3,S4,S5,S6>\n at 115200 baud from ROS 2
 * servo_serial_bridge node and drives 6 PWM servos smoothly.
 */
#include <Servo.h>  // Use <ESP32Servo.h> if compiling for ESP32

const int NUM_SERVOS = 6;
const int SERVO_PINS[NUM_SERVOS] = {3, 5, 6, 9, 10, 11}; // PWM pins
Servo servos[NUM_SERVOS];

String inputBuffer = "";
bool receiving = false;

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(90); // Neutral 90-deg startup home pose
  }
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '<') {
      inputBuffer = "";
      receiving = true;
    } else if (c == '>' && receiving) {
      receiving = false;
      parseAndMoveServos(inputBuffer);
    } else if (receiving) {
      inputBuffer += c;
    }
  }
}

void parseAndMoveServos(String data) {
  int angles[NUM_SERVOS];
  int idx = 0;
  int lastComma = -1;

  for (int i = 0; i <= data.length() && idx < NUM_SERVOS; i++) {
    if (i == data.length() || data.charAt(i) == ',') {
      String valStr = data.substring(lastComma + 1, i);
      angles[idx] = constrain(valStr.toInt(), 0, 180);
      servos[idx].write(angles[idx]);
      idx++;
      lastComma = i;
    }
  }
}
