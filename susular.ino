/*
   Vana 01.
*/

#include "LowPower.h"

const int MOTOR_IN1 = 8;
const int MOTOR_IN2 = 9;
const int ENCODER_PIN = 2;

const int ENCODER_PERIOD = 16;


void setup() {
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);

  stopMotor();
  Serial.begin(9600);
}

void loop() {
  Serial.println(F("watering..."));

  // open the valve
  turnMotorLeft(1.5);
  stopMotor();

  // keep the valve open and let the plants get some water
  powerDownFor(1);
  
  // close the valve
  turnMotorRight(1.5);
  stopMotor();
  
  // save battery
  powerDownFor(2);
}

// shutdown all units for given seconds.
void powerDownFor(unsigned long seconds) {
  // find the iteration count to sleep for SLEEP_8S
  int times = seconds / 8;

  for (int i = 0; i < times; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }

  int remainder = seconds % 8;
  for (int i = 0; i < remainder; i++) {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  }
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void turnMotorLeft(float turns) {
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  waitForTurn(turns);
}

void turnMotorRight(float turns) {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  waitForTurn(turns);
}

void waitForTurn(float turns) {
  int encoderPulse = 0;
  // assume last reading was off
  bool lastState = 1;

  do {
    bool state = digitalRead(ENCODER_PIN);

    if (state != lastState) {
      lastState = state;
      if (state == 0) {
        encoderPulse++;
      }
      lastState = state;
    }
  } while (encoderPulse < (turns * ENCODER_PERIOD));
}
