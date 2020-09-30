#include "LowPower.h"

const int MOTOR_ENA = 6;
const int MOTOR_IN1 = 7;
const int MOTOR_IN2 = 8;
const int ENCODER_PIN = 2;

const int ENCODER_PERIOD = 16;
// Approximate bounce duration of IR sensor.
// Ignore noisy readings during this time period.
const int BOUNCE_DURATION = 50; // milliseconds

const int MOTOR_PWM = 255;

volatile int encoderPulse;

void setup() {
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);

  encoderPulse = 0;
  stopMotor();
  analogWrite(MOTOR_ENA, MOTOR_PWM);
  Serial.begin(9600);
}

void loop() {
  turnMotorLeft(1.5);
  delay(3000);

  stopMotor();
  delay(1000);

  turnMotorRight(1.5);
  delay(1000);

  stopMotor();
  delay(3000);

  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}

void attachEncoderInterrupt() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, FALLING);
}

void detachEncoderInterrupt() {
  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN));
}

void countPulse() {
  // debounce logic
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();

  if ((interruptTime - lastInterruptTime) > BOUNCE_DURATION) {
    encoderPulse++;
  }

  lastInterruptTime = interruptTime;
}

void stopMotor() {
  encoderPulse = 0;

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void turnMotorLeft(float turns) {
  attachEncoderInterrupt();;
  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  waitForTurn(turns);
  detachEncoderInterrupt();
}

void turnMotorRight(float turns) {
  attachEncoderInterrupt();;
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  waitForTurn(turns);
  detachEncoderInterrupt();
}

void waitForTurn(float turns) {
  while(float(encoderPulse) / ENCODER_PERIOD < turns) {
  };
}
