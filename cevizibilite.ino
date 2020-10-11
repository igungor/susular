/*
   Vana 01.
*/

#include "LowPower.h"
#include "RTClib.h"

const int MOTOR_ENA = 6;
const int MOTOR_IN1 = 8;
const int MOTOR_IN2 = 9;
const int ENCODER_PIN = 2;

const int ENCODER_PERIOD = 16;
const int MOTOR_PWM = 255;

RTC_DS3231 rtc;

String daysOfTheWeek[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

const int dayCount = 41;
DateTime timeToWater[dayCount] = {
  DateTime(2021, 5, 15, 20, 0, 0),
  DateTime(2021, 5, 18, 20, 0, 0),
  DateTime(2021, 5, 21, 20, 0, 0),
  DateTime(2021, 5, 24, 20, 0, 0),
  DateTime(2021, 5, 27, 20, 0, 0),
  DateTime(2021, 5, 30, 20, 0, 0),
  DateTime(2021, 6,  2, 20, 0, 0),
  DateTime(2021, 6,  5, 20, 0, 0),
  DateTime(2021, 6,  8, 20, 0, 0),
  DateTime(2021, 6, 11, 20, 0, 0),
  DateTime(2021, 6, 14, 20, 0, 0),
  DateTime(2021, 6, 17, 20, 0, 0),
  DateTime(2021, 6, 20, 20, 0, 0),
  DateTime(2021, 6, 23, 20, 0, 0),
  DateTime(2021, 6, 26, 20, 0, 0),
  DateTime(2021, 6, 29, 20, 0, 0),
  DateTime(2021, 7,  2, 20, 0, 0),
  DateTime(2021, 7,  5, 20, 0, 0),
  DateTime(2021, 7,  8, 20, 0, 0),
  DateTime(2021, 7, 11, 20, 0, 0),
  DateTime(2021, 7, 14, 20, 0, 0),
  DateTime(2021, 7, 17, 20, 0, 0),
  DateTime(2021, 7, 20, 20, 0, 0),
  DateTime(2021, 7, 23, 20, 0, 0),
  DateTime(2021, 7, 26, 20, 0, 0),
  DateTime(2021, 7, 29, 20, 0, 0),
  DateTime(2021, 8,  1, 20, 0, 0),
  DateTime(2021, 8,  4, 20, 0, 0),
  DateTime(2021, 8,  7, 20, 0, 0),
  DateTime(2021, 8, 10, 20, 0, 0),
  DateTime(2021, 8, 13, 20, 0, 0),
  DateTime(2021, 8, 16, 20, 0, 0),
  DateTime(2021, 8, 19, 20, 0, 0),
  DateTime(2021, 8, 22, 20, 0, 0),
  DateTime(2021, 8, 25, 20, 0, 0),
  DateTime(2021, 8, 28, 20, 0, 0),
  DateTime(2021, 8, 31, 20, 0, 0),
  DateTime(2021, 9,  3, 20, 0, 0),
  DateTime(2021, 9,  6, 20, 0, 0),
  DateTime(2021, 9,  9, 20, 0, 0),
  DateTime(2021, 9, 12, 20, 0, 0),
};

// 1 hour buffer for date comparison
TimeSpan dateBuffer(0, 1, 0, 0);

void setup() {
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);

  stopMotor();
  analogWrite(MOTOR_ENA, MOTOR_PWM);
  Serial.begin(9600);

  if (!rtc.begin()) {
    Serial.println("couldn't find RTC");
    Serial.flush();
    abort();
  }

  // adjust RTC time for current time
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void loop() {
  if (!isTimeToWater()) {
    // sleep for an hour until next check
    powerDownFor(1 * 60 * 60);
    return;
  }

  Serial.println("watering...");

  // open the valve
  turnMotorLeft(1.5);
  stopMotor();

  // keep the valve open and let the plants watered
  powerDownFor(4 * 60 * 60);
  
  // close the valve
  turnMotorRight(1.5);
  stopMotor();
  
  // save battery
  powerDownFor(12 * 60 * 60);
}

// shutdown all units for given seconds.
void powerDownFor(int seconds) {
  // find the iteration count to sleep for SLEEP_8S
  int times = seconds / 8;

  for (int i = 0; i < times; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }
}

bool isTimeToWater() {
  printTime();
  DateTime now = rtc.now();

  Serial.println("iterating over watering days...");
  for (int i = 0; i < dayCount; i++ ) {
    DateTime timeStart = timeToWater[i];
    DateTime timeEnd = timeStart + dateBuffer;

    if ((now >= timeStart) && (now <= timeEnd)) {
      Serial.print("watering time found: ");
      Serial.println(dateTimeToString(timeStart));
      return true;
    }
  }

  return false;
}

String dateTimeToString(DateTime &dt) {
  char buffer[50];

  sprintf(buffer, "%d/%d/%d %02d:%02d:%02d",
          dt.year(), dt.month(), dt.day(),
          dt.hour(), dt.minute(), dt.second());

  return String(buffer);
}

void printTime() {
  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" (");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(") ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);

  Serial.print(" | Temperature: ");
  Serial.print(rtc.getTemperature());
  Serial.println(" C");
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
