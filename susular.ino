/*
   Vana 01.

   Pinler:
    D2 - IR cikisi (enkoder sensoru)

    D4 - Motor surucu girdi (M_IN1)
    D5 - Motor surucu girdi (M_IN2)

    D8 - mesgul indikator LEDi
    D9 - cevre birimleri guc kapisi (MOSFET gate, 0 aktif)

    D10 - SD kart SS (slave select)
    D11 - SD kart MOSI
    D12 - SD kart MISO
    D13 - SD kart SCK

    A4  - I2C SDA (RTC)
    A5  - I2C SCL (RTC)
*/

#include <SPI.h>
#include "LowPower.h"
#include "RTClib.h"
#include "SdFat.h"

// Pins
const int ENCODER = 2;
const int MOTOR_IN1 = 4;
const int MOTOR_IN2 = 5;
const int LED = 8;
const int POWER = 9;
const int SD_CHIPSELECT = 10;

// Constants
const int ENCODER_PERIOD = 16;
const char * fileName = "KAYIT.TXT";

// Valve Status
const byte VALVE_IDLE = 0;
const byte VALVE_OPENING = 1;
const byte VALVE_OPEN = 2;
const byte VALVE_CLOSING = 3;
const byte VALVE_CLOSED = 4;

RTC_DS3231 RTC;

/*
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
*/
const int dayCount = 60;
DateTime timeToWater[dayCount] = {
  DateTime(2021, 1, 30, 22, 0, 0),
  DateTime(2021, 1, 30, 22, 1, 0),
  DateTime(2021, 1, 30, 22, 2, 0),
  DateTime(2021, 1, 30, 22, 3, 0),
  DateTime(2021, 1, 30, 22, 4, 0),
  DateTime(2021, 1, 30, 22, 5, 0),
  DateTime(2021, 1, 30, 22, 6, 0),
  DateTime(2021, 1, 30, 22, 7, 0),
  DateTime(2021, 1, 30, 22, 8, 0),
  DateTime(2021, 1, 30, 22, 9, 0),
  DateTime(2021, 1, 30, 22, 10, 0),
  DateTime(2021, 1, 30, 22, 11, 0),
  DateTime(2021, 1, 30, 22, 12, 0),
  DateTime(2021, 1, 30, 22, 13, 0),
  DateTime(2021, 1, 30, 22, 14, 0),
  DateTime(2021, 1, 30, 22, 15, 0),
  DateTime(2021, 1, 30, 22, 16, 0),
  DateTime(2021, 1, 30, 22, 17, 0),
  DateTime(2021, 1, 30, 22, 18, 0),
  DateTime(2021, 1, 30, 22, 19, 0),
  DateTime(2021, 1, 30, 22, 20, 0),
  DateTime(2021, 1, 30, 22, 21, 0),
  DateTime(2021, 1, 30, 22, 22, 0),
  DateTime(2021, 1, 30, 22, 23, 0),
  DateTime(2021, 1, 30, 22, 24, 0),
  DateTime(2021, 1, 30, 22, 25, 0),
  DateTime(2021, 1, 30, 22, 26, 0),
  DateTime(2021, 1, 30, 22, 27, 0),
  DateTime(2021, 1, 30, 22, 28, 0),
  DateTime(2021, 1, 30, 22, 29, 0),
  DateTime(2021, 1, 30, 22, 30, 0),
  DateTime(2021, 1, 30, 22, 31, 0),
  DateTime(2021, 1, 30, 22, 32, 0),
  DateTime(2021, 1, 30, 22, 33, 0),
  DateTime(2021, 1, 30, 22, 34, 0),
  DateTime(2021, 1, 30, 22, 35, 0),
  DateTime(2021, 1, 30, 22, 36, 0),
  DateTime(2021, 1, 30, 22, 37, 0),
  DateTime(2021, 1, 30, 22, 38, 0),
  DateTime(2021, 1, 30, 22, 39, 0),
  DateTime(2021, 1, 30, 22, 40, 0),
  DateTime(2021, 1, 30, 22, 41, 0),
  DateTime(2021, 1, 30, 22, 42, 0),
  DateTime(2021, 1, 30, 22, 43, 0),
  DateTime(2021, 1, 30, 22, 44, 0),
  DateTime(2021, 1, 30, 22, 45, 0),
  DateTime(2021, 1, 30, 22, 46, 0),
  DateTime(2021, 1, 30, 22, 47, 0),
  DateTime(2021, 1, 30, 22, 48, 0),
  DateTime(2021, 1, 30, 22, 49, 0),
  DateTime(2021, 1, 30, 22, 50, 0),
  DateTime(2021, 1, 30, 22, 51, 0),
  DateTime(2021, 1, 30, 22, 52, 0),
  DateTime(2021, 1, 30, 22, 53, 0),
  DateTime(2021, 1, 30, 22, 54, 0),
  DateTime(2021, 1, 30, 22, 55, 0),
  DateTime(2021, 1, 30, 22, 56, 0),
  DateTime(2021, 1, 30, 22, 57, 0),
  DateTime(2021, 1, 30, 22, 58, 0),
  DateTime(2021, 1, 30, 22, 59, 0),
};

// 1 hour buffer for date comparison
//TimeSpan dateBuffer(0, 1, 0, 0); // TODO
TimeSpan dateBuffer(0, 0, 0, 10);

void setup() {
  pinMode(ENCODER, INPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(POWER, OUTPUT);

  Serial.begin(115200);

  stopMotor();

  // NOTE: enable to adjust RTC time for current time
  if (false) {
    powerOnPeripherals();
    enableRTC();
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    disableRTC();
    powerOffPeripherals();
  }
}

void loop() {
  Serial.println(F("here we go! checking the date..."));
  if (!isTimeToWater()) {
    Serial.println(F("not my time, sleeping an hour"));
    Serial.flush(); // wait for the tranmission to end before going sleep
    record(VALVE_IDLE, 0, 0);

    // sleep for an hour until next check
    // sleepFor(1 * 60 * 60);
    sleepFor(1);
    return;
  }

  Serial.println(F("watering time! opening the valve..."));

  // open the valve
  record(VALVE_OPENING, 0, 0);
  turnMotorLeft(1.5);
  stopMotor();
  record(VALVE_OPEN, 0, 0);

  // keep the valve open and let the plants get some water
  Serial.println(F("valve open. sleeping 4 hours..."));
  Serial.flush();
  // powerDownFor(4 * 60 * 60);
  sleepFor(2);

  // close the valve
  Serial.println(F("slept 4 hours. closing the valve..."));
  record(VALVE_CLOSING, 0, 0);
  turnMotorRight(1.5);
  stopMotor();
  record(VALVE_CLOSED, 0, 0);

  Serial.println(F("watering done! sleeping half a day..."));
  Serial.flush(); // wait for the tranmission to end before going sleep

  // save battery
  // powerDownFor(12 * 60 * 60UL);
  sleepFor(2);
}

// shutdown all units for given seconds.
void sleepFor(unsigned long seconds) {
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

void powerOnPeripherals() {
  pinMode(POWER, OUTPUT);
  digitalWrite(POWER, LOW); // p-channel mosfet. low -> active.
}

void powerOffPeripherals() {
  // put all digital pins into low stats
  for (byte pin = 0; pin < 14; pin++) {
    pinMode (pin, OUTPUT);
    digitalWrite (pin, LOW);
  }

  digitalWrite (POWER, HIGH);
}

void enableRTC () {
  if (!RTC.begin()) {
    Serial.println(F("rtc: not initialized"));
    return;
  }

  delay (2);
  TWBR = 72;  // 50 kHz at 8 MHz clock
}

void disableRTC () {
  // turn off i2c
  TWCR &= ~(bit(TWEN) | bit(TWIE) | bit(TWEA));

  // turn off i2c pull-ups
  digitalWrite (A4, LOW);
  digitalWrite (A5, LOW);
}

void flashLED (const byte times) {
  for (int i = 0; i < times; i++)
  {
    digitalWrite (LED, HIGH);
    delay (20);
    digitalWrite (LED, LOW);
    delay (150);
  }
}

bool isTimeToWater() {
  powerOnPeripherals();
  delay(10); // wait a few ms for rtc
  enableRTC();

  flashLED(3);
  DateTime now = RTC.now();
  disableRTC();
  powerOffPeripherals();

  printTime(now);

  Serial.println(F("iterating over watering days..."));
  for (int i = 0; i < dayCount; i++ ) {
    DateTime timeStart = timeToWater[i];
    DateTime timeEnd = timeStart + dateBuffer;

    if ((now >= timeStart) && (now <= timeEnd)) {
      Serial.print(F("watering time found: "));
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

void printTime(DateTime now) {
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}

void record(unsigned int valveStatus,
            int temperature,
            float humidity) {

  powerOnPeripherals();

  SdFat sd;

  flashLED(5);

  if (!sd.begin(SD_CHIPSELECT, SPI_HALF_SPEED)) {
    Serial.println(F("sd: failed to initialize"));
    Serial.flush();
    return;
  }

  SdFile file;
  SPI.begin();

  if (!file.open(fileName, O_CREAT | O_WRITE | O_APPEND)) {
    Serial.println(F("sd: failed to open file"));
    Serial.flush();
    return;
  }
  enableRTC();
  DateTime now = RTC.now();
  disableRTC();

  /*
     Datetime | ValveStatus | Temperature | Humidity
     Ideas:
       - System battery status
       - RTC battery status
       - Manual trigger status
  */
  char buf[30];
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
          (int) now.year(), (int) now.month(), (int) now.day(),
          (int) now.hour(), (int) now.minute(), (int) now.second());

  file.print(buf);
  file.print("\t");
  file.print(valveStatus);
  file.println();

  file.sync();
  file.close();
  delay(100);
  SPI.end();
  powerOffPeripherals();

  flashLED(2);
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
  // TODO
  delay(1000);
  return;

  int encoderPulse = 0;
  // assume last reading was off
  bool lastState = 1;

  do {
    bool state = digitalRead(ENCODER);

    if (state != lastState) {
      lastState = state;
      if (state == 0) {
        encoderPulse++;
      }
      lastState = state;
    }
  } while (encoderPulse < (turns * ENCODER_PERIOD));
}
