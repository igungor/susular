/*
   Vana 01.

   Pinler:
    D2 - IR cikisi (enkoder sensoru)

    D4 - Motor surucu girisi (M_IN1)
    D5 - Motor surucu girisi (M_IN2)

    D8 - indikator LED
    D9 - cevre birimleri guc kapisi (MOSFET gate, 0 aktif)

    D10 - SD kart SS (slave select)
    D11 - SD kart MOSI
    D12 - SD kart MISO
    D13 - SD kart SCK

    A4  - I2C SDA (RTC)
    A5  - I2C SCL (RTC)

   Hata Indikatoru:
    err1: sd-kart hatasi. LED 2 kisa flash.
    err2: rtc hatasi. LED 3 kisa flash.
    err3: dht hatasi. LED 4 kisa flash.
*/

#include <SPI.h>
#include <LowPower.h>
#include <RTClib.h>
#include <SdFat.h>
#include <DHT.h>

// Pins
const int ENCODER = 2;
const int MOTOR_IN1 = 4;
const int MOTOR_IN2 = 5;
const int DHT_PIN = 7;
const int LED = 8;
const int POWER = 9;
const int SD_CHIPSELECT = 10;

// Constants
const int ENCODER_PERIOD = 16; // Number of pulses on encoder wheel
const char * logfile = "KAYIT.CSV"; // Valve log in TSV format
const int WATERING_DURATION = 5UL;
const int WAKEUP_EVERY = 10UL;

// Valve Status
const byte VALVE_IDLE = 0;
const byte VALVE_OPENING = 1;
const byte VALVE_OPEN = 2;
const byte VALVE_CLOSING = 3;
const byte VALVE_CLOSED = 4;

// Global variables
DateTime now;
float temperature;
float humidity;

RTC_DS3231 RTC;
DHT dht(DHT_PIN, DHT11, 3);


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
DateTime schedule[dayCount] = {
  DateTime(2021, 1, 31, 15, 0, 0),
  DateTime(2021, 1, 31, 15, 1, 0),
  DateTime(2021, 1, 31, 15, 2, 0),
  DateTime(2021, 1, 31, 15, 3, 0),
  DateTime(2021, 1, 31, 15, 4, 0),
  DateTime(2021, 1, 31, 15, 5, 0),
  DateTime(2021, 1, 31, 15, 6, 0),
  DateTime(2021, 1, 31, 15, 7, 0),
  DateTime(2021, 1, 31, 15, 8, 0),
  DateTime(2021, 1, 31, 15, 9, 0),
  DateTime(2021, 1, 31, 15, 10, 0),
  DateTime(2021, 1, 31, 15, 11, 0),
  DateTime(2021, 1, 31, 15, 12, 0),
  DateTime(2021, 1, 31, 15, 13, 0),
  DateTime(2021, 1, 31, 15, 14, 0),
  DateTime(2021, 1, 31, 15, 15, 0),
  DateTime(2021, 1, 31, 15, 16, 0),
  DateTime(2021, 1, 31, 15, 17, 0),
  DateTime(2021, 1, 31, 15, 18, 0),
  DateTime(2021, 1, 31, 15, 19, 0),
  DateTime(2021, 1, 31, 15, 20, 0),
  DateTime(2021, 1, 31, 15, 21, 0),
  DateTime(2021, 1, 31, 15, 22, 0),
  DateTime(2021, 1, 31, 15, 23, 0),
  DateTime(2021, 1, 31, 15, 24, 0),
  DateTime(2021, 1, 31, 15, 25, 0),
  DateTime(2021, 1, 31, 15, 26, 0),
  DateTime(2021, 1, 31, 15, 27, 0),
  DateTime(2021, 1, 31, 15, 28, 0),
  DateTime(2021, 1, 31, 15, 29, 0),
  DateTime(2021, 1, 31, 15, 30, 0),
  DateTime(2021, 1, 31, 15, 31, 0),
  DateTime(2021, 1, 31, 15, 32, 0),
  DateTime(2021, 1, 31, 15, 33, 0),
  DateTime(2021, 1, 31, 15, 34, 0),
  DateTime(2021, 1, 31, 15, 35, 0),
  DateTime(2021, 1, 31, 15, 36, 0),
  DateTime(2021, 1, 31, 15, 37, 0),
  DateTime(2021, 1, 31, 15, 38, 0),
  DateTime(2021, 1, 31, 15, 39, 0),
  DateTime(2021, 1, 31, 15, 40, 0),
  DateTime(2021, 1, 31, 15, 41, 0),
  DateTime(2021, 1, 31, 15, 42, 0),
  DateTime(2021, 1, 31, 15, 43, 0),
  DateTime(2021, 1, 31, 15, 44, 0),
  DateTime(2021, 1, 31, 15, 45, 0),
  DateTime(2021, 1, 31, 15, 46, 0),
  DateTime(2021, 1, 31, 15, 47, 0),
  DateTime(2021, 1, 31, 15, 48, 0),
  DateTime(2021, 1, 31, 15, 49, 0),
  DateTime(2021, 1, 31, 15, 50, 0),
  DateTime(2021, 1, 31, 15, 51, 0),
  DateTime(2021, 1, 31, 15, 52, 0),
  DateTime(2021, 1, 31, 15, 53, 0),
  DateTime(2021, 1, 31, 15, 54, 0),
  DateTime(2021, 1, 31, 15, 55, 0),
  DateTime(2021, 1, 31, 15, 56, 0),
  DateTime(2021, 1, 31, 15, 57, 0),
  DateTime(2021, 1, 31, 15, 58, 0),
  DateTime(2021, 1, 31, 15, 59, 0),
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

  printSummary();
}

void loop() {
  if (!isTimeToWater()) {
    Serial.println(F("sulama vakti degil. uykuya geciyorum..."));
    Serial.flush();
    record(VALVE_IDLE);

    // sleep for an hour until next check
    sleepFor(WAKEUP_EVERY);
    return;
  }

  Serial.println(F("sulama vakti! vanayi aciyorum..."));

  // open the valve
  record(VALVE_OPENING);
  turnMotorLeft(1.5);
  stopMotor();
  record(VALVE_OPEN);

  // keep the valve open and let the plants get some water
  Serial.println(F("vana acildi. sulama bitene kadar uyuyorum..."));
  Serial.flush();
  sleepFor(WATERING_DURATION);

  // close the valve
  Serial.println(F("sulama bitti. vanayi kapiyorum..."));
  record(VALVE_CLOSING);

  turnMotorRight(1.5);
  stopMotor();

  record(VALVE_CLOSED);
  Serial.println(F("vana kapandi. uyku vakti..."));
  Serial.flush();

  sleepFor(WAKEUP_EVERY);
}

void printSummary() {
  powerOnPeripherals();
  getTime();
  getTemperature();

  Serial.println(F("### Vana1 ###"));
  printTime();
  printTemperature();
  Serial.println();

  int found = 0;
  Serial.println(F("### Sulama takvimi ###"));
  for (int i = 0; i < dayCount; i++) {
    DateTime d = schedule[i];
    String ds = dateTimeToString(d);
    if (now < d) {
      if (!found) {
        Serial.print(ds);
        Serial.println(F("  <--- Siradaki sulama"));
        found = 1;
      } else {
        Serial.println(ds);
      }
    } else {
      Serial.println(ds);
    }
  }
  Serial.println(F("=============="));
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
  delay(100); // wait a few ms for components to initialize
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
    Serial.flush();
    flashLED(3);
    return;
  }

  delay (2);
  TWBR = 72;  // 50 kHz at 8 MHz clock

  if (!RTC.lostPower()) {
    Serial.println(F("rtc: battery is empty"));
    Serial.flush();
    flashLED(3);
    return;
  }
}

void disableRTC () {
  // turn off i2c
  TWCR &= ~(bit(TWEN) | bit(TWIE) | bit(TWEA));

  // turn off i2c pull-ups
  digitalWrite (A4, LOW);
  digitalWrite (A5, LOW);
}

void getTime() {
  enableRTC();
  now = RTC.now();
  disableRTC();
}

void printTime() {
  Serial.print(F("saat: "));
  Serial.println(dateTimeToString(now));
}

String dateTimeToString(DateTime &dt) {
  char buffer[50];

  sprintf(buffer, "%d/%d/%d %02d:%02d:%02d",
          dt.year(), dt.month(), dt.day(),
          dt.hour(), dt.minute(), dt.second());

  return String(buffer);
}


void enableDHT() {
  dht.begin();
}

void disableDHT() {
  digitalWrite(DHT_PIN, LOW);
}

void getTemperature() {
  enableDHT();
  delay(2000);
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println(F("dht: failed to read from sensor!"));
    disableDHT();
    flashLED(4); // todo: blink fast to indicate an error
    return;
  }
  disableDHT();
}

void printTemperature() {
  Serial.print(F("sicaklik: "));
  Serial.print(temperature);
  Serial.print(F(" *C, nem: %"));
  Serial.println(humidity);
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
  getTime();
  powerOffPeripherals();

  Serial.println(F("sulama takvimine bakiyorum..."));
  for (int i = 0; i < dayCount; i++ ) {
    DateTime timeStart = schedule[i];
    DateTime timeEnd = timeStart + dateBuffer;

    if ((now >= timeStart) && (now <= timeEnd)) {
      Serial.print(F("sulama vakti gelmis: "));
      Serial.println(dateTimeToString(timeStart));
      return true;
    }
  }

  return false;
}

void record(unsigned int valveStatus) {
  powerOnPeripherals();

  SdFat sd;

  flashLED(5);

  if (!sd.begin(SD_CHIPSELECT, SPI_HALF_SPEED)) {
    Serial.println(F("sd: failed to initialize"));
    Serial.flush();
    flashLED(10);
    return;
  }

  SdFile file;
  SPI.begin();

  if (!file.open(logfile, O_CREAT | O_WRITE | O_APPEND)) {
    Serial.println(F("sd: failed to open file"));
    Serial.flush();
    flashLED(5);
    return;
  }
  getTime();
  getTemperature();

  char buf[30];
  sprintf(buf, "%04d-%02d-%02d %02d:%02d:%02d",
          (int) now.year(), (int) now.month(), (int) now.day(),
          (int) now.hour(), (int) now.minute(), (int) now.second());

  file.print(buf);
  file.print("\t");
  file.print(valveStatus);
  file.print("\t");
  file.print(temperature);
  file.print("\t");
  file.print(humidity);
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
