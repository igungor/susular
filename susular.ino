/*
   Vana 01.

   Pinler:
    D2 - IR cikisi (enkoder sensoru)
    D4 - Motor surucu girisi (INA1)
    D5 - Motor surucu girisi (INA2)
    D8 - indikator LED
    D9 - cevre birimleri guc kapisi (MOSFET gate, 0 aktif)
    D10 - SD kart SS (slave select)
    D11 - SD kart MOSI
    D12 - SD kart MISO
    D13 - SD kart SCK
    A4  - I2C SDA (RTC)
    A5  - I2C SCL (RTC)

   Hata Indikatoru:
    sd-kart hatasi: 1 kisa flash.
    takvim hatasi:  2 kisa flash.
    rtc hatasi:     3 kisa flash.
    dht hatasi:     4 kisa flash.
*/

#include <LowPower.h>
#include <RTClib.h>
#include <SdFat.h>
#include <DHT.h>

#define ErrSDCard 1
#define ErrSchedule 2
#define ErrRTC 3
#define ErrDHT 4

// Pins
const int ENCODER = 2;
const int MOTOR_IN1 = 5;
const int MOTOR_IN2 = 6;
const int DHT_PIN = 7;
const int LED = 8;
const int POWER = 9;
const int SD_CHIPSELECT = 10;

// Constants
const int ENCODER_PERIOD = 16; // Number of pulses on encoder wheel
const int WATERING_DURATION = 5UL; // seconds
const int WAKEUP_EVERY = 10UL; // seconds
const int ENCODER_TIMEOUT = 8; // seconds
const float VALVE_TURN_COUNT = 1.5;
const char* logfile = "KAYIT.CSV"; // Valve log in TSV format
const char* scheduleFile = "TAKVIM.CSV"; // Schedule file in TSV format

// Valve Status
const byte VALVE_IDLE = 0;
const byte VALVE_OPENING = 1;
const byte VALVE_OPEN = 2;
const byte VALVE_CLOSING = 3;
const byte VALVE_CLOSED = 4;

// Global variables
DateTime now;
String temperature;
String humidity;

RTC_DS3231 RTC;
DHT dht(DHT_PIN, DHT11, 3);

const int dayCount = 41;
DateTime schedule[dayCount];

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
  SdFile::dateTimeCallback(fileDateTime);

  stopMotor();
  powerOnPeripherals();

  // NOTE: enable to adjust RTC
  if (false) {
    enableRTC();
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
    disableRTC();
  }

  readSchedule();
  printSummary();

  powerOffPeripherals();
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
  openValve();
  Serial.println(F("vana acildi. sulama bitene kadar uyuyorum..."));
  Serial.flush();

  sleepFor(WATERING_DURATION);

  Serial.println(F("sulama bitti. vanayi kapiyorum..."));
  closeValve();
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

  bool found = 0;
  Serial.println(F("### Sulama takvimi ###"));
  for (int i = 0; i < dayCount; i++) {
    DateTime d = schedule[i];
    String ds = dateTimeToString(d);

    Serial.print(ds);
    if (now < d && !found) {
      Serial.println(F("  <--- Siradaki sulama"));
      found = true;
      continue;
    }
    Serial.println();
  }
  Serial.println(F("=============="));
}

void readSchedule() {
  powerOnPeripherals();

  SdFat sd;
  if (!sd.begin(SD_CHIPSELECT)) {
    Serial.println(F("sd: failed to initialize"));
    Serial.flush();
    errorHalt(ErrSDCard);
    return;
  }

  SdFile file;
  if (!file.open(scheduleFile, O_RDONLY)) {
    Serial.println(F("sd: failed to open schedule file"));
    Serial.flush();
    errorHalt(ErrSDCard);
    return;
  }

  /*
     Date format is like below:
     2020-02-01 20:00:00\n
  */
  char line[22];
  int idx = 0;
  while (file.available()) {
    int n = file.fgets(line, sizeof(line));
    if (n <= 0) {
      sdErrorHalt("sd: unable to read schedule.", idx, line);
    }

    if (line[n - 1] != '\n' && n == (sizeof(line) - 1)) {
      sdErrorHalt("sd: schedule line too long.", idx, line);
    }

    DateTime date;
    if (!parseLine(line, date)) {
      sdErrorHalt("sd: unable to parse schedule.", idx, line);
    }

    schedule[idx] = date;
    idx++;
  }

  file.close();

  powerOffPeripherals();
  flashLED(2);
}

void sdErrorHalt(const char* msg, uint8_t idx, char* line) {
  Serial.print(msg);
  Serial.print(F(" line no: "));
  Serial.print(idx + 1);
  Serial.print(F(". line: <"));
  Serial.print(line);
  Serial.println(F(">"));
  errorHalt(ErrSchedule);
}

bool parseLine(char* str, DateTime &date) {
  char* ptr;
  const char * sep = " -:";

  str = strtok(str, sep);
  if (!str) return false;
  uint16_t year = strtoul(str, &ptr, 10);
  if (year < 2021) return false;

  str = strtok(nullptr, sep);
  if (!str) return false;
  uint8_t month = strtoul(str, &ptr, 10);
  if (month > 12 || month == 0) return false;

  str = strtok(nullptr, sep);
  if (!str) return false;
  uint8_t day = strtoul(str, &ptr, 10);
  if (day > 31 || day == 0) return false;

  str = strtok(nullptr, sep);
  if (!str) return false;
  uint8_t hour = strtoul(str, &ptr, 10);
  if (hour > 24) return false;

  str = strtok(nullptr, sep);
  if (!str) return false;
  uint8_t minute = strtoul(str, &ptr, 10);
  if (minute > 60) return false;

  str = strtok(nullptr, sep);
  if (!str) return false;
  uint8_t second = strtoul(str, &ptr, 10);
  if (second > 60) return false;

  date =  DateTime(year, month, day, hour, minute, second);
  return true;
}

void errorHalt(uint8_t err) {
  disableDHT();
  disableRTC();
  powerOffPeripherals();

  for (;;) {
    flashLED(err);
    sleepFor(5);
  }
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
    flashLED(ErrRTC);
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

void getTime() {
  enableRTC();
  now = RTC.now();
  if (now.year() < 2021) {
    printTime();
    Serial.println(F("rtc: clock is not adjusted!"));
    Serial.flush();
    errorHalt(ErrRTC);
  }
  disableRTC();
}

void printTime() {
  Serial.print(F("saat: "));
  Serial.println(dateTimeToString(now));
}

String dateTimeToString(DateTime &dt) {
  char buffer[50];

  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
          dt.year(), dt.month(), dt.day(),
          dt.hour(), dt.minute(), dt.second());

  return String(buffer);
}

void fileDateTime(uint16_t* date, uint16_t* time)  {
  *date = FAT_DATE(now.year(), now.month(), now.day());
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void enableDHT() {
  dht.begin();
}

void disableDHT() {
  digitalWrite(DHT_PIN, LOW);
}

void getTemperature() {
  enableDHT();

  // retry until a successful reading can occur
  float t, h;
  for (int i = 0; i < 4; i++) {
    delay(1000);
    t = dht.readTemperature();
    h = dht.readHumidity();
    if (isnan(t) || isnan(h)) {
      continue;
    }
    break;
  }
  disableDHT();

  if (!isnan(t)) {
    temperature = String(t);
  } else {
    temperature = String("NaN");
    Serial.println(F("dht: failed to read temperature!"));
    Serial.flush();
    flashLED(ErrDHT);
    return;
  }

  if (!isnan(h)) {
    humidity = String(h);
  } else {
    humidity = String("NaN");
    Serial.println(F("dht: failed to read humidity!"));
    Serial.flush();
    flashLED(ErrDHT);
    return;
  }
}

void printTemperature() {
  Serial.print(F("sicaklik: "));
  Serial.print(temperature);
  Serial.print(F(" *C, nem: %"));
  Serial.println(humidity);
}

void flashLED(byte times) {
  const uint8_t flashDelay = 150;

  for (int i = 0; i < times; i++) {
    digitalWrite(LED, HIGH);
    delay(flashDelay);
    digitalWrite(LED, LOW);
    delay(flashDelay);
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

void record(uint8_t valveStatus) {
  powerOnPeripherals();
  flashLED(5); // visual cue to not to poweroff or remove sdcard

  SdFat sd;
  if (!sd.begin(SD_CHIPSELECT, SPI_HALF_SPEED)) {
    Serial.println(F("sd: failed to initialize"));
    Serial.flush();
    flashLED(ErrSDCard);
    return;
  }

  SdFile file;
  if (!file.open(logfile, O_CREAT | O_WRITE | O_APPEND)) {
    Serial.println(F("sd: failed to open log file"));
    Serial.flush();
    flashLED(ErrSDCard);
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
  powerOffPeripherals();

  flashLED(2);
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void openValve() {
  record(VALVE_OPENING);

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  waitForTurn(VALVE_TURN_COUNT);

  stopMotor();
  record(VALVE_OPEN);
}

void closeValve() {
  record(VALVE_CLOSING);

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  waitForTurn(VALVE_TURN_COUNT);

  stopMotor();
  record(VALVE_CLOSED);
}

void waitForTurn(float turns) {
  int encoderPulse = 0;
  bool lastState = 1;  // assume last reading was off
  unsigned long startTime = millis();
  unsigned long endTime, duration;

  do {
    endTime = millis();
    duration = endTime - startTime;
    if (duration > (ENCODER_TIMEOUT * 1000)) {
      Serial.print(F("encoder: zaman asimi. gecen sure: "));
      Serial.print(duration);
      Serial.println(F(" milisaniye."));
      return;
    }
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
