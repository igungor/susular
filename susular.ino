/*
   Vana 01.

   Pinler:
    D2  - DS3231 SQW alarm pini
    D3  - Limit switch 1 (Vana acildi sinyali)
    D4  - LImit switch 2 (Vana kapandi sinyali)
    D5  - TP6612FNG motor surucu girisi (INA1)
    D6  - TP6612FNG motor surucu girisi (INA2)
    D7  - DHT11 sensor girisi
    D8  - Indikator LED
    D9  - Cevre birimleri guc kapisi (MOSFET gate, 0 aktif)
    D10 - MikroSD kart SS (slave select)
    D11 - MikroSD kart MOSI
    D12 - MikroSD kart MISO
    D13 - MikroSD kart SCK
    A4  - DS3231 I2C SDA
    A5  - DS3231 I2C SCL

   Hata Indikatoru:
    sd-kart hatasi: 1 kisa flash.
    takvim hatasi:  2 kisa flash.
    rtc hatasi:     3 kisa flash.
    dht hatasi:     4 kisa flash.
*/

#include <EEPROM.h>
#include <LowPower.h>
#include <RTClib.h>
#include <SdFat.h>
#include <DHT.h>

#define ErrSDCard 1
#define ErrSchedule 2
#define ErrRTC 3
#define ErrDHT 4

// Pins
const int RTC_ALARM = 2;
const int VALVE_OPEN_SW = 3;
const int VALVE_CLOSED_SW = 4;
const int MOTOR_IN1 = 5;
const int MOTOR_IN2 = 6;
const int DHT_PIN = 7;
const int LED = 8;
const int POWER = 9;
const int SD_CHIPSELECT = 10;

// Constants
const int WATERING_DURATION = 12UL; // seconds
const int VALVE_TIMEOUT = 60UL; // seconds
const float VALVE_SWITCH_HOLD_DURATION = 250; // milliseconds

const char* logfile = "KAYIT.CSV"; // Valve log in TSV format
const char* scheduleFile = "TAKVIM.CSV"; // Schedule file in TSV format
const DateTime never = DateTime();
const int REF_VOLTAGE_ADDR = 0;

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
long vcc;
int refVoltage = 1081; // set default value for bandgap voltage

RTC_DS3231 RTC;
DHT dht(DHT_PIN, DHT11, 3); // TODO: 16MHz: 6, 8MHz: 3

const int dayCount = 60;
DateTime schedule[dayCount];


void setup() {
  pinMode(VALVE_OPEN_SW, INPUT_PULLUP);
  pinMode(VALVE_CLOSED_SW, INPUT_PULLUP);
  pinMode(RTC_ALARM, INPUT_PULLUP);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(POWER, OUTPUT);

  Serial.begin(115200);

  SdFile::dateTimeCallback(fileDateTime);

  if (false) { // NOTE: enable to save measured bandgap voltage
    analogReference(INTERNAL);
    analogRead(A0);

    Serial.println(F("Read AREF pin value (pin20 on pro mini):"));
    while (!Serial.available()) {};

    int bandgap = Serial.parseInt(SKIP_ALL);
    Serial.println(bandgap, DEC);

    EEPROM.put(REF_VOLTAGE_ADDR, bandgap);
    analogReference(DEFAULT);
  }

  // use recorded bandgap voltage if any. otherwise, use a generic voltage value.
  int eepromValue;
  EEPROM.get(REF_VOLTAGE_ADDR, eepromValue);
  Serial.print(F("vcc: bandgap value in EEPROM: "));
  Serial.println(eepromValue, DEC);
  if (eepromValue < 1000) {
    Serial.println(F("vcc: bandgap value is too low, voltage measurement is not done!"));
  } else {
    refVoltage = eepromValue;
  }

  stopMotor();
  powerOnPeripherals();
  enableRTC();

  // setup RTC
  RTC.disable32K();
  RTC.clearAlarm(1);
  RTC.clearAlarm(2);
  RTC.writeSqwPinMode(DS3231_OFF);
  RTC.disableAlarm(2);
  if (false) { // NOTE: enable to adjust RTC
    RTC.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  disableRTC();

  readSchedule();
  // printSummary();
}

void loop() {
  Serial.print("recording...");
  record(VALVE_IDLE);
  Serial.println("OK");
  delay(5000);
  return;

  powerOnPeripherals();

  DateTime dt = nextWateringTime();
  if (dt == never) {
    Serial.println(F("onumuzde sulanacak gun yok"));
    RTC.clearAlarm(1);
    flashLED(5);
    sleepFor(8);
    return;
  }

  setAlarm(dt);
  sleepForever();

  openValve();
  readTime();
  setAlarm(now + TimeSpan(WATERING_DURATION));
  sleepForever();

  closeValve();
}

void setAlarm(DateTime date) {
  enableRTC();
  RTC.clearAlarm(1);

  bool ok = RTC.setAlarm1(date, DS3231_A1_Date);
  if (!ok) {
    Serial.println(F("rtc: alarm kurulamadi: "));
    Serial.print(dateTimeToString(date));
    return;
  }
  disableRTC();

  printTime();
  Serial.print(F("rtc: bir sonraki uyanma zamani: "));
  Serial.println(dateTimeToString(date));
  Serial.flush();
  delay(100);

  attachInterrupt(digitalPinToInterrupt(RTC_ALARM), onAlarm, FALLING);
}

void onAlarm() {}

void printSummary() {
  readTime();
  readTemperature();
  readTemperature();
  readVcc();

  Serial.println(F("### Vana1 ###"));
  printTime();
  printTemperature();
  printVcc();
  Serial.println();

  bool found = 0;
  Serial.println(F("### Sulama takvimi ###"));
  for (int i = 0; i < dayCount; i++) {
    DateTime d = schedule[i];
    String ds = dateTimeToString(d);

    Serial.print(ds);
    if (now < d && !found) {
      Serial.println(F("<--- Siradaki sulama"));
      found = true;
      continue;
    }
    Serial.println();
  }
  Serial.println(F("=============="));
}

void readSchedule() {
  SdFat sd;
  if (!sd.begin(SD_CHIPSELECT)) {
    Serial.println(F("sd: failed to initialize"));
    errorHalt(ErrSDCard);
    return;
  }

  SdFile file;
  if (!file.open(scheduleFile, O_RDONLY)) {
    Serial.println(F("sd: failed to open schedule file"));
    errorHalt(ErrSDCard);
    return;
  }

  /*
    Date format: '2020-02-01 20:00:00\n'
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

    DateTime date = DateTime(line);
    if (date.year() < 2021) {
      sdErrorHalt("sd: unable to parse schedule.", idx, line);
    }
    schedule[idx] = date;
    idx++;
  }

  file.close();

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

void errorHalt(uint8_t err) {
  disableDHT();
  disableRTC();
  powerOffPeripherals();

  for (;;) {
    flashLED(err);
    sleepFor(5);
  }
}

// shutdowns all MCU units.
void sleepForever() {
  powerOffPeripherals();
  delay(100);
  Serial.flush();

  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  detachInterrupt(digitalPinToInterrupt(RTC_ALARM));
}

// shutdowns all MCU units for given seconds.
void sleepFor(unsigned long seconds) {
  powerOffPeripherals();
  Serial.flush();

  // find the iteration count to sleep for SLEEP_8S
  int times = seconds / 8;

  for (int i = 0; i < times; i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  }

  int remainder = seconds % 8;
  for (int i = 0; i < remainder; i++) {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
  }
  detachInterrupt(digitalPinToInterrupt(RTC_ALARM));
}

void powerOnPeripherals() {
  Serial.print(F("powering on..."));
  pinMode(POWER, OUTPUT);
  digitalWrite(POWER, LOW); // p-channel mosfet. low -> active.
  delay(1000);
  Serial.println(F("OK"));
}

void powerOffPeripherals() {
  Serial.print(F("powering off..."));

  // put all digital pins into low stats
  for (byte pin = 6; pin < 14; pin++) {
    pinMode (pin, OUTPUT);
    digitalWrite (pin, LOW);
  }

  pinMode(VALVE_OPEN_SW, INPUT_PULLUP);
  pinMode(VALVE_CLOSED_SW, INPUT_PULLUP);
  pinMode(RTC_ALARM, INPUT_PULLUP);
  digitalWrite (POWER, HIGH);

  Serial.println(F("OK"));
}

void enableRTC () {
  if (!RTC.begin()) {
    Serial.println(F("rtc: not initialized"));
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

void readTime() {
  enableRTC();
  now = RTC.now();
  if (now.year() < 2021) {
    disableRTC();
    printTime();
    Serial.println(F("rtc: clock is not adjusted!"));
    errorHalt(ErrRTC);
  }
  disableRTC();
}

void printTime() {
  Serial.print(F("saat: "));
  Serial.println(dateTimeToString(now));
}

String dateTimeToString(DateTime date) {
  return date.timestamp(DateTime::TIMESTAMP_FULL);
}

void fileDateTime(uint16_t* date, uint16_t* time)  {
  *date = FAT_DATE(now.year(), now.month(), now.day());
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void readVcc() {
  // REFS0 : Selects AVcc external reference
  // MUX3 MUX2 MUX1 : Selects 1.1V (VBG)
  ADMUX = bit (REFS0) | bit (MUX3) | bit (MUX2) | bit (MUX1);
  ADCSRA |= bit( ADSC );  // start conversion
  while (ADCSRA & bit (ADSC)) {} // wait for conversion to complete

  vcc = ((refVoltage * 1024L) / ADC) + 5;
  return;
}

void printVcc() {
  Serial.print(F("vcc: "));
  Serial.print(vcc / 1000.0f);
  Serial.println(F("V"));
}

void enableDHT() {
  dht.begin();
  delay(100);
}

void disableDHT() {
  digitalWrite(DHT_PIN, LOW);
}

void readTemperature() {
  Serial.println(11);
  enableDHT();
  Serial.println(22);

  // retry until a successful reading can occur
  float t, h;
  for (int i = 0; i < 4; i++) {
    delay(2000);
    Serial.println(33);

    t = dht.readTemperature();
    Serial.println(44);
    h = dht.readHumidity();
    Serial.println(55);

    if (isnan(t) || isnan(h)) {
      Serial.println("either t or h is nan");
      continue;
    }
    break;
  }
  Serial.println(66);
  disableDHT();
  Serial.println(77);

  if (!isnan(t)) {
    temperature = String(t);
  } else {
    temperature = String("NaN");
    Serial.println(F("dht: failed to read temperature!"));
    flashLED(ErrDHT);
  }

  if (!isnan(h)) {
    humidity = String(h);
  } else {
    humidity = String("NaN");
    Serial.println(F("dht: failed to read humidity!"));
    flashLED(ErrDHT);
  }
}

void printTemperature() {
  Serial.print(F("sicaklik: "));
  Serial.print(temperature);
  Serial.print(F(" *C, nem: % "));
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

DateTime nextWateringTime() {
  readTime();

  Serial.println(F("sulama takvimine bakiyorum..."));
  for (int i = 0; i < dayCount; i++ ) {
    DateTime t = schedule[i];

    if (t >= now) {
      Serial.print(F("sonraki sulama: "));
      Serial.println(dateTimeToString(t));
      return t;
    }
  }

  return never;
}

void record(uint8_t valveStatus) {
  flashLED(5); // visual cue to not to poweroff or remove sdcard

  readTime();
  Serial.println(1);
  readTemperature();
  readVcc();

  SdFat sd;
  if (!sd.begin(SD_CHIPSELECT)) {
    Serial.println(F("sd: failed to initialize"));
    flashLED(ErrSDCard);
    return;
  }

  SdFile file;
  if (!file.open(logfile, O_CREAT | O_WRITE | O_APPEND)) {
    Serial.println(F("sd: failed to open log file"));
    flashLED(ErrSDCard);
    return;
  }

  file.print(dateTimeToString(now));
  file.print("\t");
  file.print(valveStatus);
  file.print("\t");
  file.print(temperature);
  file.print("\t");
  file.print(humidity);
  file.print("\t");
  file.print(vcc);
  file.println();

  file.sync();
  file.close();
  delay(100);

  flashLED(2);
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
}

void openValve() {
  powerOnPeripherals();

  Serial.println(F("sulama vakti! vanayi aciyorum..."));
  record(VALVE_OPENING);

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  waitForSwitch(VALVE_OPEN_SW);

  stopMotor();
  delay(100);
  record(VALVE_OPEN);

  Serial.println(F("vana acildi"));
}

void closeValve() {
  powerOnPeripherals();

  Serial.println(F("sulama bitti. vanayi kapiyorum..."));
  record(VALVE_CLOSING);

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  waitForSwitch(VALVE_CLOSED_SW);

  stopMotor();
  delay(100);
  record(VALVE_CLOSED);

  Serial.println(F("vana kapandi"));
}

// waitForSwitch expects given switch to read logic low,
// indicating that it's pressed in given time. There's an
// extra debounce logic to make sure given switch is
// definitely pressed.
void waitForSwitch(int valveSwitch) {
  unsigned long startTime = millis();
  unsigned long endTime, duration;

  // pin is set to INPUT_PULLUP. expect logic high by default.
  bool lastState = HIGH;
  unsigned long lastDebounceTime = 0;

  for (;;) {
    endTime = millis();
    duration = endTime - startTime;
    if (duration > (VALVE_TIMEOUT * 1000UL)) {
      Serial.print(F("vana: zaman asimi. gecen sure: "));
      Serial.print(duration / 1000);
      Serial.println(F(" saniye."));
      return;
    }

    bool state = digitalRead(valveSwitch);

    if (state != lastState) {
      lastDebounceTime = millis();
      lastState = state;
    }

    if ((millis() - lastDebounceTime) > VALVE_SWITCH_HOLD_DURATION) {
      // expect limit switch to stay at logic low for enough time
      if (!lastState) {
        Serial.println(F("vana: limit switch tetiklendi"));
        return;
      }
    }
  }
}
