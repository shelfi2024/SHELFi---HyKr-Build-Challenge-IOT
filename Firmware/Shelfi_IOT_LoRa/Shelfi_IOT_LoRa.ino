#include <HardwareSerial.h>
#include "filter.c"
#include "stm32f1xx_hal.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#define EE_TARE_ADDR   0      // 4 bytes (long)
#define EE_SPAN_ADDR   4      // 4 bytes (long)
#define TRANSMIT_WINDOW_MS 60000  // 1 minute

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define DOUT_PIN PA4   // HX710B data
#define SCK_PIN PA5    // HX710B clock

#define HB_LED PC13
#define LOADCELL_EN PB12
#define LORA_EN PB13
#define BOOT PB14
#define LRESET PB15
#define BLVL_PIN PA1

#define LCD_FB PB7        // LCD detect
#define TARE_BTN PB0      // TARE
#define SAVE_BTN PA6      // SAVE
#define SPAN_BTN PA7      // SPAN

#define WEIGHT_DELTA 50   // grams (change only if ≥ 5g)
#define STABLE_TIME_MS   3000   // must stay stable for 1.5 sec

#define ADC_BATT_FULL   737   // 12V
#define ADC_BATT_EMPTY  615   // 9V
uint16_t battADC = 0;
uint8_t batteryPercent = 0;




HardwareSerial LORA(USART2);

signed long data_raw = 0;
signed long average = 0;
signed long lastAverage = 0;

int currentID = 11;

float BLVL = 0;
bool LCDactive = false;

signed long tare = 0;
signed long span = 0;
float cal_factor = 0;
long RES = 5;
long finalweight;

long lastWeight = 0;
bool weightChanged = false;

long lastStableWeight = 0;
unsigned long stableStartTime = 0;
bool waitingForStability = false;

bool transmitWindowActive = false;
unsigned long transmitStartTime = 0;


/* -------------------- LCD INIT -------------------- */
void initLCD() {
  LCDactive = 1;

  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();

  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LCD CONNECTED");

  delay(800);
  lcd.clear();
}

void loadCalibration() {
    EEPROM.get(EE_TARE_ADDR, tare);
    EEPROM.get(EE_SPAN_ADDR, span);

    // First time flash contains 0xFFFFFFFF → treat as empty
    if (tare == -1) tare = 0;
    if (span == -1) span = 0;

    Serial.print("Loaded TARE: ");
    Serial.println(tare);
    Serial.print("Loaded SPAN: ");
    Serial.println(span);
}

void saveCalibration() {
    EEPROM.put(EE_TARE_ADDR, tare);
    EEPROM.put(EE_SPAN_ADDR, span);
    // very important for STM32

    Serial.println("Calibration saved to EEPROM");
}


/* -------------------- WEIGHT CALC -------------------- */
void weight_ingrams(long Average)
{
    if (span == tare) {
        finalweight = 0;
        return;
    }

    if (span > tare)
        cal_factor = 1000.0 / float(span - tare);
    else
        cal_factor = 1000.0 / float(tare - span);

    float w;

    if (span > tare)
        w = (Average - tare) * cal_factor;
    else
        w = (tare - Average) * cal_factor;

    w = ((long)(w + RES/2)) / RES * RES;
    finalweight = (long)w;
}

/* -------------------- HX710B READING -------------------- */
long readHX710B() {
    long count = 0;
    while (digitalRead(DOUT_PIN));

    for (int i = 0; i < 24; i++) {
        digitalWrite(SCK_PIN, HIGH);
        count = (count << 1) | digitalRead(DOUT_PIN);
        digitalWrite(SCK_PIN, LOW);
    }

    digitalWrite(SCK_PIN, HIGH);
    digitalWrite(SCK_PIN, LOW);

    if (count & 0x800000) {
        count |= 0xFF000000;
    }

    return count;
}

/* -------------------- LORA -------------------- */
void sendCommand(String command) {
    LORA.write(command.c_str());
    LORA.write("\r\n");
    delay(20);

    while (LORA.available()) {
        Serial.print("Response: ");
        Serial.println(LORA.readString());
    }
}

void ABP_mode() {
  sendCommand("AT+CJOINMODE=1");
  delay(100);
  sendCommand("AT+CDEVADDR=2609D4BE");
  delay(100);
  sendCommand("AT+CADR=0");
  delay(100);
  sendCommand("AT+CDATARATE=5");
  delay(100);
  sendCommand("AT+CCONFIRM=1");         // confirmed uplinks
  delay(100);
  sendCommand("AT+CDEVADDR?");         // confirmed uplinks
  delay(100);
  sendCommand("AT+CAPPSKEY?");         // confirmed uplinks
  delay(100);
  sendCommand("AT+CNWKSKEY?");         // confirmed uplinks
  delay(100);
  sendCommand("AT+CSAVE");
  delay(200);
}

/* ------------ NEW: TX FORMAT INCLUDING finalweight ------------- */
void transmitCompactData(uint16_t deviceID, int16_t adcAverage, uint8_t batteryPercent, int16_t weight_g) {


    uint8_t payload[8];

    payload[0] = (deviceID >> 8) & 0xFF;
    payload[1] = deviceID & 0xFF;

    payload[2] = (adcAverage >> 8) & 0xFF;
    payload[3] = adcAverage & 0xFF;

    payload[4] = batteryPercent;   // 0–100
    payload[5] = 0x00;          // reserved


    payload[6] = (weight_g >> 8) & 0xFF;
    payload[7] = weight_g & 0xFF;

    String hexData = "";
    char hex[3];
    for (int i = 0; i < 8; i++) {
        sprintf(hex, "%02X", payload[i]);
        hexData += hex;
    }

    sendCommand("AT+DTRX=1,2,8," + hexData);
}

/* -------------------- SETUP -------------------- */
void setup() {
    Serial.begin(115200);
    LORA.begin(9600);

    pinMode(HB_LED, OUTPUT);
    pinMode(LOADCELL_EN, OUTPUT);
    pinMode(LORA_EN, OUTPUT);
    pinMode(LRESET, OUTPUT);
    pinMode(BOOT, OUTPUT);

    pinMode(LCD_FB, INPUT_PULLUP);
    pinMode(TARE_BTN, INPUT_PULLUP);
    pinMode(SPAN_BTN, INPUT_PULLUP);
    pinMode(SAVE_BTN, INPUT_PULLUP);

    pinMode(DOUT_PIN, INPUT);
    pinMode(SCK_PIN, OUTPUT);
    digitalWrite(SCK_PIN, LOW);

    digitalWrite(LOADCELL_EN, HIGH);
    digitalWrite(LORA_EN, HIGH);

    digitalWrite(LRESET, LOW);
    delay(800);
    digitalWrite(LRESET, HIGH);

    delay(2000);
    ABP_mode();

    loadCalibration();

}

/* -------------------- LOOP -------------------- */
void loop() {

    /* -------- Read ADC -------- */
    data_raw = readHX710B();
    average = long(fir1(data_raw)) >> 1;
    weight_ingrams(average);
    Serial.print("Raw ADC :");
    Serial.println(data_raw);
    battADC = analogRead(BLVL_PIN);

    /* -------- Battery Percentage -------- */
    if (battADC >= ADC_BATT_FULL) {
        batteryPercent = 100;
}
    else if (battADC <= ADC_BATT_EMPTY) {
        batteryPercent = 0;
}
    else {
        batteryPercent = map(battADC, ADC_BATT_EMPTY, ADC_BATT_FULL, 0, 100);
}   
/* -------- LCD DETECT -------- */
    if (digitalRead(LCD_FB) == 0 && LCDactive == false) {
        initLCD();

    }
    if (digitalRead(LCD_FB) == 1) {
        LCDactive=0;

    }

    /* -------- Buttons -------- */
    if (LCDactive) {

        if (!digitalRead(TARE_BTN)) {
            tare = average;
            lcd.clear();
            lcd.print("TARE SET");
            delay(300);
            saveCalibration();
        }

        if (!digitalRead(SPAN_BTN)) {
            span = average;
            lcd.clear();
            lcd.print("SPAN SET");
            delay(300);
            saveCalibration();
        }

        if (!digitalRead(SAVE_BTN)) {
            // Future: EEPROM

            lcd.clear();
            lcd.print("SAVE OK");
            delay(300);
            saveCalibration();
        }

        // UPDATE DISPLAY
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ADC:");
        lcd.print(average);

        lcd.setCursor(0, 1);
        lcd.print("WT:");
        lcd.print(finalweight);
        lcd.print("g");

        delay(100);
    }
// ---- First run init ----
static bool firstRun = true;
if (firstRun) {
    lastWeight = finalweight;
    lastStableWeight = finalweight;
    firstRun = false;
}

// ---- Step 1: Detect large change ----
if (!waitingForStability && abs(finalweight - lastWeight) >= WEIGHT_DELTA) {
    waitingForStability = true;
    stableStartTime = millis();
    lastStableWeight = finalweight;
}

// ---- Step 2: Check stability ----
if (waitingForStability) {

    if (abs(finalweight - lastStableWeight) > WEIGHT_DELTA ) {
        stableStartTime = millis();      // reset timer
        lastStableWeight = finalweight;
    }

    if (millis() - stableStartTime >= STABLE_TIME_MS) {
        weightChanged = true;
        waitingForStability = false;
    }
}

// ---- LoRa TX ----
static uint32_t lastTX = 0;

// ---- Start transmission window ----
if (weightChanged) {
    transmitWindowActive = true;
    transmitStartTime = millis();
    weightChanged = false;
}

// ---- During active window ----
if (transmitWindowActive) {

    if (millis() - lastTX > 400) {
        transmitCompactData(currentID, average, batteryPercent, finalweight);
        lastTX = millis();
        lastWeight = finalweight;
    }

    // ---- Stop after 1 minute ----
    if (millis() - transmitStartTime >= TRANSMIT_WINDOW_MS) {
        transmitWindowActive = false;
    }
}
}