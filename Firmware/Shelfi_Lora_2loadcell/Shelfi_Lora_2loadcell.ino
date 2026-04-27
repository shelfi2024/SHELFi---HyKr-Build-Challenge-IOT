#include <HardwareSerial.h>
#include "filter.c"
#include "stm32f1xx_hal.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// EEPROM ADDRESS MAP (each long = 4 bytes)
#define ADDR_TARE1  0
#define ADDR_SPAN1  4
#define ADDR_TARE2  8
#define ADDR_SPAN2  12


LiquidCrystal_I2C lcd(0x27, 16, 2);

#define DOUT_PIN1 PA4  // HX710B1 data
#define SCK_PIN1 PA5   // HX710B1 clock
#define DOUT_PIN2 PA9  // HX710B2 data
#define SCK_PIN2 PA10  // HX710B2 clock

#define HB_LED PC13
#define LOADCELL_EN PB12
#define LORA_EN PB13
#define BOOT PB14
#define LRESET PB15
#define BLVL_PIN PA1

#define LCD_FB PB7
#define TARE_BTN PB0
#define SWITCHER_BTN PA6   // Replaced SAVE_BTN with SWITCHER_BTN per request
#define SPAN_BTN PA7
#define LONG_PRESS_TIME 10000   // 10 seconds

HardwareSerial LORA(USART2);

const int numReadings1 = 10;
int readings1[numReadings1];
int readIndex1 = 0;
signed long total1 = 0;
signed long average1 = 0;
signed long lastAverage1 = 0;

const int numReadings2 = 10;
int readings2[numReadings2];
int readIndex2 = 0;
signed long total2 = 0;
signed long average2 = 0;
signed long lastAverage2 = 0;

int currentID1 = 31 ;
int currentID2 = 32 ;
unsigned long channelSwitchTimer = 0;
unsigned long transmitdelay = 1000;
int currentChannel = 1;
const int maxChannels = 8;

signed long data_raw1 = 0;
signed long data_raw2 = 0;
long tmr = 0;
long ptmr = 0;
int BEAT = 0;
bool lastTareBtn = HIGH;
bool lastSpanBtn = HIGH;
bool lastSwitcherBtn = HIGH;


int motion = 0;
long lastMotionChangeTime = 0;

float BLVL = 0;
int processRun = 0;

bool loraPowered = true;
bool LCDactive = false;
long ontmr = 0;
long pontmr = 0;

signed long tare1 =0;
signed long span1 =0;
signed long totalweight1 =0;
float cal_factor1 = 0;
long RES1 = 10;
int residue1 ;
long finalweight1 = 0;

signed long tare2 =0;
signed long span2 =0;
signed long totalweight2 =0;
float cal_factor2 = 0;
long RES2 = 10;
int residue2 ;
long finalweight2 = 0;

/* New: switcher state and send-first toggling */
bool calibrateChannel2 = false; // false => calibrate channel1; true => calibrate channel2
bool sendFirst = true;          // alternates between sending data1 and data2 each 400 ms

#define ADC_BATT_FULL   737   // 12V
#define ADC_BATT_EMPTY  615   // 9V

uint16_t battADC = 0;
uint8_t batteryPercent = 0;


unsigned long lastTarePress = 0;
unsigned long lastSpanPress = 0;
unsigned long lastSwitcherPress = 0;

long lastWeight1 = 0;
long lastWeight2 = 0;

bool weightChanged1 = false;
bool weightChanged2 = false;

#define WEIGHT_DELTA 50   // grams (adjust if needed)
#define STABLE_TIME_MS 3000   // must stay stable for 1.5 sec
// -------- Stability tracking CH1 --------
bool waitingForStability1 = false;
unsigned long stableStartTime1 = 0;
long lastStableWeight1 = 0;

// -------- Stability tracking CH2 --------
bool waitingForStability2 = false;
unsigned long stableStartTime2 = 0;
long lastStableWeight2 = 0;

unsigned long switchPressStart = 0;
bool switchHeld = false;


// -------- First run protection --------
bool firstRun = true;


const unsigned long debounceDelay = 200; // 200ms debounce

void saveCalibration() {
    EEPROM.put(ADDR_TARE1, tare1);
    EEPROM.put(ADDR_SPAN1, span1);
    EEPROM.put(ADDR_TARE2, tare2);
    EEPROM.put(ADDR_SPAN2, span2);
    Serial.println("Calibration Saved to EEPROM");
}

void loadCalibration() {
    EEPROM.get(ADDR_TARE1, tare1);
    EEPROM.get(ADDR_SPAN1, span1);
    EEPROM.get(ADDR_TARE2, tare2);
    EEPROM.get(ADDR_SPAN2, span2);

    Serial.println("Loaded EEPROM Values:");
    Serial.println("TARE1: " + String(tare1));
    Serial.println("SPAN1: " + String(span1));
    Serial.println("TARE2: " + String(tare2));
    Serial.println("SPAN2: " + String(span2));
}

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
}

void weight_ingrams1(long Average)
{
    if (span1 == tare1) {
        finalweight1 = 0;
        return;
    }

    if (span1 > tare1)
        cal_factor1 = 1000.0 / float(span1 - tare1);
    else
        cal_factor1 = 1000.0 / float(tare1 - span1);

    float w;

    if (span1 > tare1)
        w = (Average - tare1) * cal_factor1;
    else
        w = (tare1 - Average) * cal_factor1;

    w = ((long)(w + RES1/2)) / RES1 * RES1;

    finalweight1 = (long)w;
}


void weight_ingrams2(long Average)
{
    if (span2 == tare2) {
        finalweight2 = 0;
        return;
    }

    if (span2 > tare2)
        cal_factor2 = 1000.0 / float(span2 - tare2);
    else
        cal_factor2 = 1000.0 / float(tare2 - span2);

    float w;

    if (span2 > tare2)
        w = (Average - tare2) * cal_factor2;
    else
        w = (tare2 - Average) * cal_factor2;

    w = ((long)(w + RES2/2)) / RES2 * RES2;

    finalweight2 = (long)w;
}
// Read from HX710B1
long readHX710B1() {
    long count = 0;
    while (digitalRead(DOUT_PIN1));

    for (int i = 0; i < 24; i++) {
        digitalWrite(SCK_PIN1, HIGH);
        count = (count << 1) | digitalRead(DOUT_PIN1);
        digitalWrite(SCK_PIN1, LOW);
    }

    digitalWrite(SCK_PIN1, HIGH);
    digitalWrite(SCK_PIN1, LOW);

    if (count & 0x800000) {
        count |= 0xFF000000;
    }

    return count;
}

// Read from HX710B2
long readHX710B2() {
    long count = 0;
    while (digitalRead(DOUT_PIN2));

    for (int i = 0; i < 24; i++) {
        digitalWrite(SCK_PIN2, HIGH);
        count = (count << 1) | digitalRead(DOUT_PIN2);
        digitalWrite(SCK_PIN2, LOW);
    }

    digitalWrite(SCK_PIN2, HIGH);
    digitalWrite(SCK_PIN2, LOW);

    if (count & 0x800000) {
        count |= 0xFF000000;
    }

    return count;
}

void sendCommand(String command) {
    LORA.write(command.c_str());
    LORA.write("\r\n");
    delay(20);

    while (LORA.available()) {
        String response = LORA.readString();
        Serial.print("Response: ");
        Serial.println(response);
    }
}

void ABP_mode() {
  sendCommand("AT+CJOINMODE=1");
  delay(100);
  sendCommand("AT+CDEVADDR=007E6AE1");
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

void setupTransmitter(int T_address) {
    sendCommand("AT+CTXADDRSET=" + String(T_address));
    sendCommand("AT+CADDRSET=2");
    sendCommand("AT+CTX=868000000,5,0,1,21,1");
}

void transmitCompactData1(uint16_t deviceID, int16_t adcAverage,uint8_t batteryPercent, int16_t weight_g) {


    uint8_t payload[8];

    payload[0] = (deviceID >> 8) & 0xFF;
    payload[1] = deviceID & 0xFF;

    payload[2] = (adcAverage >> 8) & 0xFF;
    payload[3] = adcAverage & 0xFF;

    payload[4] = batteryPercent;  // 0–100 %
    payload[5] = 0x00;            // reserved

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

void transmitCompactData2(uint16_t deviceID,int16_t adcAverage,uint8_t batteryPercent,int16_t weight_g) {


    uint8_t payload[8];

    payload[0] = (deviceID >> 8) & 0xFF;
    payload[1] = deviceID & 0xFF;

    payload[2] = (adcAverage >> 8) & 0xFF;
    payload[3] = adcAverage & 0xFF;

    payload[4] = batteryPercent;  // 0–100 %
    payload[5] = 0x00;            // reserved


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

void setup() {
    Serial.begin(115200);
    LORA.begin(9600);

    Serial.println("Initializing transmitter...");
    delay(5000);

    pinMode(HB_LED, OUTPUT);
    pinMode(LOADCELL_EN, OUTPUT);
    pinMode(LORA_EN, OUTPUT);
    pinMode(LRESET, OUTPUT);
    pinMode(BOOT, OUTPUT);
    pinMode(LCD_FB, INPUT_PULLUP);
    pinMode(TARE_BTN, INPUT_PULLUP);
    pinMode(SPAN_BTN, INPUT_PULLUP);
    pinMode(SWITCHER_BTN, INPUT_PULLUP);  // changed pin here
    Wire.setSDA(PB9);
    Wire.setSCL(PB8);
    Wire.begin();
    digitalWrite(BOOT, LOW);
    delay(200);

    digitalWrite(HB_LED, LOW);
    digitalWrite(LOADCELL_EN, HIGH);
    digitalWrite(LORA_EN, HIGH);

    digitalWrite(LRESET, LOW);
    delay(1000);
    digitalWrite(LRESET, HIGH);
    delay(100);

    while (LORA.available()) {
        Serial.print(LORA.read());
    }

    delay(3000);
    // setupTransmitter(2);
    pinMode(DOUT_PIN1, INPUT);
    pinMode(SCK_PIN1, OUTPUT);
    digitalWrite(SCK_PIN1, LOW);

    pinMode(DOUT_PIN2, INPUT); // fixed duplicated line
    pinMode(SCK_PIN2, OUTPUT);
    digitalWrite(SCK_PIN2, LOW);

    // sendCommand("AT?");  // Send hex data over LoRa
    delay(2000);
    ABP_mode();

    for (int thisReading1 = 0; thisReading1 < numReadings1; thisReading1++) {
        readings1[thisReading1] = 0;
    }

    for (int thisReading2 = 0; thisReading2 < numReadings2; thisReading2++) {
        readings2[thisReading2] = 0;
    }
    loadCalibration();  // load saved tare/span
    weight_ingrams1(average1);
    weight_ingrams2(average2);

    lastWeight1 = finalweight1;
    lastWeight2 = finalweight2;


}

void loop() {
    data_raw1 = readHX710B1();
    average1 = long(fir1(data_raw1)) >> 5;
    data_raw2 = readHX710B2();
    average2 = long(fir2(data_raw2)) >> 5;
    battADC = analogRead(BLVL_PIN);

    if (battADC >= ADC_BATT_FULL) {
        batteryPercent = 100;
}
    else if (battADC <= ADC_BATT_EMPTY) {
        batteryPercent = 0;
}
    else {
        batteryPercent = map(battADC, ADC_BATT_EMPTY, ADC_BATT_FULL, 0, 100);
}


    weight_ingrams1(average1);
    weight_ingrams2(average2);
// ---- First run init ----
if (firstRun) {
    lastWeight1 = finalweight1;
    lastWeight2 = finalweight2;
    lastStableWeight1 = finalweight1;
    lastStableWeight2 = finalweight2;
    firstRun = false;
}
// ---- CH1: detect large change ----
if (!waitingForStability1 && abs(finalweight1 - lastWeight1) >= WEIGHT_DELTA) {
    waitingForStability1 = true;
    stableStartTime1 = millis();
    lastStableWeight1 = finalweight1;
}

// ---- CH1: wait for stability ----
if (waitingForStability1) {

    if (abs(finalweight1 - lastStableWeight1) >= WEIGHT_DELTA) {
        stableStartTime1 = millis();          // reset timer
        lastStableWeight1 = finalweight1;
    }

    if (millis() - stableStartTime1 >= STABLE_TIME_MS) {
        weightChanged1 = true;
        waitingForStability1 = false;
    }
}
// ---- CH2: detect large change ----
if (!waitingForStability2 && abs(finalweight2 - lastWeight2) >= WEIGHT_DELTA) {
    waitingForStability2 = true;
    stableStartTime2 = millis();
    lastStableWeight2 = finalweight2;
}

// ---- CH2: wait for stability ----
if (waitingForStability2) {

    if (abs(finalweight2 - lastStableWeight2) >= WEIGHT_DELTA) {
        stableStartTime2 = millis();          // reset timer
        lastStableWeight2 = finalweight2;
    }

    if (millis() - stableStartTime2 >= STABLE_TIME_MS) {
        weightChanged2 = true;
        waitingForStability2 = false;
    }
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

bool tareNow = digitalRead(TARE_BTN);

if (lastTareBtn == HIGH && tareNow == LOW) {
    delay(50); // debounce
    if (!digitalRead(TARE_BTN)) {

        if (calibrateChannel2) {
            tare2 = average2;
            lcd.clear();
            lcd.print("TARE2 SET");
            delay(300);
        } else {
            tare1 = average1;
            lcd.clear();
            lcd.print("TARE1 SET");
            delay(300);
        }

        saveCalibration();
        delay(300); // user feedback
    }
}
lastTareBtn = tareNow;

bool spanNow = digitalRead(SPAN_BTN);

if (lastSpanBtn == HIGH && spanNow == LOW) {
    delay(50);
    if (!digitalRead(SPAN_BTN)) {

        if (calibrateChannel2) {
            if (abs(average2 - tare2) > 200) {   
                span2 = average2;
                lcd.clear();
                lcd.print("SPAN2 SET");
                delay(300);
                saveCalibration();
            }
        }
             else {
                span1 = average1;
                lcd.clear();
                lcd.print("SPAN1 SET");
                delay(300);
                saveCalibration();
        }

}
}

lastSpanBtn = spanNow;


bool switchNow = digitalRead(SWITCHER_BTN);

if (lastSwitcherBtn == HIGH && switchNow == LOW) {
    delay(50);
    if (!digitalRead(SWITCHER_BTN)) {

        calibrateChannel2 = !calibrateChannel2;
        lcd.clear();
        lcd.print(calibrateChannel2 ? "CAL: CH 2" : "CAL: CH 1");

        delay(300);
    }
}
lastSwitcherBtn = switchNow;



lcd.clear();
if (!calibrateChannel2) {
    lcd.setCursor(0, 0);
    lcd.print("ADC1:");
    lcd.print(average1);

    lcd.setCursor(0, 1);
    lcd.print("W1:");
    lcd.print(finalweight1);
    lcd.print("g");
} else {
    lcd.setCursor(0, 0);
    lcd.print("ADC2:");
    lcd.print(average2);

    lcd.setCursor(0, 1);
    lcd.print("W2:");
    lcd.print(finalweight2);
    lcd.print("g");
}

    }

    Serial.println("data_raw1 : " + String(data_raw1) +
                   ", average1 : " + String(average1) +
                   ", data_raw2 : " + String(data_raw2) +
                   ", average2 : " + String(average2) +
                   ", motion : " + String(motion) +
                   ", processRun : " + String(processRun) +
                   ", BLVL : " + String(BLVL));

    tmr = millis() - ptmr;

// -------- LoRa TX  --------
static uint32_t lastTX = 0;

if (millis() - lastTX > 1000) {

    if (weightChanged1) {
        transmitCompactData1(currentID1, average1, batteryPercent, finalweight1);
        lastWeight1 = finalweight1;
        weightChanged1 = false;
        lastTX = millis();
        Serial.println("LoRa TX: CH1");
    }
    else if (weightChanged2) {
        transmitCompactData2(currentID2, average2, batteryPercent, finalweight2);
        lastWeight2 = finalweight2;
        weightChanged2 = false;
        lastTX = millis();
        Serial.println("LoRa TX: CH2");
    }
}


}