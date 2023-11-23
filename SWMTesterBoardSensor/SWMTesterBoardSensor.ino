#include <RadioLib.h>
#include "STM32LowPower.h"
#include "backup.h"
#include <ArduinoUniqueID.h>

//baru
#define HAL_PWR_MODULE_ENABLED
#define LED       PC13
#define NSS       PA4
#define DIO0      PB2
#define DIO1      PB1
#define LORA_RST  PC14
#define LORA_ON   PA11
#define SENSOR    PA0
#define SENSOR_ON PA1
//lama
//#define HAL_PWR_MODULE_ENABLED
//#define LED       PC13
//#define NSS       PA4
//#define DIO0      PB1
//#define DIO1      PA15
//#define LORA_RST  PB14
//#define LORA_ON   PA11
//#define SENSOR    PA0
//#define SENSOR_ON PA1

#define lowPin    PB10
#define inp       PB11
#define CONTROL_WORD   0
#define SAMPLING_REG   1
#define METER_REG      2
#define STA_REG        3
#define SAMPLING_RATE  100
#define UPDATE_RATE    36000 //600 1 jam
#define THRESHOLD_DOWN 100

SX1276 lora = new Module(NSS, DIO0, LORA_RST, DIO1);
uint32_t meterCounter;
uint32_t samplingCounter;
uint8_t st1;
int transmissionState =  ERR_NONE;
bool transmitFlag = false;
bool sendSta = false;
volatile bool enableInterrupt = true;
volatile bool operationDone = false;
volatile bool receivedFlag = false;

byte calculateCRC(byte ar[], byte s) {
  byte rtn = 0;;
  for (byte i = 0; i < s; i++) {
    rtn ^= ar[i];
  }
  return rtn;
}

void setFlag(void) {
  operationDone = true;
}

typedef struct {
  uint8_t       typeId; //
  uint32_t      nodeId; //store this nodeId
  uint8_t       battery; //type parameter
  uint32_t      cacah;   //temperature maybe?
} Payload;
Payload theData;

void blinkLed(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, LOW);
    delay(20);
    digitalWrite(LED, HIGH);
    delay(20);
  }
}

void sendDataSensor(uint8_t urate) {
  uint32_t DEVID;
  uint8_t Part1 = 0;
  uint8_t Part2 = 0;
  uint8_t Part3 = 0;
  uint8_t Part4 = 0;

  for (int i = 0; i <= 2; i++) { // 0 1 2
    Part1 += UniqueID[i];
  }
  for (int i = 3; i <= 5; i++) { // 3 4 5
    Part2 += UniqueID[i];
  }
  for (int i = 6; i <= 8; i++) { // 6 7 8
    Part3 += UniqueID[i];
  }
  for (int i = 9; i <= 11; i++) { // 9 10 11
    Part4 += UniqueID[i];
  }

  DEVID =  ( ((uint32_t)Part4) | ((uint32_t)Part3 << 8) | ((uint32_t)Part2 << 16) | ((uint32_t)Part1 << 24));

  int state = lora.begin(922.0, 125.0, 9, 7, SX127X_SYNC_WORD, 17, 8, 0);
  lora.setDio0Action(setFlag);
  uint32_t count = getBackupRegister(METER_REG);

  //  Serial.println(",value = "+String(valuee));
  theData.typeId = 8;
  theData.nodeId = DEVID;
  theData.battery = urate;
  theData.cacah = count;
  byte len = sizeof(theData);
  byte byteArray[len + 1];
  memcpy(byteArray, (const void*)(&theData), len);
  byteArray[len] = calculateCRC(byteArray, len);
  transmissionState = lora.startTransmit(byteArray, len + 1);
  if (state == ERR_NONE) {
    //If Success
    blinkLed(1);
  } else {
    //If Failed
    blinkLed(20);
    while (true);
  }
  transmitFlag = true;
}

void setup() {
//  Serial.begin(115200);
  enableBackupDomain();
  LowPower.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Konfigurasi pin dan inisialisasi
  pinMode(SENSOR, INPUT_ANALOG);        // Set pin SENSOR sebagai input analog
  pinMode(SENSOR_ON, OUTPUT_OPEN_DRAIN); // Set pin SENSOR_ON sebagai output open-drain
  digitalWrite(SENSOR_ON, LOW);         // Matikan pin SENSOR_ON
  delay(2); // Delay sementara
  uint32_t controlWord = getBackupRegister(CONTROL_WORD);
  if (controlWord == 0x01) {
    samplingCounter = getBackupRegister(SAMPLING_REG);
    meterCounter = getBackupRegister(METER_REG);
    st1 = getBackupRegister(STA_REG);
  } else {
    setBackupRegister(CONTROL_WORD, 0x01);
    setBackupRegister(SAMPLING_REG, 0);
    setBackupRegister(METER_REG, 0);
    setBackupRegister(STA_REG, 0);
    meterCounter = 0;
    st1 = 0;
    samplingCounter = 0;
  }
  int analogIn = analogRead(SENSOR);
  if (analogIn > THRESHOLD_DOWN) { // Saat Nilai Sensor HIGH
    st1 = 1;
  } else { // Saat Nilai Sensor LOW
    if (st1 == 1) {
      meterCounter += 1; // incre cacahan
      setBackupRegister(METER_REG, meterCounter);
      st1 = 0;
    }
  }
  setBackupRegister(STA_REG, st1);
//  Serial.print("input:");
//  Serial.print(analogIn);
//  Serial.print(",");
//  Serial.print("samp:");
//  Serial.print(samplingCounter);
//  Serial.print(",");
//  Serial.print("THRESHOLD:");
//  Serial.print(THRESHOLD_DOWN);
//  Serial.print(",");
//  Serial.print("cacahan:");
//  Serial.println(meterCounter);


  digitalWrite(SENSOR_ON, HIGH);
  samplingCounter = getBackupRegister(SAMPLING_REG);
  samplingCounter++;

  uint16_t urate = 0;
  pinMode(inp, INPUT_PULLUP);
  pinMode(lowPin, OUTPUT);
  digitalWrite(lowPin, LOW);
  if (!digitalRead(inp)) {
    urate = 10; // 1 detik(jika samplingrate 100);
  } else {
    urate = 36000;
  }
  if (samplingCounter >= urate) { // send data
    sendSta = true;
    samplingCounter = 0;
    pinMode(LED, OUTPUT);
    pinMode(LORA_ON, OUTPUT);
    digitalWrite(LED, HIGH);
    digitalWrite(LORA_ON, LOW);
    sendDataSensor(urate);
    while (!operationDone);
  }
  setBackupRegister(SAMPLING_REG, samplingCounter);

  if (operationDone) {
    operationDone = false;
    if (transmitFlag) {
      if (transmissionState ==  ERR_NONE) {
        // Transmisi Berhasil
      } else {
        // Transmisi gagal
      }
      digitalWrite(LORA_ON, HIGH);
      transmitFlag = false; //Kirim
    } else {
      transmitFlag = false; //Nerima
    }
  }
  if (!operationDone && !transmitFlag) {
    LowPower.shutdown(SAMPLING_RATE);
  }
  //  delay(100);
}
