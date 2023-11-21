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

#define RedLED    PB10 // paling ujung
#define BlueLED   PB11 // kedua dr ujung

SX1276 lora = new Module(NSS, DIO0, LORA_RST, DIO1);
bool transmitFlag = false;
volatile bool enableInterrupt = true;
volatile bool operationDone = false;
bool BlueActive = false;
unsigned long prevmillis = 0;
const long timeout = 5000;

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

void setup() {
  int state = lora.begin(920.0, 125.0, 9, 7, SX127X_SYNC_WORD, 17, 8, 0);
  lora.setDio0Action(setFlag);
  state = radio.startReceive();
  if (state == ERR_NONE) {
    //No Error
  } else {
    while (true);
  }
  pinMode(RedLED, OUTPUT);
  pinMode(BlueLED, OUTPUT);
  digitalWrite(RedLED, HIGH);
  digitalWrite(BlueLED, LOW);
}

void loop() {
  if (operationDone) {
    operationDone = false;
    if (transmitFlag) {
      transmitFlag = false; //Kirim
    } else {
      byte len = sizeof(theData);
      byte byteArray[len + 1];
      int state = radio.readData(byteArray, len + 1);
      if (state == ERR_NONE) {
        theData = *(Payload*)byteArray;
        if (byteArray[len] == calculateCRC(byteArray, len) && theData.typeId != 0 && theData.vParams > 0) {
          BlueActive = true;
          digitalWrite(RedLED, LOW);
          digitalWrite(BlueLED, HIGH);
        }
      }
      transmitFlag = false; //Nerima
    }
  }
  if (BlueActive) {
    unsigned long curmillis = millis();
    if (curmillis - prevmillis >= timeout) {
      // save the last time you blinked the LED
      prevmillis = curmillis;
      digitalWrite(RedLED, LOW);
      digitalWrite(BlueLED, HIGH);
      BlueActive = false;
    }
  }
  delay(100);
}
