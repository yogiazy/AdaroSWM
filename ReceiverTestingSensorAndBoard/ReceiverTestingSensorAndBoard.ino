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

#define RedLED       PB10 // paling ujung  
#define YellowLED    PB11 // kedua dr ujung 
#define GreenLED     PB12 // ketiga dr ujung

SX1276 lora = new Module(NSS, DIO0, LORA_RST, DIO1);
bool transmitFlag = false;
bool stat1 = false;
bool stat2 = false;
bool stat3 = false;
volatile bool enableInterrupt = true;
volatile bool operationDone = false;
unsigned long prevmillis = 0;
const long timeout = 5000;
int params = 0;
uint32_t nodeid = 0;


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
    delay(100);
    digitalWrite(LED, HIGH);
    delay(100);
  }
}
void blinkok1(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(RedLED, LOW);
    delay(100);
    digitalWrite(RedLED, HIGH);
    delay(100);
  }
  digitalWrite(RedLED, LOW);
}
void blinknotok1(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(RedLED, LOW);
    delay(100);
    digitalWrite(RedLED, HIGH);
    delay(100);
  }
}

void blinkok2(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(YellowLED, LOW);
    delay(100);
    digitalWrite(YellowLED, HIGH);
    delay(100);
  }
  digitalWrite(YellowLED, LOW);
}
void blinknotok2(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(YellowLED, LOW);
    delay(100);
    digitalWrite(YellowLED, HIGH);
    delay(100);
  }
}

void blinkokall(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(GreenLED, LOW);
    delay(100);
    digitalWrite(GreenLED, HIGH);
    delay(100);
  }
  digitalWrite(GreenLED, LOW);
}

void blinkall(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(RedLED, LOW);
    digitalWrite(YellowLED, LOW);
    digitalWrite(GreenLED, LOW);
    delay(100);
    digitalWrite(RedLED, HIGH);
    digitalWrite(YellowLED, HIGH);
    digitalWrite(GreenLED, HIGH);
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  pinMode(LORA_ON, OUTPUT);
  digitalWrite(LORA_ON, LOW);
  delay(10);
  int state = lora.begin(922.0, 125.0, 9, 7, SX127X_SYNC_WORD, 17, 8, 0);
  lora.setDio0Action(setFlag);
  if (state == ERR_NONE) {
    //No Error
    Serial.println("Not Error Lora");
  } else {
    Serial.println("Error Lora");
    while (true);
  }
  state = lora.startReceive();
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  pinMode(RedLED, OUTPUT);
  pinMode(YellowLED, OUTPUT);
  pinMode(GreenLED, OUTPUT);
  digitalWrite(RedLED, HIGH);
  digitalWrite(YellowLED, HIGH);
  digitalWrite(GreenLED, HIGH);
  Serial.println("Done Setup");
  blinkLed(2);
  blinkall(3);
  delay(100);
}

void loop() {
  if (operationDone) {
    operationDone = false;
    if (transmitFlag) {
      transmitFlag = false; //Kirim
    } else {
      Serial.println("------------- incoming Data");
      byte len = sizeof(theData);
      byte byteArray[len + 1];
      int state = lora.readData(byteArray, len + 1);
      if (state == ERR_NONE) {
        theData = *(Payload*)byteArray;
        if (byteArray[len] == calculateCRC(byteArray, len) && theData.typeId != 0) {
          stat1 = true;
          Serial.println("Data OK");
          blinkok1(1);
        } else {
          stat1 = false;
          blinknotok1(1);
        }
        if (theData.cacah > 0) {
          stat2 = true;
          Serial.println("Value OK");
          blinkok2(1);
        } else {
          stat2 = false;
          blinknotok2(1);
        }
        if (stat1 && stat2) {
          stat3 = true;
          Serial.println("OK ALL");
          nodeid = theData.nodeId;
          params = theData.cacah;
          blinkokall(1);
        } else {
          stat3 = false;
          blinkall(5);
        }
      }
      lora.startReceive();
      transmitFlag = false; //Nerima
    }
    blinkLed(2);
  }
  if (stat3) {
    delay(3000);
    blinkall(1);
    Serial.println(String(params));
    Serial.print("Node : ");
    Serial.println(nodeid,HEX);
    if (abs(params - 9) <= 2) {
      digitalWrite(RedLED, LOW);
      Serial.print("Cacahan 3");
      delay(10000);
    } else if (abs(params - 6) <= 2) {
      digitalWrite(YellowLED, LOW);
      Serial.print("Cacahan 2");
      delay(10000);
    } else if (abs(params - 3) <= 2) {
      digitalWrite(GreenLED, LOW);
      Serial.print("Cacahan 1");
      delay(10000);
    } else {
      Serial.println("Tidak ada kesimpulan Cacahan");
      blinkall(3);
    }
    digitalWrite(RedLED, HIGH);
    digitalWrite(YellowLED, HIGH);
    digitalWrite(GreenLED, HIGH);
    stat1 = false;
    stat2 = false;
    stat3 = false;
  }
  delay(100);
}
