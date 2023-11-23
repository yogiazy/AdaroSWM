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

#define THRESHOLD      100  //100
#define RedLED    PB10 // paling ujung  
#define YellowLED    PB11 // kedua dr paling ujung  
#define GreenLED     PB12 // ketiga dr ujung 
 
bool stat  = false;
bool stat2 = false;
bool stat3 = false;
void blinkLed(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED, LOW);
    delay(20);
    digitalWrite(LED, HIGH);
    delay(20);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RedLED, OUTPUT);
  pinMode(YellowLED, OUTPUT);
  pinMode(GreenLED, OUTPUT);
  pinMode(SENSOR, INPUT_ANALOG);        // Set pin SENSOR sebagai input analog
  pinMode(SENSOR_ON, OUTPUT_OPEN_DRAIN); // Set pin SENSOR_ON sebagai output open-drain
  digitalWrite(SENSOR_ON, LOW);         // Matikan pin SENSOR_ON
  digitalWrite(RedLED, LOW);
  digitalWrite(YellowLED, HIGH);
  digitalWrite(GreenLED, HIGH);
  blinkLed(2);
  delay(1000);
}

void loop() {
  int analogIn = analogRead(SENSOR);
  Serial.println(analogIn);
  if (analogIn > THRESHOLD) { // Saat Nilai Sensor HIGH Green NYALA
    digitalWrite(GreenLED, LOW);
    digitalWrite(YellowLED, HIGH);
  } else if(analogIn <= THRESHOLD){  // Saat Nilai Sensor LOW GREEN NYALA
    digitalWrite(YellowLED, LOW);
    digitalWrite(GreenLED, HIGH);
  }
  delay(100);
}
