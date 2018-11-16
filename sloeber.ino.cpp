#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2018-11-16 10:21:14

#include "Arduino.h"
#include <ClickEncoder.h>
#include <RH_ASK.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DigisparkJQ6500.h>
#include "SparkFunCCS811.h"
#include "AS3935_mod1016.h"
#include <U8g2lib.h>
#include <driver/gpio.h>
#include <Preferences.h>
#include <menu.h>
#include <menuIO/u8g2Out.h>
#include <menuIO/clickEncoderIn.h>
#include <menuIO/chainStream.h>

void IRAM_ATTR timerIsr() ;
void IRAM_ATTR updateSettings() ;
void loadSettings() ;
result IRAM_ATTR doSave() ;
void IRAM_ATTR onAS3935IRQ() ;
void IRAM_ATTR gotTouch() ;
void setup() ;
void loop() ;
void updateBME280() ;
void updateCCS811() ;
void translateIRQ(uns8 irq) ;
void printDistance() ;
void doWork() ;
void sayValues() ;
void sayLightning() ;
double getTemperature() ;
double getPressure() ;
void printValues() ;
void ozv(int myfile) ;
void sayUom(int value, int file1, int file2, int file3) ;
void voicedig(char cc[]) ;

#include "WEATHER_STATION.ino"


#endif
