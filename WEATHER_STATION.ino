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

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

#define I2C_SDA_PIN 23
#define I2C_SCL_PIN 22

#define SERIAL1_RXPIN 16
#define SERIAL1_TXPIN 13

#define CCS811_ADDR 0x5A

#define AS3935_IRQ_PIN 27

#define SPEECH_TRIGGER_PIN 2

#define BusyState 19 // пин BUSY плеера
#define c19 19
#define c100 29
#define c1000 38
#define odna 41
#define dve 42

#define touchThreshold 55

bool fl;
char ccc[3];
byte troyka [3];

static char string_4dig[10];

struct WEATHER_INFO
{
  char signature[12];
  long temperature;
  long pressure;
  long humidity;
  long co2;
  long tvoc;
};


Adafruit_BME280 bme; // I2C

JQ6500_Serial mp3(&Serial1);

RH_ASK transmitter(2000,26,25,27);

CCS811 ccs811(CCS811_ADDR);



ClickEncoder encoder(35, 34, 32);
ClickEncoderStream encStream(encoder,1);

#define GFX_CLK  4
#define GFX_DIN  16
#define GFX_DC  17
#define GFX_CS  18
#define GFX_RST 5

//static PCD8544 lcd=PCD8544(GFX_CLK,GFX_DIN,GFX_DC,GFX_RST,GFX_CS);

static U8G2_UC1701_MINI12864_F_4W_SW_SPI u8g2(U8G2_R2, GFX_CLK, GFX_DIN, GFX_CS, GFX_DC , GFX_RST);


boolean touchedT2 = false;
boolean interrupt_AS3935 = false;

int transmitCounter = 0;


hw_timer_t * timer = NULL;

Preferences preferences;

int temperatureCalibration=0;
int soundVolume=15;
int radioTransmission=0;
int lightningAlert=0;
boolean settingsChanged = false;




int noiseCount = 0;
int disturberCount = 0;
int lightningsCount = 0;
int distance = 0;

void doWork();
void printDistance();
double getTemperature();
double getPressure();
void printValues();
void sayLightning();
void sayValues();


void IRAM_ATTR timerIsr() {
  encoder.service();
}

void IRAM_ATTR updateSettings()
{
  Serial.println("Saving settings...");
  timerDetachInterrupt(timer);
  timerAlarmDisable(timer);
  transmitter.stopInterrupts();

  preferences.begin("wst", false);
  preferences.putInt("volume", soundVolume);
  preferences.putInt("tCal", temperatureCalibration);
  preferences.putInt("radio", radioTransmission);
  preferences.putInt("lightning", lightningAlert);
  preferences.end();
  mp3.setVolume(soundVolume);
  settingsChanged = false;

  transmitter.resumeInterrupts();
  timerAttachInterrupt(timer, &timerIsr, true);
  timerAlarmEnable(timer);
  Serial.println("Saved settings");
}

void loadSettings()
{
  preferences.begin("wst", false);
  soundVolume = preferences.getInt("volume", 10);
  temperatureCalibration = preferences.getInt("tCal", 0);
  radioTransmission = preferences.getInt("radio", 0);
  lightningAlert = preferences.getInt("lightning", 0);
  preferences.end();
}

result IRAM_ATTR doSave()
{
  settingsChanged = true;
  return proceed;
}

void IRAM_ATTR onAS3935IRQ()
{
  interrupt_AS3935 = true;
}

TOGGLE(lightningAlert,setLightningAlert," Lightning: ",doNothing,noEvent,noStyle
  ,VALUE("On",1,doNothing,noEvent)
  ,VALUE("Off",0,doNothing,noEvent)
);

MENU(mainMenu, "SETTINGS", Menu::doNothing, Menu::noEvent, Menu::wrapStyle
  ,FIELD(temperatureCalibration," T.cal.","",-100,100,1,1, Menu::doNothing, Menu::noEvent, Menu::noStyle)
  ,FIELD(soundVolume," Volume","",0,100,1,1,Menu::doNothing, Menu::noEvent, Menu::noStyle)
  ,FIELD(radioTransmission," Radio:","",0,100,1,1,Menu::doNothing, Menu::noEvent, Menu::noStyle)
  ,SUBMENU(setLightningAlert)
  ,OP(" SAVE SETTINGS",doSave,enterEvent)
  ,EXIT(" EXIT")
);


MENU_INPUTS(in,&encStream);

#define MAX_DEPTH 2

#define fontName u8g2_font_7x13B_mf
#define fontX 7
#define fontY 15
#define offsetX 0
#define offsetY 0
#define U8_Width 128
#define U8_Height 64

const colorDef<uint8_t> colors[] MEMMODE={
  {{0,0},{0,1,1}},//bgColor
  {{1,1},{1,0,0}},//fgColor
  {{1,1},{1,0,0}},//valColor
  {{1,1},{1,0,0}},//unitColor
  {{0,1},{0,0,1}},//cursorColor
  {{1,1},{1,0,0}},//titleColor
};

//--------------------------------- SENSOR READINGS -------------------
double sensor_Temperature = -1;
double sensor_Pressure = -1;
double sensor_Humidity = -1;
double sensor_CO2 = -1;
double sensor_TVOC = -1;
//---------------------------------------------------------------------


MENU_OUTPUTS(out,MAX_DEPTH
  ,U8G2_OUT(u8g2,colors,fontX,fontY,offsetX,offsetY,{0,0,U8_Width/fontX,U8_Height/fontY})
  //,SERIAL_OUT(Serial)
  ,NONE
);

NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);//the navigation root object


void IRAM_ATTR gotTouch()
{
    touchedT2 = true;
}

void setup() {
    Serial.begin(9600);

    pinMode(I2C_SDA_PIN,INPUT_PULLUP);
    pinMode(I2C_SCL_PIN,INPUT_PULLUP);
    pinMode(BusyState,INPUT);

    Serial1.begin(9600, SERIAL_8N1, SERIAL1_RXPIN, SERIAL1_TXPIN);
    Wire.begin( I2C_SDA_PIN, I2C_SCL_PIN);

    loadSettings();

    u8g2.begin();
    u8g2.setPowerSave(0);
    u8g2.setContrast(150);
    u8g2.setDisplayRotation(U8G2_R2);

    if (!transmitter.init())
      Serial.println("transmitter init failed");

    mp3.reset();
    mp3.setVolume(soundVolume);
    mp3.setLoopMode(MP3_LOOP_NONE);
    mp3.sleep();

    bool status;

    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin(0x76, &Wire);
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        //while (1);
    }

    CCS811Core::status returnCode = ccs811.begin();
    if (returnCode != CCS811Core::SENSOR_SUCCESS)
    {
       Serial.println("Error CCS811 init");
    }

    mod1016.init(AS3935_IRQ_PIN);
    autoTuneCaps(AS3935_IRQ_PIN);
    mod1016.setIndoors();
    mod1016.setNoiseFloor(5);

    Serial.println("TUNE\tIN/OUT\tNOISEFLOOR");
    Serial.print(mod1016.getTuneCaps(), HEX);
    Serial.print("\t");
    Serial.print(mod1016.getAFE(), BIN);
    Serial.print("\t");
    Serial.println(mod1016.getNoiseFloor(), HEX);
    Serial.print("\n");

    pinMode(AS3935_IRQ_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(AS3935_IRQ_PIN), onAS3935IRQ, RISING);
    mod1016.getIRQ();
    Serial.println("after interrupt");

    pinMode(SPEECH_TRIGGER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(SPEECH_TRIGGER_PIN), gotTouch, FALLING);
    //touchAttachInterrupt(T2, gotTouch, touchThreshold);


    timer = timerBegin(1, 80, true);
    timerAttachInterrupt(timer, &timerIsr, true);
    timerAlarmWrite(timer, 1000, true);
    timerAlarmEnable(timer);

    nav.idleOn();
}


int16_t last, value;

void loop() {
//    value += encoder.getValue();
//
//  if (value != last) {
//    last = value;
//    Serial.print("Encoder Value: ");
//    Serial.println(value);
//  }
//
//  return;


  nav.doInput();
  if (nav.changed(0)) {//only draw if menu changed for gfx device
    //change checking leaves more time for other tasks
    u8g2.firstPage();
    do nav.doOutput(); while(u8g2.nextPage());
  }

  if (nav.sleepTask)
  {
      doWork();
  }
}


void updateBME280()
{
  sensor_Temperature = getTemperature();
  sensor_Pressure = getPressure();
  sensor_Humidity = bme.readHumidity();
}

void updateCCS811()
{
  if (ccs811.dataAvailable())
    {
      ccs811.readAlgorithmResults();
      sensor_CO2 = ccs811.getCO2();
      sensor_TVOC = ccs811.getTVOC();
      ccs811.setEnvironmentalData(round(sensor_Humidity), sensor_Temperature);
    }
}

void translateIRQ(uns8 irq) {
  switch(irq) {
      case 1:
        Serial.println("NOISE DETECTED");
        noiseCount++;
        break;
      case 4:
        Serial.println("DISTURBER DETECTED");
        disturberCount++;
        break;
      case 8:
        Serial.println("LIGHTNING DETECTED");
        lightningsCount++;
        printDistance();
        break;
    }
}

void printDistance() {
  distance = mod1016.calculateDistance();
  if (distance == -1)
    Serial.println("Lightning out of range");
  else if (distance == 1)
    Serial.println("Distance not in table");
  else if (distance == 0)
    Serial.println("Lightning overhead");
  else {
    Serial.print("Lightning ~");
    Serial.print(distance);
    Serial.println("km away\n");
  }
}

void doWork()
{
    updateBME280();
    updateCCS811();

    printValues();

    if (interrupt_AS3935)
    {
      translateIRQ(mod1016.getIRQ());
      interrupt_AS3935 = false;

      if (mod1016.getIRQ() == 8 && lightningAlert == 1)
      {
        sayLightning();
      }
    }

    if (touchedT2)
    {
        if (millis() > 15000)
        {
          sayValues();
        }
        touchedT2 = false;
    }

    if (settingsChanged)
    {
      updateSettings();
    }

    WEATHER_INFO wi;

    strcpy(wi.signature, "WI_001_TEST");
    wi.temperature = round(sensor_Temperature);
    wi.pressure = round(sensor_Pressure);
    wi.humidity = round(sensor_Humidity);
    wi.co2 = round(sensor_CO2);
    wi.tvoc = round(sensor_TVOC);

    if (radioTransmission > 0)
    {
        if (transmitCounter < 0)
        {
          Serial.println("Transmitting...");
          for (int i = 0; i < radioTransmission; i++)
          {
              if (transmitter.send((uint8_t *)&wi, sizeof(WEATHER_INFO)))
              {
                  transmitter.waitPacketSent();
              }
              else
              {
                Serial.println("Transmitting error");
              }
          }

          Serial.println("Transmitted");

          transmitCounter = 60;
        }

        transmitCounter--;
    }

    delay(1000);
}

void sayValues()
{
  int temperature = (int)round(sensor_Temperature);
  int pressure = (int)round(sensor_Pressure);
  int humidity = (int)round(sensor_Humidity);
  int co2 = (int)round(sensor_CO2);
  int tvoc = (int)round(sensor_TVOC);

  ozv(55);
  voicedig(itoa(temperature, string_4dig, 10));
  sayUom(temperature, 49, 50, 51);

  ozv(56);
  voicedig(itoa(pressure, string_4dig, 10));
  sayUom(pressure, 43, 44, 45);

  ozv(57);
  voicedig(itoa(humidity, string_4dig, 10));
  sayUom(humidity, 46, 47, 48);

  ozv(58);
  voicedig(itoa(co2, string_4dig, 10));
  //sayUom(co2, 52, 53, 54);

  ozv(59);
  voicedig(itoa(tvoc, string_4dig, 10));
  //sayUom(tvoc, 52, 53, 54);
}

void sayLightning()
{
  distance = mod1016.calculateDistance();
  if (distance == 0)
  {
    ozv(60);
  }
  if (distance > 1)
  {
    ozv(60);
    voicedig(itoa(distance, string_4dig, 10));
    sayUom(distance, 61, 62, 63);
  }
}

double getTemperature()
{
  return bme.readTemperature() + temperatureCalibration/10.0;
}

double getPressure()
{
  return bme.readPressure() * 0.00750062;
}

void printValues() {
  u8g2.setFont(fontName);

  u8g2.setCursor(1,12);
  u8g2.print(sensor_Temperature, 1);
  u8g2.print(" ");
  u8g2.print("C");

  u8g2.setCursor(1,24);
  u8g2.print(sensor_Pressure, 0);
  u8g2.print(" mmHg");

  u8g2.setCursor(1,36);
  u8g2.print(sensor_Humidity, 1);
  u8g2.println(" %");

  u8g2.setCursor(1,48);
  u8g2.print("CO2: ");
  u8g2.print(sensor_CO2, 0);
  u8g2.println(" ppm    ");

  u8g2.setCursor(1,60);
  u8g2.print("TVOC: ");
  u8g2.print(sensor_TVOC, 0);
  u8g2.println(" ppb    ");

  u8g2.sendBuffer();

  Serial.println(sensor_Temperature);
  Serial.println(sensor_Pressure);
  Serial.println(sensor_Humidity);
  Serial.println(sensor_CO2);
  Serial.println(sensor_TVOC);
}


void ozv(int myfile)
{
  mp3.playFileByIndexNumber(myfile);
  delay(500);
  while(digitalRead(BusyState));
}


void sayUom(int value, int file1, int file2, int file3)
{
  byte a=value%100;
  if (a>19) a=value%10;
  if (a==1) ozv(file1);
  else if (a>0 && a<5) ozv(file2);
  else ozv(file3);
}
//------------------------------------------------------------

void voicedig(char cc[])
{
  int a,b,c,d,jj,sme,dp;
  a=strlen(cc);
  for (byte i=0;i<3;i++) ccc[i]=0;
  b=a%3;c=a/3;jj=0;
  for (byte i=0;i<c+1;i++)
    {strncpy(ccc,cc+jj,b);
     d=atoi(ccc); a=d;
     for (byte i=0;i<3;i++)
       { troyka[2-i]=a%10;
         a=a/10;
       }
    if (d>0)
     { dp=troyka[2];
       if (c-i==1)
       {
        if (troyka[2]==1) dp=odna;
          else if(troyka[2]==2) dp=dve;
       }
       if (troyka[0]>0) ozv(c100+troyka[0]-1);
       if (troyka[1]>1) ozv(c19+troyka[1]);
          else if (troyka[1]==1)
                  {ozv(troyka[1]*10+troyka[2]+1);
                   goto m1;
                  }
       if (troyka[2]>0) ozv(dp+1);
       m1: a=d%100;
        if (a>19) a=d%10;
        if (a==1) sme=0; else
        if (a>1 && a<5) sme=1; else sme=2;
       if (c-i>0) ozv(c1000+(c-i-1)*3+sme);
      }
      jj=jj+b;b=3;
      delay(100);
    }
}
