/*********************

Relay switching temp controller & interface.
alex babb - 2016

NOTES:
- you have both the lcd shield and the RTC on the same i2c port. they seem
  to be colliding in some nasty way, although they seem to have different
  addresses (0x20 vs 0x68). 
  - Causes extremely weird behavior, for example, adding an RTC.now() call
    to a statement that NEVER GETS EXECUTED causes behavior changes.
  - Post to forums, maybe
  - Try this on the Uno. maybe it's a mega thing. you'll have to change the
    SD calls, and potentially roll back the associated lib changes.

TODO:
- add an input file parser
- add the time-in-profile page
- implement profiled tgt temp
  

**********************/

//--------------------------------------------------------------------------
// LIBS
//--------------------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"


//--------------------------------------------------------------------------
// STATIC DECLARATIONS
//--------------------------------------------------------------------------

// TEMP SENSOR
// Data wire is plugged into port 22 on the Arduino Mega
// Sensor power on 24
// Sensor ground on 26
#define TEMP_DAT 22
#define TEMP_PWR 24
#define TEMP_GND 26

// WALL POWER RELAY
#define RELAY 52

// OneWire class for Temp Sensor
OneWire oneWire(TEMP_DAT);
// Temp Sensor class
DallasTemperature sensors(&oneWire);

// DISPLAY SHIELD
// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share
// the I2C bus.
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

// number of pages i've defined
#define N_PAGES 5
#define N_CTRL_TYPES 2

// DATA LOGGER
bool sdcard;
File outfile;
File inpfile;
RTC_DS1307 RTC;

// PROGRAM VARIABLES
float temp_cur;
float temp_tgt;
float deadband;
bool relay_active;
int pg_cur;
int ctrl_type;
DateTime tnow;
long prfl_time[100];
float prfl_temp[100];
size_t prfl_len;

//--------------------------------------------------------------------------
// HELPER FUNCTIONS
//--------------------------------------------------------------------------

// Read Temp Sensor Measurement and Print to LCD
void performTempSensorProcessing()
{ 
  sensors.requestTemperatures(); // Send the command to get temperatures
  temp_cur = sensors.getTempFByIndex(0);
}

void printTempCur()
{
  // Reset the display  
  //lcd.clear();
  lcd.home();

  static char tcur_buffer [6];
  // sprintf(tcur_buffer,"Temp: %f F", 2.234);
  dtostrf(temp_cur, 5, 1, tcur_buffer);

  //  Serial.print("temp: ");
  //  Serial.print(tcur_buffer);
  //  Serial.println(" F");
  
  // Print our characters on the LCD
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print("Tmp: ");
  lcd.print(tcur_buffer);
  lcd.print(" F");
}

void printTempTgt()
{
  // also print target temp
  lcd.setCursor(0,1);
  lcd.print("Tgt: ");
  static char ttgt_buffer [6];
  // sprintf(buffer,"Temp: %f F", 2.234);
  dtostrf(temp_tgt, 5, 1, ttgt_buffer);
  lcd.print(ttgt_buffer);
  lcd.print(" F");
}

void tempControlHeat()
{
  //---------------------
  // Heater Control Logic
  //---------------------
  if (temp_cur < (temp_tgt - deadband))
  {
    digitalWrite(RELAY,HIGH);
    relay_active = true;
  }
  else if (temp_cur > (temp_tgt + deadband))
  {
    digitalWrite(RELAY,LOW);
    relay_active = false;
  }
  else // in hysteresis range
  {
    if (relay_active)
    {
      digitalWrite(RELAY,HIGH);
    }
    else
    {
      digitalWrite(RELAY,LOW);
    }
  }
}

void tempControlCool()
{
  //---------------------
  // Cooler Control Logic
  //---------------------
  if (temp_cur > (temp_tgt + deadband))
  {
    digitalWrite(RELAY,HIGH);
    relay_active = true;
  }
  else if (temp_cur < (temp_tgt - deadband))
  {
    digitalWrite(RELAY,LOW);
    relay_active = false;
  }
  else // in hysteresis range
  {
    if (relay_active)
    {
      digitalWrite(RELAY,HIGH);
    }
    else
    {
      digitalWrite(RELAY,LOW);
    }
  }
}

void tgtTempFromProfile()
{
  if (ctrl_type == 2 && prfl_len > 0)
  {
    for (size_t i=0; i<prfl_len; i++)
    {
      if (long(tnow.unixtime()) >= long(prfl_time[i]))
        temp_tgt = prfl_temp[i];
      else
        break;
    }
  }
}

void displayPage()
{
  if (pg_cur == 1)
  {
    printTempCur();
    printTempTgt();
    if (relay_active)
    {
      lcd.setCursor(13,1);
      lcd.print(" ON");
    }
    else
    {
      lcd.setCursor(13,1);
      lcd.print("OFF");
    }
  }
  else if (pg_cur == 2)
  {
    lcd.setCursor(0,0);
    lcd.print("deadband: ");
    static char db_buf [4];
    dtostrf(deadband, 3, 1, db_buf);
    lcd.print(db_buf);
  }
  else if (pg_cur == 3)
  {
    lcd.setCursor(0,0);
    lcd.print("Control type: ");

    lcd.setCursor(0,1);
    if (ctrl_type == 1)
      lcd.print("manual");
    else if (ctrl_type == 2)
      lcd.print("SD profile");
  }
  else if (pg_cur == 4)
  {
    // uncommenting these lines causes arduino to continuously reset, 
    // even though the "flag" is never printed indicating this 
    // statement is not entered. how is this possible?!
//    Serial.println("flag");
//    DateTime now = RTC.now();

    // year
    static char yr_buf [5];
    dtostrf(tnow.year(),2,0,yr_buf);
    lcd.setCursor(0,0);
    lcd.print(yr_buf);

    // month
    static char mon_buf [3];
    dtostrf(tnow.month(),2,0,mon_buf);
    if (tnow.month() < 10)
      mon_buf[0] = '0';
    lcd.setCursor(4,0);
    lcd.print("/");
    lcd.setCursor(5,0);
    lcd.print(mon_buf);

    // day
    static char day_buf [3];
    dtostrf(tnow.day(),2,0,day_buf);
    if (tnow.day() < 10)
      day_buf[0] = '0';
    lcd.setCursor(7,0);
    lcd.print("/");
    lcd.setCursor(8,0);
    lcd.print(day_buf);

    // hour
    static char hr_buf [3];
    dtostrf(tnow.hour(),2,0,hr_buf);
    if (tnow.hour() < 10)
      hr_buf[0] = '0';
    lcd.setCursor(0,1);
    lcd.print(hr_buf);

    // min
    static char min_buf [3];
    dtostrf(tnow.minute(),2,0,min_buf);
    if (tnow.minute() < 10)
      min_buf[0] = '0';
    lcd.setCursor(2,1);
    lcd.print(":");
    lcd.setCursor(3,1);
    lcd.print(min_buf);

    // sec
    static char sec_buf [3];
    dtostrf(tnow.second(),2,0,sec_buf);
    if (tnow.second() < 10)
      sec_buf[0] = '0';
    lcd.setCursor(5,1);
    lcd.print(":");
    lcd.setCursor(6,1);
    lcd.print(sec_buf);
  
  }
  else if (pg_cur == 5)
  {
    static char tnix_buf [12];
    String tmp_str0(tnow.unixtime());
    tmp_str0.toCharArray(tnix_buf,11);
      
    lcd.setCursor(0,0);
    lcd.print("tnix: ");
    lcd.setCursor(6,0);
    lcd.print(tnix_buf);

    // also print time in profile
    if (prfl_len > 0)
    {
      long TiP = long(tnow.unixtime()) - long(prfl_time[0]);
      String tmp_str1(TiP);
      static char tip_buf [12];
      tmp_str1.toCharArray(tip_buf,11);
      lcd.setCursor(0,1);
      lcd.print("TiP: ");
      lcd.setCursor(6,1);
      lcd.print(tip_buf);
    }
  }
  else
  {
    lcd.setCursor(0,0);
    lcd.print("NO PAGE DATA");
  }
}

void handleButtons(uint8_t buttons)
{
  if (buttons & BUTTON_UP) {
    if (pg_cur == 1)
      temp_tgt += 0.5;
    else if (pg_cur == 2)
      deadband += 0.5;
    else if (pg_cur == 3)
    {
      ctrl_type += 1;
      if (ctrl_type > N_CTRL_TYPES)
        ctrl_type = 1;
    }
  }
  else if (buttons & BUTTON_DOWN) {
    if (pg_cur == 1)
      temp_tgt -= 0.5;
    else if (pg_cur == 2)
    {
      deadband -= 0.5;
      if (deadband < 0)
        deadband = 0;
    }
    else if (pg_cur == 3)
    {
      ctrl_type -= 1;
      if (ctrl_type < 1)
        ctrl_type = N_CTRL_TYPES;
    }
  }

  // left/right always traverses pages
  if (buttons & BUTTON_RIGHT) {
    if (pg_cur == N_PAGES)
      pg_cur = 1;
    else
      pg_cur += 1;
  }
  else if (buttons & BUTTON_LEFT) {
    if (pg_cur == 1)
      pg_cur = N_PAGES;
    else
      pg_cur -= 1;
  }
}

void dumpFile()
{
  if (inpfile) {
    Serial.println("dumping file");
    while (inpfile.available()) {
      Serial.write(inpfile.read());
    }
    inpfile.close();
  }
}

bool readLine(File &f, char* line, size_t maxLen) {
  for (size_t n = 0; n < maxLen; n++) {
    int c = f.read();
    if ( c < 0 && n == 0) return false;  // EOF
    if (c < 0 || c == '\n') {
      line[n] = 0;
      return true;
    }
    line[n] = c;
  }
  return false; // line too long
}

bool readVals(long* v1, float* v2) {
  char line[40], *ptr, *str;
  if (!readLine(inpfile, line, sizeof(line))) {
    return false;  // EOF or too long
  }
  *v1 = strtol(line, &ptr, 10);
  if (ptr == line) return false;  // bad number if equal
  while (*ptr) {
    if (*ptr++ == ',') break;
  }
  *v2 = strtod(ptr, &str);
  return str != ptr;  // true if number found
}

void updateDatFile()
{
    String str = "";
    outfile.print(tnow.unixtime());   outfile.print(",");
    outfile.print(temp_cur);          outfile.print(",");
    outfile.print(temp_tgt);          outfile.print(",");
    outfile.print(deadband);          outfile.print(",");
    outfile.print((int)relay_active); outfile.print(",");
    outfile.println();
    outfile.flush();
}

//--------------------------------------------------------------------------
// SETUP() AND LOOP() FUNCTIONS
//--------------------------------------------------------------------------

// Global setup -- Run once
void setup() {
  // power temp sensor
  pinMode(TEMP_PWR,OUTPUT);
  pinMode(TEMP_GND,OUTPUT);
  digitalWrite(TEMP_PWR,HIGH);
  digitalWrite(TEMP_GND,LOW);

  // Temp Control Initialization
  pinMode(RELAY,OUTPUT);
  digitalWrite(RELAY,LOW);
  relay_active = false;

  
  // Debugging output
  Serial.begin(9600);
  Serial.println("=========================================================");
  Serial.println("Initializing...");
  
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  
  lcd.setBacklight(WHITE);
  pg_cur = 1;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Initializing");
 
  // initialize control
  ctrl_type = 1;

  // initialize realtime clock
  RTC.begin();
  DateTime now = RTC.now();
  String timestr =  "Time: " +
                    String(now.year()) + "-" + 
                    String(now.month()) + "-" +
                    String(now.day()) + " " +
                    String(now.hour()) + ":" +
                    String(now.minute()) + ":" +
                    String(now.second());
  Serial.println(timestr);

  // initialize the SD card
  // make sure that the default chip select pin is set to
  // output, even if you don't use it
  // - for regular duinos, this is just pin 10
  // - for MEGA, this is 10, 11, 12, 13
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  
  // see if the card is present and can be initialized.
  // NOTE: the FAT filesystem limits filenames to 8 characters plus
  // 3-character extensions (stupid)
  if (!SD.begin(10,11,12,13)) {
    Serial.println("SD card failed, or not present. Ignoring.");
    sdcard = false;
  }
  else
  {
    Serial.println("SD card found.");
    Serial.println("writing to file: BEERLOG.CSV");
    outfile = SD.open("beerlog.csv", FILE_WRITE);
    sdcard = true;

    // Also attempt to read a pre-set temperature profile from
    // the SD card...
    inpfile = SD.open("INPUTS.CSV", FILE_READ);
    if (inpfile)
      Serial.println("Input file detected");
      // dumpFile();
    else
      Serial.println("Input file not found");

    // format is csv time, temp
    prfl_len = 0;
    while (readVals(&prfl_time[prfl_len], &prfl_temp[prfl_len]))
      prfl_len++;

    inpfile.close();
      
    Serial.println("stored profile:");
    for (size_t i=0; i<prfl_len; i++)
    {
      Serial.print(prfl_time[i]);
      Serial.print(", ");
      Serial.println(prfl_temp[i]);
    }
  }

  // Initialize temps
  temp_cur = 0;
  temp_tgt = 55.00; // arbitrary!
  deadband = 0.5; // arbitrary!
  performTempSensorProcessing();
  delay(1000); // temp sensor spits out garb for the first second or so
  performTempSensorProcessing();

  lcd.clear();
}

// Main loop
uint8_t i=0;
void loop() {
  // TODO: should make outfile update interval independently configurable.
  
  tnow = RTC.now();

  performTempSensorProcessing();
  tgtTempFromProfile();
  tempControlCool();
  displayPage();
  updateDatFile();
  
  // spend a second listening for buttons between updates. this is kinda dumb, 
  // but improves usability. hijacking millis interrupt might be a better solution.
  float t_listen = 2000;
  float t0 = millis();
  while( (millis() - t0) < t_listen ){
    uint8_t buttons = lcd.readButtons();
    if (buttons) {
      handleButtons(buttons);
      // display changes and chill out for a bit for debouncing
      lcd.clear();
      displayPage();
      delay(200);
    }
  }
}
