String programName = "ardBeeQueenPID3";
String date = "201910012";
String author = "Jon Sagebrand";
String email = "jonsagebrand@gmail.com";

// enable plotting function
// all other output to serial will be suppressed
bool plot = 1;

/*******************************
   EEPROM setup
 *******************************/
// include EEPROM library
#include <EEPROM.h>
int eeAddr = 0; // address to store the set point temp

/*******************************
   DHT setup
 *******************************/
// include the DHT library
#include <DHT.h>
// define DHT pin
#define DHTPIN 2  // declare pin number
// uncomment one of the below sensors
//#define DHTTYPE DHT11   // DHT 11 used
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM230)
// set up DHT sensor
DHT dht(DHTPIN, DHTTYPE);  // initialize DHT sensor
// wait time between temp reads, in milliseconds
unsigned long readMillis; // stores millis
int waitTime = 2000; // millis to wait between reads
// variables
double hum = 0; // read humidity
double temp = 0; // read temperature
double dewPoint = 0; // calculated dew point
double dewPointFast = 0; // another calculated dew point
double hic = 0;

/*******************************
   Input/Output pins setup
 *******************************/
// heating relay
const int heatRelay = 3;  // relay heating
int heatRelayState;
int heatState = 0; // current state of operation
int heatStateLast = 0; // last state of operation
// fan relay
const int fanRelay = 4;  // relay cooling
int fanRelayState = LOW; // fan is always on/off
int fanState = 0; // current state of operation
int fanStateLast = 0; // last state of operation

// rotary encoder pins
const int encoderCLK = 5; // rotary encoder CLK signal input pin
const int encoderDT = 6; // rotary encoder DT signal input pin
const int encoderSW = 7; // rotary encoders switch input pin, goes LOW when pressed
// for storing encoder values
int encoderCLKState = 0;
int encoderDTState = 0;
int encoderSWState = 0;
int encoderDTStateLast = 0;
int encoderSWStateLast = 0;
// times for debounce
unsigned long encoderSWTimeMillis = 0; // the last time encoder button was toggled
int debounce = 200;   // the debounce time, increase if the output flickers

/*******************************
   LCD setup
 *******************************/
// include the LCD library
#include <LiquidCrystal.h>
// set LCD pins
int LCD_RS = 13; // LCD RS, pin 4
int LCD_EN = 12; // LCD E, pin 6, Enable
int LCD_D4 = 11; // LCD D4, pin 11, databit 4
int LCD_D5 = 10; // LCD D5, pin 12, databit 5
int LCD_D6 = 9; // LCD D6, pin 13, databit 6
int LCD_D7 = 8; // LCD D7, pin 14, databit7
/* other pins on LCD are:
 * VSS, pin 1, GND
 * VDD, pin 2, +5V
 * V0, pin 3, 10k potentiometer, ends to +5V and GND, middle pin to V0, contrast
 * R/W, pin 5, GND, Read/Write
 * A, pin 15, 220ohm resistor, one end to +5V, other to A, back light anode, ~+4.2V
 * K, pin 16, GND, back light cathode
 * D0, pin 7, databit 0, not used/connected
 * D1, pin 8, databit 1, not used/connected
 * D2, pin 9, databit 2, not used/connected
 * D3, pin 10, databit 3, not used/connected
 */
// initialize the LCD library
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
// columns and rows of the LCD
int lcdColumns = 16;
int lcdRows = 2;

/*******************************
   PID setup
 *******************************/
// include PID library by Brett Beauregard
#include <PID_v1.h>
// define variables
double setPointTemp = 34.5; // the goal temp, in degrees celsius
double Output; // the PIDs output
// specify tuning parameters
double Kp = 0.1, Ki = 5500.0, Kd = 1375.0; // PID variables
PID myPID(&temp, &Output, &setPointTemp, Kp, Ki, Kd, DIRECT);
// PID limits
unsigned long windowSize = 30000; // the time which the PID distributes between ON and OFF
int PIDCalculated; // for checking if PID has been calculated
unsigned long windowTime;
unsigned long onTimeVal; // on time value

/*******************************
   Text output setup
 *******************************/
// for calculation of length of values
String valString = "";
int valLength;

// text to print before temps
String setPointTempText = "SP:";
String actualTempText = "A:";
String humidityText = "Hum:";

// LCD text offsets
const int setPointYOffset = 0;
const int actualTempYOffset = 9;
const int humidityYOffset = 0;
const int PIDYOffset = 9;

// buffer to store float to string
char dtostrfbuffer[5];

void setup(void) {

  // start LCD
  lcd.begin(lcdColumns, lcdRows);
  // print a message to the LCD
  lcd.setCursor(0, 0);
  lcd.print(programName);
  lcd.setCursor(0, 1);
  lcd.print("Booting ...");

  // start serial port
  lcd.setCursor(0, 1);
  lcd.print("Starting serial ...");
  Serial.begin(9600);

  // print information
  if (!plot) Serial.println(programName);
  if (!plot) Serial.println(date);
  if (!plot) Serial.print("by ");
  if (!plot) Serial.print(author);
  if (!plot) Serial.println(email);
  if (!plot) Serial.println();

  // read set temp from eeprom
  if (!plot) Serial.println("Reading last temp from eeprom ...");
  lcd.setCursor(0, 1);
  lcd.print("Reading last temp ...");
  float f = 0.00f;
  EEPROM.get( eeAddr, f );
  if (!plot) Serial.println( f );

  // check if there is a value stored in eeprom
  // if so, then use it as set point,
  // else store the predefined value
  if ( isnan(f) ) {
    if (!plot) Serial.println("No value found in eeprom");
    if (!plot) Serial.print("Storing value: ");
    if (!plot) Serial.println(setPointTemp);
    EEPROM.put(eeAddr, setPointTemp);
  } else {
    if (!plot) Serial.print("Found a value stored in eeprom: ");
    if (!plot) Serial.println(f);
    if (!plot) Serial.println("Using it as set point temperature");
    setPointTemp = f;
    if (!plot) Serial.println();
  }

  // start DHT sensor
  if (!plot) Serial.println("Starting DHT sensor ...");
  lcd.setCursor(0, 1);
  lcd.print("Starting sensor ...");
  dht.begin();  // start up dht sensors

  // define in- and outputs
  // relay pins
  if (!plot) Serial.println("Starting outputs  ...");
  lcd.setCursor(0, 1);
  lcd.print("Starting outputs ...");
  pinMode(heatRelay, OUTPUT);
  pinMode(fanRelay, OUTPUT);
  // rotary encoder pins
  if (!plot) Serial.println("Starting inputs ...");
  lcd.setCursor(0, 1);
  lcd.print("Starting inputs ...");
  pinMode(encoderCLK, INPUT);
  pinMode(encoderDT, INPUT);
  pinMode(encoderSW, INPUT);

  // start PID
  if (!plot) Serial.println("Starting PID ...");
  lcd.setCursor(0, 1);
  lcd.print("Starting PID ...");
  myPID.SetOutputLimits(0, 1000); // 1000 steps in the control loop
  myPID.SetSampleTime(1000); //default is 100 miliseconds
  myPID.SetMode(MANUAL);// clears windup anc verifies PID calculations are correct based on sample time
  delay(5000); // prep the millis() timer below to have 5 seconds on it before PID starts calculating and turns on the heater
  myPID.SetMode(AUTOMATIC); // // start the PID ready for first calculation

  lcd.clear();

  printSetPoint(); // print set point temp to LCD
  printHeatState(); // print heat state to LCD and serial

  if (!plot) Serial.println("Program starts ...");
  if (!plot) Serial.println();
}

void loop(void) {
  /*******************************
     Rotary Encoder
   *******************************/
  encoderCLKState = digitalRead(encoderCLK); // read rotary encoder
  encoderDTState = digitalRead(encoderDT);
  encoderSWState = digitalRead(encoderSW);

  if ( encoderSWState == 0 ) { // button is pressed
    if ((encoderDTStateLast == 0) && (encoderDTState == 1)) { // encoder has been turned
      if (encoderCLKState == 0) { // encoder is turned clockwise
        setPointTemp += 0.1;
        if (!plot) Serial.print("Increasing set point. New value: ");
      } else { // encoder is turned anti clockwise
        setPointTemp -= 0.1;
        if (!plot) Serial.print("Decreasing set point. New value: ");
      }
      if (!plot) Serial.println(setPointTemp);
    }
    encoderDTStateLast = encoderDTState; // store last value of encoder
  }

  if ( encoderSWState != encoderSWStateLast ) { // button has toggled
    if ( encoderSWState == 0 ) { // button was pressed
      if (!plot) Serial.println("Button is down");
    } else { // button was released
      if (!plot) Serial.println("Button is up");
      if (!plot) Serial.print("Storing new value to eeprom: ");
      if (!plot) Serial.print(setPointTemp);
      if (!plot) Serial.println(" ...");
      EEPROM.put(eeAddr, setPointTemp); // write new set point to eeprom
    }
    encoderSWStateLast = encoderSWState; // store last value of encoder switch
    if (!plot) Serial.println();
  }

  printSetPoint(); // print the new set point temp to LCD

  /*******************************
     Temperature
   ********************************/
  if ( encoderSWState ) { // only read values if encoder button is UP
    if ( millis() - readMillis >= waitTime ) { // check if it is time to read sensor
      if (!plot) Serial.println("Requesting humidity and temperature ...");
      hum = dht.readHumidity(); // read humidity
      temp = dht.readTemperature(); // read temperature as celsius

      if (isnan(hum) || isnan(temp)) { // check if any reads failed
        if (!plot) Serial.println(F("Failed to read from DHT sensor!"));
        return;
      }
      else {
        if (plot) {
          Serial.print(setPointTemp); // print set point, temperature in celcius
          Serial.print(",");
          Serial.print(temp); // print process value, temperature in celsius
          Serial.print(",");
          Serial.print(hum); // prints humidity, relative %
          Serial.print(",");
          Serial.print(10 + Output / 100); // prints PID output 10-20
          Serial.print(",");
          Serial.println(heatState * 10); // 0 = cooling off, 10 = heating, 20 = waiting
        }
      }
      printActualValues(); // print measured values to serial and LCD
      readMillis = millis(); // this is the time when sensor was last read
    }
  }

  /*******************************
     PID
   *******************************/
  PIDCalculated = myPID.Compute(); // this only calculates once every second and returns TRUE when it does
  writeToheatRelay(Output); // switch relay on or off
  if (PIDCalculated) {
    if (!plot) Serial.print("SP: ");
    if (!plot) Serial.print(setPointTemp);
    if (!plot) Serial.print("°C, PV: ");
    if (!plot) Serial.print(temp);
    if (!plot) Serial.print("°C, PIDout: ");
    if (!plot) Serial.print(Output / 10);
    if (!plot) Serial.print("%, WT: ");
    if (!plot) Serial.print(windowTime);
    if (!plot) Serial.print(", OTV: ");
    if (!plot) Serial.println(onTimeVal);
    if (!plot) Serial.print(", HRS: ");
    if (!plot) Serial.println((heatRelayState) ? "On" : "OFF");
    if (!plot) Serial.println();
    printPIDOutput(); // print PID output to LCD
  }

  if ( heatState != heatStateLast ) { // there has been a change in heating or cooling
    printHeatState(); // print heat state to LCD and serial
    heatStateLast = heatState; // store last value
  }

  /*******************************
   Fan
  *******************************/
  digitalWrite(fanRelay, fanRelayState);

  if ( fanState != fanStateLast ) { // there has been a change in fan status
    printFanState(); // print fan status to LCD and serial
    fanStateLast = fanState; // store last value
  }
}

// dewPoint function NOAA
// reference: http://wahiduddin.net/calc/density_algorithms.htm
double dewPointFunction(double celsius, double humidity) {
  double A0 = 373.15 / (273.15 + celsius);
  double SUM = -7.90298 * (A0 - 1);
  SUM += 5.02808 * log10(A0);
  SUM += -1.3816e-7 * (pow(10, (11.344 * (1 - 1 / A0))) - 1);
  SUM += 8.1328e-3 * (pow(10, (-3.49149 * (A0 - 1))) - 1);
  SUM += log10(1013.246);
  double VP = pow(10, SUM - 3) * humidity;
  double T = log(VP / 0.61078);   // temp var
  return (241.88 * T) / (17.558 - T);
}

// delta max = 0.6544 wrt dewPoint()
// 5x faster than dewPoint()
// reference: http://en.wikipedia.org/wiki/Dew_point
double dewPointFastFunction(double celsius, double humidity) {
  double a = 17.271;
  double b = 237.7;
  double temp = (a * celsius) / (b + celsius) + log(humidity / 100);
  double Td = (b * temp) / (a - temp);
  return Td;
}

int intToStringToLength(int val) { // returns how many numbers in integer
  valString = String(val); // convert to string
  valLength = valString.length(); // count number of characters in string
  return valLength;
}

void printSetPoint() { // prints set point temp to LCD
  lcd.setCursor(setPointYOffset, 0);
  lcd.print(setPointTempText);
  valLength = setPointTempText.length(); // number of characters before digits
  lcd.setCursor(setPointYOffset + valLength, 0);
  lcd.print(setPointTemp, 1); // prints set point
  dtostrf(setPointTemp, 1, 1, dtostrfbuffer);
  valLength = valLength + strlen(dtostrfbuffer); // number of characters before digits + number of digits
  lcd.setCursor(setPointYOffset + valLength, 0);
  lcd.print((char)223); // prints degree sign
  lcd.setCursor(setPointYOffset + valLength + 1, 0);
  lcd.print(" ");
}

void printActualValues() { // prints measured values to serial and LCD
  dewPoint = dewPointFunction(temp, hum);   // calculate dew points in two different ways
  dewPointFast = dewPointFastFunction(temp, hum);

  hic = dht.computeHeatIndex(temp, hum, false);   // calculate heat index in Celsius (isFahrenheit = false)

  if (!plot) Serial.print("    Dew Point: ");
  if (!plot) Serial.println(dewPoint);
  if (!plot) Serial.print("    Dew Point Fast: ");
  if (!plot) Serial.println(dewPointFast);
  if (!plot) Serial.print("    Heat Index: ");
  if (!plot) Serial.println(hic);

  if (!plot) Serial.print("    Temperature: ");
  if (!plot) Serial.println(temp);

  lcd.setCursor(actualTempYOffset, 0);
  lcd.print(actualTempText);
  valLength = actualTempText.length();
  lcd.setCursor(actualTempYOffset + valLength, 0);
  lcd.print(temp, 1);
  dtostrf(temp, 1, 1, dtostrfbuffer);
  valLength = valLength + strlen(dtostrfbuffer);
  lcd.setCursor(actualTempYOffset + valLength, 0);
  lcd.print((char)223);
  lcd.setCursor(actualTempYOffset + valLength + 1, 0);
  lcd.print(" ");

  if (!plot) Serial.print("    Humidity: ");
  if (!plot) Serial.println(hum);

  lcd.setCursor(humidityYOffset, 1);
  lcd.print(humidityText);
  valLength = humidityText.length(); // number of characters before digits
  lcd.setCursor(humidityYOffset + valLength, 1);
  lcd.print(hum, 0);
  dtostrf(hum, 1, 0, dtostrfbuffer);
  valLength = valLength + strlen(dtostrfbuffer);
  lcd.setCursor(humidityYOffset + valLength, 1);
  lcd.print("%  ");

  if (!plot) Serial.println();
}

void printPIDOutput() { // prints PID output to LCD
  lcd.setCursor(PIDYOffset, 1);
  int output = round(Output / 10);
  lcd.print(output);
  valLength = intToStringToLength(output);
  lcd.setCursor(PIDYOffset + valLength, 1);
  lcd.print("%  ");
}

void printHeatState() { // prints heat state (ie C, H or W) to LCD and serial
  if ( heatState == 0 ) {
    if (!plot) Serial.println("Cooling ...");
    lcd.setCursor(15, 1);
    lcd.print("C");
  } else if ( heatState == 1 ) {
    if (!plot) Serial.println("Heating ...");
    lcd.setCursor(15, 1);
    lcd.print("H");
  } else {
    if (!plot) Serial.println("At goal temp or waiting for button to come up");
    lcd.setCursor(15, 1);
    lcd.print("W");
  }
  if (!plot) Serial.println();
}

void printFanState() { // prints fan state to LCD and serial
	  if ( fanState == 1 ) {
	    if (!plot) Serial.println("Fan ON");
	    lcd.setCursor(14, 1);
	    lcd.print("F");
	  } else {
	    if (!plot) Serial.println("Fan OFF");
	    lcd.setCursor(14, 1);
	    lcd.print(" ");
	  }
	  if (!plot) Serial.println();
}

void writeToheatRelay(double value) {
  windowTime = millis() % windowSize; // convert millis into a millisecond counter that resets at windowSize
  onTimeVal = (unsigned long)(value * (double)windowSize / 1000.0); // convert windowSize into seconds and multiply it by output which ranges from 0 to 1000

  if (onTimeVal > windowTime) {
    heatRelayState = HIGH;
    heatState = 1;

  } else {
    heatRelayState = LOW;
    heatState = 0;
  }

  digitalWrite(heatRelay, heatRelayState); // set the output
  /*
    // short cycle prevention (Blink without delay)
    static unsigned long ShortCycleTimer;
    if ((millis() - ShortCycleTimer) >= (5000)) {
    if ((digitalRead(heatRelay) == LOW) && (heatRelayState = HIGH)) {
    	ShortCycleTimer = millis(); // this sets a blink without delay timer that prevents the following line from triggeringfor 5 seconds after it changes the output to HIGH
    }
    digitalWrite(heatRelay, heatRelayState); // set the output
    }
  */
}
