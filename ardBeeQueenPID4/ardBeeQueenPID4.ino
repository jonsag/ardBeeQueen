String programName = "ardBeeQueenPID4";
String date = "201910020";
String author = "Jon Sagebrand";
String email = "jonsagebrand@gmail.com";

const bool plot = 1; // enable plotting function, all other output to serial will be suppressed

/*******************************
   EEPROM setup
 *******************************/
#include <EEPROM.h> // include EEPROM library
const int eeAddr = 0; // address to store the set point temp
double f; // stores the temp set in eeprom

/*******************************
   DHT setup
 *******************************/
#include <DHT.h> // include the DHT library
#define DHTPIN 2  // declare DHT pin number

// uncomment one of the below sensors
//#define DHTTYPE DHT11   // DHT 11 used
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM230)

DHT dht(DHTPIN, DHTTYPE);  // initialize DHT sensor

unsigned long readMillis; // stores millis
const int waitTime = 2000; // millis to wait between reads

double hum = 0; // stores humidity
double temp = 0; // stores temperature
double dewPoint = 0; // calculated dew point
double dewPointFast = 0; // another calculated dew point
double hic = 0;

/*******************************
   Input/Output pins setup
 *******************************/
const int heatRelay = 3;  // heating relay
bool heatRelayState;
bool heatRelayStateLast = 0; // last state of operation

const int fanRelay = 4;  // cooling relay
bool fanRelayState = 0; // fan is always on/off
bool fanRelayStateLast = 0; // last state of operation

const int encoderCLK = 5; // rotary encoder CLK signal input pin
const int encoderDT = 6; // rotary encoder DT signal input pin
const int encoderSW = 7; // rotary encoders switch input pin, goes LOW when pressed

bool encoderCLKState = 0; // for storing encoder values
bool encoderDTState = 0;
bool encoderSWState = 0;
bool encoderDTStateLast = 0;
bool encoderSWStateLast = 0;

unsigned long encoderSWTimeMillis = 0; // the last time encoder button was toggled
const int debounce = 200;   // the debounce time, increase if the output flickers

/*******************************
   LCD setup
 *******************************/
#include <LiquidCrystal.h> // include the LCD library

// define LCD pins
const int LCD_RS = 13; // LCD RS, pin 4
const int LCD_EN = 12; // LCD E, pin 6, Enable
const int LCD_D4 = 11; // LCD D4, pin 11, databit 4
const int LCD_D5 = 10; // LCD D5, pin 12, databit 5
const int LCD_D6 = 9; // LCD D6, pin 13, databit 6
const int LCD_D7 = 8; // LCD D7, pin 14, databit7
/* other pins on LCD are:
   VSS, pin 1, GND
   VDD, pin 2, +5V
   V0, pin 3, 10k potentiometer, ends to +5V and GND, middle pin to V0, contrast
   R/W, pin 5, GND, Read/Write
   A, pin 15, 220ohm resistor, one end to +5V, other to A, back light anode, ~+4.2V
   K, pin 16, GND, back light cathode
   D0, pin 7, databit 0, not used/connected
   D1, pin 8, databit 1, not used/connected
   D2, pin 9, databit 2, not used/connected
   D3, pin 10, databit 3, not used/connected
*/

LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7); // initialize the LCD library

const int lcdColumns = 16; // columns and rows of the LCD
const int lcdRows = 2;

/*******************************
   PID setup
 *******************************/
#include <PID_v1.h> // include PID library by Brett Beauregard

double setPointTemp = 34.5; // the goal temp, in degrees celsius
double Output; // the PIDs output

double Kp = 0.1, Ki = 5500.0, Kd = 1375.0; // specify PID tuning variables
PID myPID(&temp, &Output, &setPointTemp, Kp, Ki, Kd, DIRECT);

const unsigned long windowSize = 30000; // the time which the PID distributes between ON and OFF
bool PIDCalculated; // for checking if PID has been calculated
unsigned long windowTime;
unsigned long onTimeVal; // on time value

/*******************************
   Text output setup
 *******************************/
String valString = ""; // for calculation of length of values
int valLength;

String setPointTempText = "SP:"; // text to print before values on LCD
String actualTempText = "A:";
String humidityText = "Hm:";

const int setPointYOffset = 0; // LCD text offsets
const int actualTempYOffset = 9;
const int humidityYOffset = 0;
const int PIDYOffset = 8;

char dtostrfbuffer[5]; // buffer to store float to string

void setup(void) {
  /*******************************
    start LCD
  *******************************/
  lcd.begin(lcdColumns, lcdRows); // start LCD

  lcd.setCursor(0, 0); // print a message to the LCD
  lcd.print(programName);
  lcd.setCursor(0, 1);
  lcd.print("Booting ...");

  /*******************************
    start serial
  *******************************/
  lcd.setCursor(0, 1);
  lcd.print("Starting serial ...");
  Serial.begin(9600); // start serial port

  if (!plot) {
    Serial.println(programName); // print information
    Serial.println(date);
    Serial.print("by ");
    Serial.print(author);
    Serial.println(email);
    Serial.println();
  }

  /*******************************
    read value from eeprom
  *******************************/
  if (!plot) {
    Serial.println("Reading last temp from eeprom ...");
  }
  lcd.setCursor(0, 1);
  lcd.print("Reading last temp ...");
  f = 0.00f;
  EEPROM.get(eeAddr, f); // read set point from eeprom
  if (!plot) {
    Serial.print("Found value: ");
    Serial.println(f);
    Serial.println();
  }

  if (isnan(f)) { // check if there is a value stored in eeprom
    if (!plot) {
      Serial.println("No value found in eeprom");
      Serial.print("Storing value: ");
      Serial.println(setPointTemp);
      Serial.println();
    }
    EEPROM.put(eeAddr, setPointTemp); // if not, store the predefined val
  } else {
    if (!plot) {
      Serial.print("Found setpoint stored in eeprom: ");
      Serial.println(f);
      Serial.println("Using it as set point temperature");
      Serial.println();
    }
    setPointTemp = f; // else use it as set point
  }

  /*******************************
    start DHT
  *******************************/
  if (!plot) {
    Serial.println("Starting DHT sensor ...");
    Serial.println();
  }
  lcd.setCursor(0, 1);
  lcd.print("Starting sensor ...");
  dht.begin();  // start up dht sensors

  /*******************************
    define in- and outputs
  *******************************/
  if (!plot) {
    Serial.println("Starting outputs  ...");
  }
  lcd.setCursor(0, 1);
  lcd.print("Starting outputs ...");
  pinMode(heatRelay, OUTPUT);
  pinMode(fanRelay, OUTPUT);

  // rotary encoder pins
  if (!plot) {
    Serial.println("Starting inputs ...");
    Serial.println();
  }
  lcd.setCursor(0, 1);
  lcd.print("Starting inputs ...");
  pinMode(encoderCLK, INPUT);
  pinMode(encoderDT, INPUT);
  pinMode(encoderSW, INPUT);

  /*******************************
    start PID
  *******************************/
  if (!plot) {
    Serial.println("Starting PID ...");
    Serial.println();
  }
  lcd.setCursor(0, 1);
  lcd.print("Starting PID ...");
  myPID.SetOutputLimits(0, 1000); // 1000 steps in the control loop
  myPID.SetSampleTime(1000); //default is 100 miliseconds
  myPID.SetMode(MANUAL);// clears windup anc verifies PID calculations are correct based on sample time
  delay(5000); // prep the millis() timer below to have 5 seconds on it before PID starts calculating and turns on the heater
  myPID.SetMode(AUTOMATIC); // // start the PID ready for first calculation

  /*******************************
    start
  *******************************/
  lcd.clear();

  printSetPoint(); // print set point temp to LCD
  printHeatState(); // print heat state to LCD and serial
  printFanState(); // print fan state to LCD and serial

  if (!plot) {
    Serial.println("Program starts ...");
    Serial.println();
  }
}

void loop(void) {
  /*******************************
     Rotary Encoder
   *******************************/
  encoderCLKState = digitalRead(encoderCLK); // read rotary encoder
  encoderDTState = digitalRead(encoderDT);
  encoderSWState = digitalRead(encoderSW);

  if (!encoderSWState) { // button is pressed
    if (!encoderDTStateLast && encoderDTState) { // encoder has been turned
      if (!encoderCLKState) { // encoder is turned clockwise
        setPointTemp += 0.1;
        if (!plot) {
          Serial.print("Increasing set point. New value: ");
        }
      } else { // encoder is turned anti clockwise
        setPointTemp -= 0.1;
        if (!plot) {
          Serial.print("Decreasing set point. New value: ");
        }
      }
      if (!plot) {
        Serial.println(setPointTemp);
      }
    }
    encoderDTStateLast = encoderDTState; // store last value of encoder
  }

  if (encoderSWState != encoderSWStateLast) { // button has toggled
    if (!encoderSWState) { // button was pressed
      if (!plot) {
        Serial.println("Button is down");
      }
    } else { // button was released
      if (!plot) {
        Serial.println("Button is up");
        Serial.print("Storing new value to eeprom: ");
        Serial.print(setPointTemp);
        Serial.println(" ...");
      }
      EEPROM.put(eeAddr, setPointTemp); // write new set point to eeprom
    }
    encoderSWStateLast = encoderSWState; // store last value of encoder switch
    if (!plot) {
      Serial.println();
    }
  }

  printSetPoint(); // print the new set point temp to LCD

  /*******************************
     Temperature
   ********************************/
  if (encoderSWState) { // only read values if encoder button is UP
    if (millis() - readMillis >= waitTime) { // check if it's time to read sensor
      if (!plot) {
        Serial.println("Requesting humidity and temperature ...");
      }
      hum = dht.readHumidity(); // read humidity
      temp = dht.readTemperature(); // read temperature as celsius

      if (isnan(hum) || isnan(temp)) { // check if any reads failed
        if (!plot) {
          Serial.println(F("Failed to read from DHT sensor!"));
        }
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
          Serial.println(heatRelayState * 10); // 0 = cooling off, 10 = heating, 20 = waiting
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
    printPIDOutput(); // print PID output to LCD and serial
  }

  if (heatRelayState != heatRelayStateLast) { // there has been a change in heating or cooling
    printHeatState(); // print heat state to LCD and serial
    heatRelayStateLast = heatRelayState; // store last value
  }

  /*******************************
    Fan
  *******************************/
  digitalWrite(fanRelay, fanRelayState);

  if (fanRelayState != fanRelayStateLast) { // there has been a change in fan status
    printFanState(); // print fan status to LCD and serial
    fanRelayStateLast = fanRelayState; // store last value
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

  if (!plot) {
    Serial.print("    Temperature: ");
    Serial.println(temp);
    Serial.print("    Humidity: ");
    Serial.println(hum);
    Serial.print("    Dew Point: ");
    Serial.println(dewPoint);
    Serial.print("    Dew Point Fast: ");
    Serial.println(dewPointFast);
    Serial.print("    Heat Index: ");
    Serial.println(hic);
    Serial.println();
  }

  lcd.setCursor(actualTempYOffset, 0); // temp
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

  lcd.setCursor(humidityYOffset, 1); // humidity
  lcd.print(humidityText);
  valLength = humidityText.length(); // number of characters before digits
  lcd.setCursor(humidityYOffset + valLength, 1);
  lcd.print(hum, 0);
  dtostrf(hum, 1, 0, dtostrfbuffer);
  valLength = valLength + strlen(dtostrfbuffer);
  lcd.setCursor(humidityYOffset + valLength, 1);
  lcd.print("%  ");
}

void printPIDOutput() { // prints PID output to LCD and serial
  if (!plot) {
    Serial.print("SP: ");
    Serial.print(setPointTemp);
    Serial.print("°C, PV: ");
    Serial.print(temp);
    Serial.print("°C, PIDout: ");
    Serial.print(Output / 10);
    Serial.print("%, WT: ");
    Serial.print(windowTime);
    Serial.print(", OTV: ");
    Serial.println(onTimeVal);
    Serial.print(", HRS: ");
    Serial.println((heatRelayState) ? "On" : "OFF");
    Serial.println();
  }
  
  lcd.setCursor(PIDYOffset, 1);
  int output = round(Output / 10);
  lcd.print(output);
  valLength = intToStringToLength(output);
  lcd.setCursor(PIDYOffset + valLength, 1);
  lcd.print("%  ");
}

void printHeatState() { // prints heat state (ie C, H or W) to LCD and serial
  if (!heatRelayState) {
    if (!plot) {
      Serial.println("Cooling ...");
    }
    lcd.setCursor(15, 1);
    lcd.print("C");
  } else {
    if (!plot) {
      Serial.println("Heating ...");
    }
    lcd.setCursor(15, 1);
    lcd.print("H");
  }

  if (!plot) {
    Serial.println();
  }
}

void printFanState() { // prints fan state to LCD and serial
  if (fanRelayState) {
    if (!plot) {
      Serial.println("Fan ON");
    }
    lcd.setCursor(14, 1);
    lcd.print("F");
  } else {
    if (!plot) {
      Serial.println("Fan OFF");
    }
    lcd.setCursor(14, 1);
    lcd.print("x");
  }
  if (!plot) {
    Serial.println();
  }
}

void writeToheatRelay(double value) {
  windowTime = millis() % windowSize; // convert millis into a millisecond counter that resets at windowSize
  onTimeVal = (unsigned long)(value * (double)windowSize / 1000.0); // convert windowSize into seconds and multiply it by output which ranges from 0 to 1000

  if (onTimeVal > windowTime) {
    heatRelayState = 1;
  } else {
    heatRelayState = 0;
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
