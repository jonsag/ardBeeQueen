// ardBeeQueenPID
/*
  The LCD circuit:
                               GND            display pin 1
    supply voltage for logic   +5V            display pin 2

   10K potentiometer:
   ends to +5V and ground
   LCD VO - contrast           wiper          display pin 3

   LCD RS pin                  digital pin 13 display pin 4
   LCD R/W pin                 GND            display pin 5
   LCD Enable pin              digital pin 12 display pin 6
   LCD D4                      digital pin 11 display pin 11
   LCD D5                      digital pin 10 display pin 12
   LCD D6                      digital pin 9  display pin 13
   LCD D7                      digital pin 8  display pin 14

   220ohm resistor to +5V
   back light anode:+4.2V      220ohm         display pin 15
   back light cathode          GND	          display pin 16
*/

// include EEPROM library
#include <EEPROM.h>
int eeAddr = 0; // address to store the set point temp

// include the DHT library
#include <DHT.h>
// define DHT pin
#define DHTPIN 2  // declare Pin Number
// uncomment one of the below sensors
//#define DHTTYPE DHT11   // DHT 11 used
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM230)
// set up DHT sensor
DHT dht(DHTPIN, DHTTYPE);  // initialize DHT sensor

// relay pins
const int heatingRelay = 3;  // relay heating
const int coolingRelay = 4;  // relay cooling
int heatState = 0; // current state of operation
int heatStateLast = 0; // last state of operation

// rotary encoder
// encoder pins
const int encoderCLK = 5; // rotary encoder CLK signal input pin
const int encoderDT = 6; // rotary encoder DT signal input pin
const int encoderSW = 7; // rotary encoders switch input pin, goes LOW when pressed
// for storing encoder values
int encoderCLKState = 0;
int encoderDTState = 0;
int encoderSWState = 0;
int encoderDTStateLast = 0;
int encoderSWStateLast = 0;
// times
unsigned long encoderSWTimeMillis = 0; // the last time encoder button was toggled
int debounce = 200;   // the debounce time, increase if the output flickers

// LCD
// include the LCD library
#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);
// columns and rows of the LCD
int lcdColumns = 16;
int lcdRows = 2;

// Variables
float hum = 0; // read humidity
float temp = 0; // read temperature

// PID
// include PID library by Brett Beauregard
#include <PID_v1.h>
// define Variables we'll be connecting to
float setPointTemp = 34.5; // the goal temp, in degrees celsius
float Output; // the PIDs output
// specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1; // PID variables
PID myPID(&temp, &Output, &setPointTemp, Kp, Ki, Kd, DIRECT);
// PID limits
int windowSize = 5000; // the time of the PID regulatory size(?)
unsigned long windowStartTime; // stores millis

double dewPoint = 0; // calculated dew point
double dewPointFast = 0; // another calculated dew point
float hic = 0;

// for calculation of length of values
String valString = "";
int valLength;

// wait time between temp reads, in milliseconds
unsigned long readMillis; // stores millis
int waitTime = 2000; // millis to wait between reads

// text to print before temps
String setPointTempText = "SP:";
String actualTempText = "A:";

// buffer to store float to string
char dtostrfbuffer[5];

void setup(void) {

  // start LCD
  lcd.begin(lcdColumns, lcdRows);
  // print a message to the LCD
  lcd.setCursor(0, 0);
  lcd.print("ardBeeQueen");
  lcd.setCursor(0, 1);
  lcd.print("Booting ...");

  // start serial port
  lcd.setCursor(0, 1);
  lcd.print("Starting serial ...");
  Serial.begin(9600);

  Serial.println("ardBeeQueenPID");
  Serial.println("201910011");
  Serial.println("by Jon Sagebrand");
  Serial.println("jonsagebrand@gmail.com");
  Serial.println();

  // read set temp from eeprom
  Serial.println("Reading last temp from eeprom ...");
  lcd.setCursor(0, 1);
  lcd.print("Reading last temp ...");
  float f = 0.00f;
  EEPROM.get( eeAddr, f );
  Serial.println( f );

  // check if there is a value stored in eeprom
  // if so, then use it as set point,
  // else store the predefined value
  if ( isnan(f) ) {
    Serial.println("No value found in eeprom");
    Serial.print("Storing value: ");
    Serial.println(setPointTemp);
    EEPROM.put(eeAddr, setPointTemp);
  } else {
    Serial.print("Found a value stored in eeprom: ");
    Serial.println(f);
    Serial.println("Using it as set point temperature");
    setPointTemp = f;
    Serial.println();
  }

  // start DHT sensor
  Serial.println("Starting DHT sensor ...");
  lcd.setCursor(0, 1);
  lcd.print("Starting sensor ...");
  dht.begin();  // start up dht sensors

  // define in- and outputs
  // relay pins
  Serial.println("Starting outputs  ...");
  lcd.setCursor(0, 1);
  lcd.print("Starting outputs ...");
  pinMode(heatingRelay, OUTPUT);
  pinMode(coolingRelay, OUTPUT);
  // rotary encoder pins
  Serial.println("Starting inputs ...");
  lcd.setCursor(0, 1);
  lcd.print("Starting inputs ...");
  pinMode(encoderCLK, INPUT);
  pinMode(encoderDT, INPUT);
  pinMode(encoderSW, INPUT);

  // start PID
  Serial.println("Starting PID ...");
  lcd.setCursor(0, 1);
  lcd.print("Starting PID ...");
  initiatePID();
  windowStartTime = millis();

  lcd.clear();

  printSetPoint();

  Serial.println("Program starts ...");
  Serial.println();

}
void loop(void) {

  // read rotary encoder
  encoderCLKState = digitalRead(encoderCLK);
  encoderDTState = digitalRead(encoderDT);
  encoderSWState = digitalRead(encoderSW);

  // check encoder if button is pressed
  if ( encoderSWState == 0 ) { // Button is pressed
    if ((encoderDTStateLast == 0) && (encoderDTState == 1)) {
      if (encoderCLKState == 0) {
        setPointTemp += 0.1;
        Serial.print("Counting up set point. New value: ");
      } else {
        setPointTemp -= 0.1;
        Serial.print("Counting down set point. New value: ");
      }
      Serial.println(setPointTemp);
      printSetPoint(); // Prints the new set point to LCD
    }
    encoderDTStateLast = encoderDTState;
  }

  printSetPoint();

  // check if button is released
  if ( encoderSWState != encoderSWStateLast ) {
    if ( encoderSWState == 0 ) {
      Serial.println("Button is down");
    } else {
      Serial.println("Button is up");
      Serial.print("Storing value new value to eeprom: ");
      Serial.println(setPointTemp);
      EEPROM.put(eeAddr, setPointTemp);
    }
    encoderSWStateLast = encoderSWState;
    Serial.println();
    initiatePID(); // initiate PID with new value
  }

  // read temperature
  if ( encoderSWState ) { // Only read values if button is UP
    if ( millis() - readMillis >= waitTime ) {
      Serial.println("Requesting humidity and temperature ...");
      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      hum = dht.readHumidity();
      // Read temperature as Celsius (the default)
      temp = dht.readTemperature();

      // check if any reads failed and exit early (to try again).
      if (isnan(hum) || isnan(temp)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
      }
      printActualValues(); // print measured values to serial and LCD
      readMillis = millis();
    }
  }

  // compute PID
  myPID.Compute();

  // turn the output pin on/off based on pid output
  if (millis() - windowStartTime > WindowSize) { // time to shift the Relay Window
	  windowStartTime += WindowSize;
  }
  if (Output < millis() - windowStartTime) {
	  digitalWrite(RELAY_PIN, HIGH);
  }
  else {
	  digitalWrite(RELAY_PIN, LOW);
  }

/*
  if ( temp < setPointTemp && encoderSWState ) { // if temp is below set point and button is up
    digitalWrite(heatingRelay, 1);
    digitalWrite(coolingRelay, 0);
    heatState = 1; // heating
  } else if ( temp > setPointTemp && encoderSWState ) { // if temp is above set point and button is up
    digitalWrite(heatingRelay, 0);
    digitalWrite(coolingRelay, 1);
    heatState = 0; // cooling
  } else {
    digitalWrite(heatingRelay, 0);
    digitalWrite(coolingRelay, 0);
    heatState = 2; // at correct temp
  }
   */

  if ( heatState != heatStateLast ) { // if there has been a change in heating or cooling
    printHeatState();
    heatStateLast = heatState;
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

int intToStringToLength(int val) { // return how many numbers in integer
  valString = String(val);
  valLength = valString.length();
  return valLength;
}

void printSetPoint() { // prints set point temp to LCD
  lcd.setCursor(0, 0);
  lcd.print(setPointTempText);
  valLength = setPointTempText.length(); // number of characters before digits
  lcd.setCursor(valLength, 0);
  lcd.print(setPointTemp, 1); // prints set point
  dtostrf(setPointTemp, 1, 1, dtostrfbuffer);
  valLength = valLength + strlen(dtostrfbuffer); // number if charaters before digits + number of digits
  lcd.setCursor(valLength, 0);
  lcd.print((char)223); // prints degree sign
  lcd.setCursor(1 + valLength, 0);
  lcd.print(" ");
}

void printActualValues() {
  // calculate dew points
  dewPoint = dewPointFunction(temp, hum);
  dewPointFast = dewPointFastFunction(temp, hum);

  // compute heat index in Celsius (isFahrenheit = false)
  hic = dht.computeHeatIndex(temp, hum, false);

  Serial.print("Dew Point: ");
  Serial.println(dewPoint);
  Serial.print("Dew Point Fast: ");
  Serial.println(dewPointFast);
  Serial.print("Heat Index: ");
  Serial.println(hic);

  Serial.print("Temperature: ");
  Serial.println(temp);

  lcd.setCursor(2 + valLength, 0);
  lcd.print(actualTempText);
  valLength = valLength + actualTempText.length();
  lcd.setCursor(2 + valLength, 0);
  lcd.print(temp, 1);
  dtostrf(temp, 1, 1, dtostrfbuffer);
  valLength = valLength + strlen(dtostrfbuffer);
  lcd.setCursor(2 + valLength, 0);
  lcd.print((char)223);
  lcd.setCursor(3 + valLength, 0);
  lcd.print(" ");

  Serial.print("Humidity: ");
  Serial.println(hum);
  lcd.setCursor(0, 1);
  lcd.print("Humidity:");
  lcd.setCursor(9, 1);
  lcd.print(hum, 0);
  valLength = intToStringToLength(hum);
  lcd.setCursor(9 + valLength, 1);
  lcd.print("%  ");

  Serial.println();
}

void printHeatState() {
  if ( heatState == 0 ) {
    Serial.println("Cooling...");
    lcd.setCursor(15, 1);
    lcd.print("C");
  } else if ( heatState == 1 ) {
    Serial.println("Heating...");
    lcd.setCursor(15, 1);
    lcd.print("H");
  } else {
    Serial.println("At goal temp or waiting for button to come up");
    lcd.setCursor(15, 1);
    lcd.print("W");
  }
  Serial.println();
}

void initiatePID() {
	Serial.println("Initializing PID ...");
	Serial.println();

	// tell the PID to range between 0 and the full window size
	myPID.SetOutputLimits(0, windowSize);

	// turn the PID on
	myPID.SetMode(AUTOMATIC);
}
