/*
 The LCD circuit:
 *                             GND            display pin 1
 *  supply voltage for logic   +5V            display pin 2

 * 10K potentiometer:
 * ends to +5V and ground
 * LCD VO - contrast           wiper          display pin 3

 * LCD RS pin                  digital pin 13 display pin 4
 * LCD R/W pin                 GND            display pin 5
 * LCD Enable pin              digital pin 12 display pin 6
 * LCD D4                      digital pin 11 display pin 11
 * LCD D5                      digital pin 4 display pin 12
 * LCD D6                      digital pin 9  display pin 13
 * LCD D7                      digital pin 8  display pin 14

 * 220ohm resistor to +5V
 * back light anode:+4.2V      220ohm         display pin 15
 * back light cathode          GND	          display pin 16
 */

// Include the DHT library
#include <DHT.h>

// Define DHT pin
#define DHTPIN 5  //Declare Pin Numbers
//#define DHTTYPE DHT11   // DHT 11 used
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM230
DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor

// The LCD library
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 11, 4, 9, 8);

// Columns and rows of the LCD
int lcdColumns = 16;
int lcdRows = 2;

// Variables
float hum = 0;
float temp = 0;

double dewPoint = 0;
double dewPointFast = 0;
float hic = 0;
//int chk = 0;

String valString = ""; // for calculation of length of values
int valLength;

// The goal temp, in degrees celsius
float setPointTemp = 27.8;

// Wait time between reads, in milliseconds
int waitTime = 2000;

String setPointTempText ="SP:";
String actualTempText = "A:";

// Buffer to store float to string
char dtostrfbuffer[5];

// Relay pins
#define HEATING_RELAY  6  // Relay heating
#define COOLING_RELAY  7  // Relay cooling

///// Rotary encoder
const int encoderCLK = 1; // Rotary encoder CLK signal input pin
const int encoderDT = 2; // Rotary encoder DT signal input pin
const int encoderSW = 3; // Rotary encoders switch input pin, goes LOW when pressed
int encoderCLKState = 0;
int encoderDTState = 0;
int encoderSWState = 0;
int encoderDTStateLast = 0;
int encoderSWStateLast = 0;

long encoderSWTimeMillis = 0; // the last time encoder button was toggled
long debounce = 200;   // the debounce time, increase if the output flickers

void setup(void) {
/*
	// set up the LCD's number of columns and rows:
	Serial.println("Initializing LCD...");
*/

	lcd.begin(lcdColumns, lcdRows);
	// Print a message to the LCD
  lcd.setCursor(0, 0);
	lcd.print("Booting...");

	// start serial port
	lcd.setCursor(0, 1);
	lcd.print("Starting serial");
	Serial.begin(9600);

	Serial.println("ardBeeQueen_broken_mini");

  dht.begin();  // start up dht sensors

	lcd.setCursor(0, 1);
	lcd.print("Starting outputs");
	pinMode(HEATING_RELAY, OUTPUT);
	pinMode(COOLING_RELAY, OUTPUT);

  // Define rotary encoder
  pinMode(encoderCLK, INPUT);
  pinMode(encoderDT, INPUT);
  pinMode(encoderSW, INPUT);

 lcd.clear();

}
void loop(void) {

  // Handle rotary encoder
  encoderCLKState = digitalRead(encoderCLK);
  encoderDTState = digitalRead(encoderDT);
  encoderSWState = digitalRead(encoderSW);

  if ((encoderDTStateLast == 0) && (encoderDTState == 1)) {
    if (encoderCLKState == 0) {
      setPointTemp = setPointTemp - 0.1;
    } else {
      setPointTemp = setPointTemp + 0.1;
    }
  }
  encoderDTStateLast = encoderDTState;

  if (encoderSWState != encoderSWStateLast) {
    if (encoderSWState == 0 && encoderSWStateLast == 1 && millis() - encoderSWTimeMillis > debounce) {
      //
      encoderSWStateLast = encoderSWState;
      encoderSWTimeMillis = millis();
    }
  }

	lcd.setCursor(0, 0);
  lcd.print(setPointTempText);
  valLength = setPointTempText.length();
  lcd.setCursor(valLength, 0);
	lcd.print(setPointTemp, 1);
  dtostrf(setPointTemp,1, 1, dtostrfbuffer);
  valLength = valLength + strlen(dtostrfbuffer);
	lcd.setCursor(valLength, 0);
  lcd.print((char)223);

  Serial.println();
	Serial.println(" Requesting humidity and temperature...");

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  hum = dht.readHumidity();
	// Read temperature as Celsius (the default)
  temp = dht.readTemperature();
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(hum) || isnan(temp)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  //hum = round(hum); // rounding to nearest whole value
  //temp = round(temp * 10) / 10.0; // rounding to one decimal place
  
  /*
	Serial.println("DONE");
	switch (chk) {
	case 0:
		Serial.println("OK");
		break;
	case -1:
		Serial.println("Checksum error");
		break;
	case -2:
		Serial.println("Time out error");
		break;
	default:
		Serial.println("Unknown error");
		break;
	}
  */


	dewPoint = dewPointFunction(temp, hum);
	dewPointFast = dewPointFastFunction(temp, hum);

  // Compute heat index in Celsius (isFahreheit = false)
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
  dtostrf(temp,1, 1, dtostrfbuffer);
  valLength = valLength + strlen(dtostrfbuffer);
  lcd.setCursor(2 + valLength, 0);
  lcd.print((char)223);

	Serial.print("Humidity: ");
	Serial.println(hum);
	lcd.setCursor(0, 1);
  lcd.print("Humidity:");
  lcd.setCursor(9, 1);
	lcd.print(hum, 0);
  valLength = intToStringToLength(hum);
  lcd.setCursor(9 + valLength, 1);
  lcd.print("%  ");
  
	if (temp < setPointTemp) {
		digitalWrite(HEATING_RELAY, 0);
		digitalWrite(COOLING_RELAY, 1);

		Serial.println("Heating...");
		lcd.setCursor(15, 1);
		lcd.print("H");

	} else if (temp > setPointTemp) {
		digitalWrite(HEATING_RELAY, 1);
		digitalWrite(COOLING_RELAY, 0);

		Serial.println("Cooling...");
		lcd.setCursor(15, 1);
		lcd.print("C");
	} else {
		digitalWrite(HEATING_RELAY, 0);
		digitalWrite(COOLING_RELAY, 0);

		Serial.println("At goal temp!");
    lcd.setCursor(15, 1);
    lcd.print(" ");
	}
	
  delay(waitTime);  // Wait a few seconds between measurements
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

int intToStringToLength(int val) {
  valString = String(val);
  valLength = valString.length();
  return valLength;
}
