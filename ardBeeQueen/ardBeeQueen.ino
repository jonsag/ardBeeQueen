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
 * LCD D5                      digital pin 10 display pin 12
 * LCD D6                      digital pin 9  display pin 13
 * LCD D7                      digital pin 8  display pin 14

 * 220ohm resistor to +5V
 * back light anode:+4.2V      220ohm         display pin 15
 * back light cathode          GND	          display pin 16
 */

// Include the DHT library
#include <dht11.h>

// Define DHT pin
dht11 DHT11;  //Declare objects
#define DHT11PIN 2  //Declare Pin Numbers

// The LCD library
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

// Columns and rows of the LCD
int lcdColumns = 16;
int lcdRows = 2;

// Variables
float humidity = 0;
float temperature = 0;
double dewPoint = 0;
double dewPointFast = 0;
int chk = 0;

// The goal temp
float goalTemp = 27.8;

// Relay pins
#define HEATING_RELAY  3  // Relay heating
#define COOLING_RELAY  4  // Relay cooling

void setup(void) {
	// set up the LCD's number of columns and rows:
	Serial.println("Initializing LCD...");

	lcd.begin(lcdColumns, lcdRows);
	// Print a message to the LCD.
	lcd.print("Booting...");

	// start serial port
	lcd.setCursor(0, 1);
	lcd.print("Starting serial ");
	Serial.begin(9600);

	Serial.println("ardBeeQueen");

	//// Start up the 1.wire library
	//lcd.setCursor(0, 2);
	//lcd.print("Starting 1-wire...");
	//sensors.begin();

	lcd.setCursor(0, 1);
	lcd.print("Starting outputs");
	pinMode(HEATING_RELAY, OUTPUT);
	pinMode(COOLING_RELAY, OUTPUT);

}
void loop(void) {

	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print(goalTemp, 1);

	Serial.print(" Requesting humidity and temperature...");

	chk = DHT11.read(DHT11PIN);

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

	humidity = DHT11.humidity, 2;
	temperature = DHT11.temperature, 2;

	dewPoint = dewPointFunction(DHT11.temperature, DHT11.humidity);
	dewPointFast = dewPointFastFunction(DHT11.temperature, DHT11.humidity);

	Serial.print("Temperature: ");
	Serial.println(temperature);
	lcd.setCursor(5, 0);
	lcd.print(temperature, 1);

	Serial.print("Humidity: ");
	Serial.println(humidity);
	lcd.setCursor(0, 1);
	lcd.print(humidity);

	Serial.print("Dew point: ");
	Serial.println(dewPoint);

	Serial.print("Dew point fast: ");
	Serial.println(dewPointFast);

	if (temperature < goalTemp) {
		digitalWrite(HEATING_RELAY, 0);
		digitalWrite(COOLING_RELAY, 1);

		Serial.println("Heating...");
		lcd.setCursor(15, 0);
		lcd.print("H");

	} else if (temperature > goalTemp) {
		digitalWrite(HEATING_RELAY, 1);
		digitalWrite(COOLING_RELAY, 0);

		Serial.println("Cooling...");
		lcd.setCursor(15, 1);
		lcd.print("C");
	} else {
		digitalWrite(HEATING_RELAY, 0);
		digitalWrite(COOLING_RELAY, 0);

		Serial.println("At goal temp!");
	}
	delay(1000);
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

