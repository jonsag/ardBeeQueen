/*
 The LCD circuit:
 *                             GND            display pin 1
 *  supply voltage for logic   +5V            display pin 2

 * 10K resistor:
 * ends to +5V and ground
 * LCD VO - contrast           wiper          display pin 3

 * LCD RS pin                  digital pin 13 display pin 4
 * LCD R/W pin                 GND            display pin 5
 * LCD Enable pin              digital pin 12 display pin 6
 * LCD D4                      digital pin 11 display pin 11
 * LCD D5                      digital pin 10 display pin 12
 * LCD D6                      digital pin 9  display pin 13
 * LCD D7                      digital pin 8  display pin 14

 * 10K resistor:
 * ends to +5V and ground
 * back light anode:+4.2V      wiper          display pin 15
 * back light cathode          wiper          display pin 16
 */

// First we include the Dallas/1-Wire libraries
#include <OneWire.h>
#include <DallasTemperature.h>

// Dallas data wire pin
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// The LCD library
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

// Columns and rows of the LCD
int lcdColumns = 20;
int lcdRows = 4;

// Variables
float temperature = 0;

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
	lcd.print("Starting serial");
	Serial.begin(9600);

	Serial.println("ardBeeQueen");

	// Start up the 1.wire library
	lcd.setCursor(0, 2);
	lcd.print("Starting 1-wire...");
	sensors.begin();

	lcd.setCursor(0, 2);
	lcd.print("Starting outputs...");
	pinMode(HEATING_RELAY, OUTPUT);
	pinMode(COOLING_RELAY, OUTPUT);

	lcd.clear();

}
void loop(void) {
	// call sensors.requestTemperatures() to issue a global temperature
	// request to all devices on the bus

	Serial.print(" Requesting temperatures...");
	sensors.requestTemperatures(); // Send the command to get temperature readings
	Serial.println("DONE");

	temperature = sensors.getTempCByIndex(0);

	Serial.print("Temperature is: ");
	Serial.println(temperature);

	if (temperature < goalTemp) {
		digitalWrite(HEATING_RELAY, 0);
		digitalWrite(COOLING_RELAY, 1);

		Serial.println("Heating...");

	} else if (temperature > goalTemp) {
		digitalWrite(HEATING_RELAY, 1);
		digitalWrite(COOLING_RELAY, 0);

		Serial.println("Cooling...");
	} else {
		digitalWrite(HEATING_RELAY, 0);
		digitalWrite(COOLING_RELAY, 0);

		Serial.println("At goal temp!");
	}
	delay(1000);
}
