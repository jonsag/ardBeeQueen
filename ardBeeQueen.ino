// First we include the libraries
#include <OneWire.h>
#include <DallasTemperature.h>

// Dallas data wire pin
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Variables
float temperature = 0;

// The goal temp
float goalTemp= 27.8;

// Relay pins
#define HEATING_RELAY  4  // Relay heating
#define COOLING_RELAY  6  // Relay cooling

void setup(void) {
	// start serial port
	Serial.begin(9600);

	Serial.println("ardBeeQueen");

	// Start up the library
	sensors.begin();

	pinMode(HEATING_RELAY, OUTPUT);
	pinMode(COOLING_RELAY, OUTPUT);

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

	if(temperature < goalTemp){
		digitalWrite(HEATING_RELAY,0);
		digitalWrite(COOLING_RELAY,1);

		Serial.println("Heating...");

	} else if(temperature > goalTemp){
		digitalWrite(HEATING_RELAY, 1);
		digitalWrite(COOLING_RELAY, 0);

		Serial.println("Cooling...");
	} else{
		digitalWrite(HEATING_RELAY, 0);
		digitalWrite(COOLING_RELAY, 0);

		Serial.println("At goal temp!");
	}
	delay(1000);
}
