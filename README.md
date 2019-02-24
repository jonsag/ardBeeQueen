# ardBeeQueen
A simple project to control a bee queen breeding chamber

Parts needed
===============
1 x Arduino Pro Mini, kjell.com art#87964  
1 x Arduino USB to Serial Converter, kjell.com art#87872  
2 x Arduino Single Relay Module, kjell.com art#87878  
1 x LCD Display, kjell.com art#90215  
1 x Dallas 1-wire Temp and Humidity Sensor DHT11, kjell.com art#87877 or  
1 x Dallas 1-wire Temp and Humidity Sensor DHT22, kjell.com art#90647
1 x 10kohm Potentiometer, kjell.com art#90633  
1 x 220ohm resistor  

Casing to hold everything

Wires and soldering material  

Note
===============

All commands in the following instructions are run on Linux  
All links are valid and versions are the latest as of february 2019  


Set up Arduino IDE
===============

Go to https://www.arduino.cc/en/main/software and download Arduino IDE for your architecture and system  
>$ wget https://downloads.arduino.cc/arduino-1.8.8-linux64.tar.xz  
>$ tar xvJf arduino-1.8.8-linux64.tar.xz -C ~/bin/  
>$ ln -s arduino-1.8.8/arduino ~/bin/arduino  

Get libraries 
---------------

Adafruit Unified Sensor library  
---------------
Download library from https://www.arduinolibraries.info/libraries/adafruit-unified-sensor  
>$ wget http://downloads.arduino.cc/libraries/github.com/adafruit/Adafruit_Unified_Sensor-1.0.2.zip  
>$ unzip Adafruit_Unified_Sensor-1.0.2.zip -d ~/bin/arduino-1.8.8/libraries/  
>$ mv ~/bin/arduino-1.8.8/libraries/Adafruit_Unified_Sensor-1.0.2 ~/bin/arduino-1.8.8/libraries/Adafruit_Unified_Sensor  

DHT sensor library  
---------------
Download library from https://www.arduinolibraries.info/libraries/dht-sensor-library  
>$ wget http://downloads.arduino.cc/libraries/github.com/adafruit/DHT_sensor_library-1.3.4.zip  
>$ unzip DHT_sensor_library-1.3.4.zip -d ~/bin/arduino-1.8.8/libraries/  
>$ mv ~/bin/arduino-1.8.8/libraries/DHT_sensor_library-1.3.4 ~/bin/arduino-1.8.8/libraries/DHT_sensor_library  


Programming the Arduino Mini Pro
===============

If using the Arduino USB 2 Serial  
--------------------
Connect the programmer to your Pro Mini  
GND/BLK -> GND  
+5V -> VCC  
TX -> RX  
RX -> TX  
EXT RESET -> GRN

Open your sketch in Arduino IDE  
Select 'Tools'->'Port'  
* /dev/ttyACM0 or  
* /dev/ttyUSB0  

Select 'Tools'->'Board'  
For the 3.3V versions of the Arduino Pro Mini, select:  
* Arduino Pro or  
* Pro Mini (3.3V, 8 MHz) w/ ATmega328P or  
* Arduino Pro or  
* Pro Mini (3.3V, 8 MHz) w/ ATmega168  
(depending on the microcontroller on your board)  
For the 5V versions of the Arduino Pro Mini, select  
* Arduino Duemilanove or  
* Nano w/ ATmega328P or  
* Arduino Diecimila, Duemilanove (this is what I used) or  
* Nano w/ ATmega168

If using the PC-PL2303 Cable  
---------------
Black cable -> GND  
Green cable -> TXD  
White cable -> RXD  
Red cable -> VCC (5V)  


Fritzing  
===============
Download fritzing at http://fritzing.org/download/  

>$ wget http://fritzing.org/download/0.9.3b/linux-64bit/fritzing-0.9.3b.linux.AMD64.tar.bz2  
>$ tar jxvf ~/Downloads/fritzing-0.9.3b.linux.AMD64.tar.bz2 -C ~/bin/  
>$ ln -s ln -s fritzing-0.9.3b.linux.AMD64/Fritzing ~/bin/fritzing  










