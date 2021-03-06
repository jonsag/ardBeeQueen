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
1 x Rotary encoder, kjell.com art#87918  
1 x 10kohm Potentiometer, kjell.com art#90633  
1 x 220ohm resistor  
1 x 10K ohm resistor  
2 x 0,47uF capacitor  

Casing to hold everything

Wires and soldering material  

If you want to build a PCB using the Gerber file you will also need some headers and connnectors.  

Building electrical  
===============
Use schemes ardBeeQueenFritz* or Schematic_ardBeeQueen-v2_ardBeeQueen-v2_* to build.  
If you do not use the encoder you must still pull up D7 with a 10K resistor to VCC.  


Eagle PCB  
===============
If using the 50x52 PCB in Documents/Eagle directory, you can mount it in box electrokit.com, art#12109003  
Outside measures: 69 x 102 x 37 mm  
Inside measures: 63 x 96 x 33 mm  

If using the 105x84 PCB in Documents/Eagle directory, you can mount it in box electrokit.com, art#12109005  
Outside measures: 103 x 148 x 56 mm  
Insode measures: 96 x 142 x 52 mm  


Note
===============
All commands in the following instructions are run on Linux  
All links are valid and versions are the latest as of february 2019  


Set up Arduino IDE
===============
Go to https://www.arduino.cc/en/main/software and download Arduino IDE for your architecture and system  
>$ wget https://downloads.arduino.cc/arduino-1.8.10-linux64.tar.xz  
>$ tar xvJf arduino-1.8.10-linux64.tar.xz -C ~/bin/  
>$ ln -s arduino-1.8.10/arduino ~/bin/arduino  

Get libraries 
---------------

Adafruit Unified Sensor library  
---------------
Download library from https://www.arduinolibraries.info/libraries/adafruit-unified-sensor  
>$ wget http://downloads.arduino.cc/libraries/github.com/adafruit/Adafruit_Unified_Sensor-1.0.3.zip  
>$ unzip Adafruit_Unified_Sensor-1.0.3.zip -d ~/bin/arduino-1.8.10/libraries/  
>$ mv ~/bin/arduino-1.8.10/libraries/Adafruit_Unified_Sensor-1.0.3 ~/bin/arduino-1.8.10/libraries/Adafruit_Unified_Sensor  

DHT sensor library  
---------------
Download library from https://www.arduinolibraries.info/libraries/dht-sensor-library  
>$ wget http://downloads.arduino.cc/libraries/github.com/adafruit/DHT_sensor_library-1.3.7.zip  
>$ unzip DHT_sensor_library-1.3.7.zip -d ~/bin/arduino-1.8.10/libraries/  
>$ mv ~/bin/arduino-1.8.10/libraries/DHT_sensor_library-1.3.7 ~/bin/arduino-1.8.10/libraries/DHT_sensor_library  


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

You will have to press the reset button on the arduino to reset it right before the upload from Arduino IDE.  

Fritzing  
===============
Download fritzing at http://fritzing.org/download/  

>$ wget http://fritzing.org/download/0.9.3b/linux-64bit/fritzing-0.9.3b.linux.AMD64.tar.bz2  
>$ tar jxvf ~/Downloads/fritzing-0.9.3b.linux.AMD64.tar.bz2 -C ~/bin/  
>$ ln -s fritzing-0.9.3b.linux.AMD64/Fritzing ~/bin/fritzing  


EasyEDA  
==============
Use easyEDA online at https://easyeda.com/editor or download client from https://easyeda.com/page/download  

>$ wget https://image.easyeda.com/files/easyeda-client-linux-x64.zip  
>$ unzip easyeda-client-linux-x64.zip  
>$ cd easyeda-client-linux-x64  
>$ sudo bash install.sh  

Download local router from https://docs.easyeda.com/en/PCB/Route/index.html#Local-Auto-Router  
>$ wget https://image.easyeda.com/files/EasyEDA-Router-v0.8.1.zip  
>$ unzip EasyEDA-Router-v0.8.1.zip  
>$ cd EasyEDA-Router-v0.8.1  

Start router with  
>$ bash lin64.sh  


Autodesk EAGLE
===================
Download EAGLE at https://www.autodesk.com/products/eagle/free-download  

>$ tar xvzf Autodesk_EAGLE_9.3.1_English_Linux_64bit.tar.gz -C ~/bin  
>$ ln -s eagle-9.3.1/eagle ~/bin/eagle  

Start and sign in with your Autodesk account  










