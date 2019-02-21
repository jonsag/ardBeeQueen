# ardBeeQueen
A simple project to control a bee queen breeding chamber

Parts needed
===============
1 x Arduino Pro Mini, kjell.com art#87964  
1 x Arduino USB to Serial Converter, kjell.com art#87872  
2 x Arduino Single Relay Module, kjell.com art#87878  
1 x LCD Display, kjell.com art#90215  
1 x Dallas 1-wire Temp and Humidity Sensor DHT11, kjell.com art#87877  
1 x 10kohm Potentiometer, kjell.com art#90633  
1 x 220ohm resistor  

Casing to hold everything

Wires and soldering material  

Set up Arduino IDE
===============

Go to https://www.arduino.cc/en/main/software and download Arduino IDE for your architecture and system  
>$ tar xvJf arduino-1.8.8-linux64.tar.xz -C ~/bin/  

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









