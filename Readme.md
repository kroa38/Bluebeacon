
# IBEACON Sensor with 10 years battery life .
------------------------------------------------
BME680 temperature, humidity, barometric pressure and gas sensor.
TMP117x High-Accuracy, Low-Power, Digital Temperature Sensor
TPL5110 Nano-Power System Timer for Power Gating.
Blueduino module from April Brother with CC2540
Use one 14500 Lithium Ion battery (3.6v)
----------------------------------------------------------------
## Apple Ibeacon is : UUID + MAJOR + MINOR

### Example : UUID = 2332A4C2-xx01-0e63-04dc-00f726ed014a

2332A4C2 = Sensor UUID prefix

xx01 = Sensor number with xx random number for identify frame

0e63 = battery value in hex :  3.683V

04dc = temperature from BME680 : +24.4°C

00f7 = altitude in meter : 247m

26ed = pressure in Hpa : 996.5Hpa

014a = gaz resistor in Kohms : 330Kohms

### Example : MAJOR = 1247

1244 = temperature from TMP117 : +24.7°C
 
### Example : MINOR = 4889

4889 = Relative Humidity in % : 48.89%

---------------------------------------------------------------------
# Setup !

Before using you must connect the module to USB port of your PC and configure it with
the parameters below by using a the Serialpassthrough arduino sketch at 9600 baud.

AT+NAME=MYNAME  -> set module name

AT+ADVI1        -> set 100ms advertising interval

AT+POWR1        -> set power at +4dBm

AT+ROLE0        -> set role as slave

AT+RESET        -> reset module

AT+BAUD1        -> set 19200 baud

  
Once the ILT254s updated  you must use the SPI for the ATMEGA32U4 to programm the software.

Flash the generated hex file by using SPI connexion.

Disable BOOTRST FUSE (this accelerate start-up)
  
Don't forget to update the UUID_SENSOR number in the "defs.h" for each module.