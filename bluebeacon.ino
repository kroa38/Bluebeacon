/**********************************************************************************************

                                           \|||/
                                           (o o)
                   +------------------oooO-(_)---------------------+
                   |                                               |
                   |                BLUEBEACON SENSOR              |
                   |                                               |
                   +-----------------------------Ooo---------------+
                                          |__|__|
                                           || ||
                                          ooO Ooo
*********************************************************************************************/
/*!
 * @file BlueBeacon.ino
 * 
 * IBEACON Sensor with 10 years life battery .
 * 
 * BME680 temperature, humidity, barometric pressure and gas sensor
 * Battery monitoring
 * TMP117x High-Accuracy, Low-Power, Digital Temperature Sensor
 * TPL5110 Nano-Power System Timer for Power Gating.
 * Blueduino module from April Brother with CC2540
 * ----------------------------------------------------------------
 * example : IBEACON UUID = 2332A4C2-xx01-0e63-04dc-00f726ed014a
 * 2332A4C2 = Sensor UUID prefix
 * xx01 = Sensor number with xx random number for identify frame
 * 0e63 = battery value in hex :  3.683V
 * 04dc = temperature from BME680 : +24.4°C
 * 00f7 = altitude in meter : 247m
 * 26ed = pressure in Hpa : 996.5Hpa
 * 014a = gaz resistor in Kohms : 330Kohms
 * 
 * example : IBEACON MAJOR = 1247
 * 1244 = temperature from TMP117 : +24.7°C
 * 
 * example : IBEACON MINOR = 4889
 * 4889 = Relative Humidity in % : 48.89%
 * ----------------------------------------------------------------
 * Before using you must connect the module to USB port of your PC
 * and configure it with the parameters below by using a serialpassthrough
 * sketch at 9600 baud.

 * AT+NAME=MYNAME  -> set module name
 * AT+ADVI1        -> set 100ms advertising interval
 * AT+POWR1        -> set power at +4dBm
 * AT+ROLE0        -> set role as slave
 * AT+RESET        -> reset module
 * AT+BAUD1        -> set 19200 baud
 * 
 * Once the ILT254s updated  you must use the SPI for
 * the ATMEGA32U4 to programm the software.
 * 
 * Flash the generated hex file by using SPI connexion.
 * Disable BOOTRST FUSE (this accelerate strat-up)
 * 
 * Don't forget to update the UUID_SENSOR number in the defs.h
 * for each module.
 * 
 */

/* Include ------------------------------------------------------------*/
#include "defs.h"

/* Global variables      -------------------------------------------------*/
Ssensor pSsens={0x1001,0x1002,0x1003,0x1004,0x1005,0x1006,0x1007};
char uuid[UUID_LENGTH];
char major[UUID_LENGTH];
char minor[UUID_LENGTH];
bool at_ack = true;

/* Private handle ---------------------------------------------------------*/
Adafruit_BME680 bme; // I2C
AB_BLE ble(&Serial1);
TMP117 tmp(TMP117_ADDR);

/************************************************************************
  * @brief  arduino setup
  * @param  none
  * @retval none
************************************************************************/
void setup()   /*----( SETUP: RUNS ONCE )----*/
{
    USBCON = 0;                     // disable USB for no enumeration
    configure_wdt(WDT_16MS);        // configure watchdog
    pinMode(TEST_PIN,OUTPUT);       // debug pin
    digitalWrite(TEST_PIN,LOW);
    pinMode(DONE_PIN,OUTPUT);       // Done-Pin for TPL5110
    digitalWrite(DONE_PIN,LOW);
    test_pin(1);
    analogReference(INTERNAL);      // internal ref at 2.56V
    Serial1.begin(19200);           // UART at 19200 baud
	  Serial1.setTimeout(15);         // serial time out at 15ms
    bme680_configure(BME_ON);       // init BME680
}
/************************************************************************
  * @brief  arduino main loop
  * @param  none
  * @retval none 
************************************************************************/
void loop() 
{
    configure_wdt(WDT_16MS);                // watchdog enabled at 16mS  
    bme680_read_data();                     // read value from BME680
    bme680_configure(BME_OFF);              // power off BME680
    tmp117_read_data();                     // read value from TMP117
    adc_read_data();                        // read battery voltage
    debug_serial();                         // for debug (USB with Boot must be enable on 32U4)
    /***********************/
    ble_build_uuid();                       // construct UUID string
    ble_build_major();                      // construct UUID string
    ble_build_minor();                      // construct UUID string
    /**********************/
    at_ack = ble_send_AT(major,10);         // write major to CC2540
    at_ack &= ble_send_AT(minor,4);         // write minor to CC2540
    at_ack &= ble_send_AT(uuid,4);          // write uuid to CC2540
    at_ack &= ble_send_AT("AT+BCON1",4);    // CC2540 radio ON
    sleep(15);                              // sleep 32U4 during broadcast
    at_ack &= ble_send_AT("AT+BCON0",10);   // CC2540 radio OFF
    if (!at_ack)  reset_by_wdt();           // reset 
    test_pin(1);          
    digitalWrite(DONE_PIN,HIGH);            // Power OFF if in battery mode 
    configure_wdt(WDT_1S);                  // not appends in battery mode
    for(byte i=0; i<225; i++)               // sleep for 15min if USB powered
    {                                      
    sleep(4);                               // 4*225*1 = 900s = 15min 
    }
}

/************************************************************************
  * @brief  construct minor string 13 char
  * @param  none
  * @retval none
************************************************************************/
void ble_build_minor()
{
  String str;
  str = "AT+MINR";
  str += String(pSsens.bme_hum,DEC);
  str.toCharArray(minor, MINOR_LENGTH);
}
/************************************************************************
  * @brief  construct major string 13 char
  * @param  none
  * @retval none
************************************************************************/
void ble_build_major()
{
  String str;
  str = "AT+MAJR";
  str += String(pSsens.tmp117_value,DEC); 
  str.toCharArray(major, MAJOR_LENGTH);
}
/************************************************************************
  * @brief  construct the uuid with different sensor value (41 char)
  * @param  none
  * @retval none
************************************************************************/
void ble_build_uuid()
{
  String str,rndstr;
  uint16_t randNumber;
    
  randomSeed(analogRead(1));            // true random number !!
  randNumber = (uint16_t)(random(0X1000,0xFFFF));
  rndstr  = uint_to_string(randNumber);     // frame identifier

  str = "AT+PUID=";
  str += UUID_PREFIX;                       //  "2332A4C2"
  str += rndstr;                            //  "1F45"
  str += SENSOR_TYPE;                       //  "01"
  str += SENSOR_NUMBER;                     //  "01"
  str += uint_to_string(pSsens.batt_value); //  "0135"
  str += uint_to_string(pSsens.bme_alt);    //  "0254"
  str += uint_to_string(pSsens.bme_press);  //  "4515"
  str += uint_to_string(pSsens.bme_gas);    //  "0045"
  str.toCharArray(uuid, UUID_LENGTH);
}
/************************************************************************
  * @brief  send the AT command on serial
  * @param  AT cde, number of retry
  * @retval false if fail, true if ok
************************************************************************/
bool ble_send_AT(char *mesg,byte ttw)
{
  bool rep=true;
 
  while(!ble.waitOk())
  {
    Serial1.println(mesg);
    Serial1.flush();
    delay(15);
    if(--ttw == 0) 
    {
      rep = false;
      break;
    }
  }
  return rep;
}
/************************************************************************
  * @brief  retrun a string of 4 letter from an uint16
  * @param  uint16_te ex 4561
  * @retval String ex: "4561"
************************************************************************/
String uint_to_string(uint16_t val)
{
  String teststring = "aaaa";
  byte testlength;
  
  teststring = String(val,HEX);
  testlength = teststring.length();

  switch (testlength)
  {
  case 1:
  teststring = "000" + teststring;
  break;
  case 2:
  teststring = "00" + teststring;
  break;
  case 3:
  teststring = "0" + teststring;
  break;
  default:
  asm("nop");
  break;
  }

  return teststring;
}
/************************************************************************
  * @brief  read data from BME680 and store into global structure
  * @param  none
  * @retval none
************************************************************************/
void bme680_read_data(void)
{
    float val;
    uint32_t valgas;
    for(byte i=0;i<4;i++)
    {
       if(bme.performReading())
       {
        break;
       }
       delay(50);
    }  
    
    /****************************************************************************************/
    val = bme.temperature;                        // return temperature as float ex 23.56°C
    if(val<0) 
    {
      val = abs(val)*10+2000;                     // return 2152 for -15.2°C
    }
    else
    {
      val = val*10 + 1000;                        // return 1237 for +23.7°C
    }
    pSsens.bme_temp = (uint16_t)(val);            //return bme_temp as int
    
    /***************************************************************************************/
    val = bme.humidity;                           // return humidity as float ex 33.65%
    val = val *100;
    pSsens.bme_hum = (uint16_t)(val);             // return humidity as 3365
    /****************************************************************************************/    
    val = bme.pressure;                           // return pressure as float ex 995.2Hpa
    pSsens.bme_press = (uint16_t)(val/10);        // return 9952
    /***************************************************************************************/
    val = bme.readAltitude(SEALEVELPRESSURE_HPA); // return alt as 223m 
    pSsens.bme_alt = (uint16_t)(val);             // return alt as 223
    /**************************************************************************************/
    valgas = bme.gas_resistance;                  // return resistance as ohms ex : 447756 
    pSsens.bme_gas = (uint16_t)(valgas/1000);     // return 447
}


/************************************************************************
  * @brief  configure bme680
  * @param  (mode : BME_ON or BME_OFF)
  * @retval none
************************************************************************/
void bme680_configure(byte mode_init)
{
  
  if(mode_init == BME_ON)
  {

  // Set up oversampling and filter initialization
  bme.begin();                    // check if BME ok 
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);

  if (USBSTA & 0x01)			     // detect if powered by USB
  {
	  bme.setGasHeater(320,150); // enable gas function 320*C for 150 ms
    return;                   // no need to power off
  }
  else
  {	  
	  bme.setGasHeater(0,0);    // disable gas function  or set 320*C for 150 ms 
  }

  }
  else if(mode_init == BME_OFF)
  {
       if (USBSTA & 0x01)             // detect if powered by USB
      {
        return;                       // no need to power off
      }   
      Wire.beginTransmission(0x76);   // The default I2C address for BME680
      Wire.write(0x74);               // BME680_CONF_T_P_MODE_ADDR
      Wire.write(0x00);               // put the BME680 in sleep mode
      Wire.endTransmission();         
  }
  
}
/************************************************************************
  * @brief  tmp117  (get temperature from tmp117 as 2711 = 27,11°C
  * @param  none
  * @retval int (2152 for -15.2°C, 1237 for +23.7°C)
************************************************************************/
void tmp117_read_data(void)
{

  #define TMP117_REG_CONFIGURATION        0x01
  
  double val;
  int16_t reg_value;
  uint8_t data[2] = {0}; 
  int16_t datac = 0;    
  bool data_ready = false;
  


  tmp.init ( NULL );
  tmp.setConvMode (ONESHOT);
  tmp.setConvTime (C15mS5);    // C15mS5,C125mS,C500mS,C4S
  tmp.setAveraging (AVE8);    // NOAVE, AVE8, AVE32, AVE64
  
  val = tmp.getTemperature();
  val = val * 10.0;

  if(val<0) 
  {
    val = abs(val)+2000;  // return 2152 for -15.2°C
  }
  else
  {
    val = val + 1000;     // return 1237 for +23.7°C
  }
  pSsens.tmp117_value = (uint16_t)(val);

}
/************************************************************************
  * @brief  read battery voltage
  * @param  pin as int
  * @retval uint16_t (ex 4752 = 4.752V)
************************************************************************/

void adc_read_data(void)
{
 float val;

    for (byte i=0; i<4; i++)
    {
      val = float(analogRead(BATT_PIN));   
      delay(5);
    }

      val = 1000*(val*2*ADC_REFERENCE) / 1023;
   
    pSsens.batt_value = (uint16_t)(val);
}
