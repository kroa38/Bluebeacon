/*
 * utils.ino
 *
 * Created: 26/03/2019 10:37:25
 *  Author: KROA38
 */ 
/* Include ---------------------------------------------------------------*/
#include "defs.h"
/* Global variables      -------------------------------------------------*/
int nbr_remaining; 
/************************************************************************
  * @brief  interrupt vector for watchdog
  * @param  none
  * @retval none
************************************************************************/
ISR(WDT_vect)
{
   // not hanging, just waiting
   // reset the watchdog
   wdt_reset();
}
/************************************************************************
  * @brief  configure watchdog timer
  * @param  none
  * @retval none
  * https://folk.uio.no/jeanra/Microelectronics/ArduinoWatchdog.html
************************************************************************/
void configure_wdt(byte wdt_value)
{
 
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags

                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | wdt_value; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds
  sei();                           // re-enable interrupts
  
}
/************************************************************************
  * @brief  put atmega32U4 in sleep mode
  * @param  none
  * @retval none
************************************************************************/
void sleep(int ncycles)
{  
  nbr_remaining = ncycles; // defines how many cycles should sleep
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  power_adc_disable();
 
  while (nbr_remaining > 0)
  { 
    sleep_mode();
    sleep_disable();
    nbr_remaining = nbr_remaining - 1;
  }
 power_all_enable();
 
}
/************************************************************************
  * @brief  reset using watchdog
  * @param  none
  * @retval none
************************************************************************/
void reset_by_wdt(void)
{
  test_pin(1);
  Serial1.end();
  delay(1000);
  cli();                              // disable interrupt
  WDTCSR |= (1U<<WDCE) | (1U<<WDE) ;  // Enable a watchdog change
  WDTCSR = (1U<<WDE) ;                // Watchdog ON and Time Out = 16ms
  test_pin(1);
  while(1U);
}
/************************************************************************
  * @brief  toogle pin test
  * @param  delay in ms
  * @retval none
************************************************************************/
void test_pin(byte dly)
{
    digitalWrite(TEST_PIN,HIGH);
    delay(dly);
    digitalWrite(TEST_PIN,LOW);
    delay(dly);
}

/************************************************************************
  * @brief  debug_serial (debug at 19200baud on the USB port
  * @param  type, and string
  * @retval None
************************************************************************/
void debug_serial(void)
{
#ifdef DEBUG_TO_USB_SERIAL
String txtMsg;
#define ADDEN 0x80       

  if (UDADDR & ADDEN) // detect if there is a USB connexion
  {
      Serial.begin(115200);       // start serial sur port USB    
      Serial.println("Module started");

      Serial.print("Temp BME680 = ");
      txtMsg = uint_to_string(pSsens.bme_temp);
      Serial.print(txtMsg);
      Serial.println("  (Valeur Hex à diviser par 10 pour des *C)");
      
      Serial.print("Pres BME680 = ");
      txtMsg = uint_to_string(pSsens.bme_press);
      Serial.print(txtMsg);
      Serial.println("  (Valeur Hex à diviser par 10 pour des Hpa)");
      
      Serial.print("Hum  BME680 = ");
      txtMsg = uint_to_string(pSsens.bme_hum);
      Serial.print(txtMsg);
      Serial.println("  (Valeur Hex à diviser par 100 pour des %) ");
      
      Serial.print("Gas  BME680 = ");
      txtMsg = uint_to_string(pSsens.bme_gas);
      Serial.print(txtMsg);
      Serial.println("  (Valeur Hex en  KOhms)");
      
      Serial.print("Alt  BME680 = ");
      txtMsg = uint_to_string(pSsens.bme_alt);
      Serial.print(txtMsg);
      Serial.println("  (Valeur Hex en m)"); 

      Serial.print("Temp TMP117 = ");
      txtMsg = uint_to_string(pSsens.tmp117_value);
      Serial.print(txtMsg);
      Serial.println("  (Valeur Hex à diviser par 10 pour des *C)"); 

      Serial.print("Battery     = ");
      txtMsg = uint_to_string(pSsens.batt_value);
      Serial.print(txtMsg);
      Serial.println("  (Valeur Hex à diviser par 1000 pour des V)"); 
      Serial.println(" "); 
    }
  
  #endif DEBUG_TO_USB_SERIAL
}
