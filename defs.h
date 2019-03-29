/*
 * defs.h
 *
 * Created: 26/03/2019 10:41:34
 *  Author: KROA38
 */ 
#ifndef BLUEDUINO_H_
#define BLUEDUINO_H_
/* Include ------------------------------------------------------------------*/
#include <EEPROM.h>
#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control
#include <AB_BLE.h>
#include <TMP117.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <Wire.h>
/* Pre-Processor directive --------------------------------------------------*/
#undef USB_POWERED
#undef  DEBUG_TO_USB_SERIAL
/* Private define ------------------------------------------------------------*/
#define TMP117_ADDR 0x48
#define SEALEVELPRESSURE_HPA 1026.2
#define ADC_REFERENCE 2.56
#define MAJOR 100
#define DONE_PIN 7
#define TEST_PIN 9
#define BATT_PIN A0
#define UUID_PREFIX "2332A4C2"    // UUID prefix
#define UUID_SENSOR "01"          // put here the sensor number
#define UUID_LENGTH 41
#define MAJOR_LENGTH 13
#define MINOR_LENGTH 13
#define WDT_16MS 0b000000
#define WDT_500MS 0b000101
#define WDT_1S 0b000110
#define WDT_4S 0b100000
#define WDT_8S 0b100001
/*  MACRO  -----------------------------------------------------------------*/

/* Private enum  ------------------------------------------------------------*/
enum BME_MODE
{
	BME_ON = 0,
	BME_OFF,
};

/* Structures ---------------------------------------------------------*/
typedef struct{
	uint16_t batt_value;
	uint16_t tmp117_value;
	uint16_t bme_temp;
	uint16_t bme_hum;
	uint16_t bme_press;
	uint16_t bme_alt;
	uint16_t bme_gas;
}Ssensor;
/* Private function prototypes -----------------------------------------------*/
void ble_write_config(void);
void ble_write_uuid(void);
void ble_write_major(void);
void ble_write_minor(void);
void ble_write_ibeacon(void);
void bsp_set_config(void);
void adc_read_data(void);
void tmp117_read_data(void);
void bme680_configure(byte);
void bme680_read_data(void);
String uint_to_string(uint16_t );
void ble_build_uuid(void);
void ble_build_major(void);
void ble_build_minor(void);
bool ble_send_AT(char *,byte );
void configure_wdt(void);
void sleep(int );
void test_pin(byte);
void reset_by_wdt(void);



#endif /* BLUEDUINO_H_ */
