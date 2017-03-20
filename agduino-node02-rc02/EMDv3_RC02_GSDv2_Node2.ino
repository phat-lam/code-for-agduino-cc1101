/*
 * Simple code for wsn id 02 in mushroom growing room. 
 * Reads sensors and sends data to gateway by RF wireless transceiver CC1101. 
 * Timer period: 10 seconds.
 * 
 * Atmega328p-au MCU with arduino bootloader (or Arduino Nano).
 * Data transceiver: rfic CC1101 - UART serial interface, frequency: 433MHz, default baud-rate: 9600 kbps.
 * Sensors: temperature & relative humidity SHT11, light intensity BH1750, CO2 concentration sensor RC-02.
 * 
 * The circuit:
 * D2: SHT11 clock pin.
 * D6: SHT11 data pin.
 * A4: BH1750 SDA pin.
 * A5: BH1750 SCL pin.
 * A1: RC-02 analog output.
 * RXD(D0): TXD pin (RF transceiver CC1101).
 * TXD(D1): RXD pin (RF transceiver CC1101).
 * 
 * Created 18 Mar 2017 by AGCT.
 */
// I2C lib for light intensity sensor. Comes with IDE.
#include <Wire.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
// SHT lib for T&RH sensor
#include "SHT1x.h"
// Timer lib
#include "SimpleTimer.h"

SimpleTimer timer;
/*-----( Declare Constants and Pin Numbers )------*/
// WSN ID. "002" shows this node in Network 0 and has ID 01.
char node_id[] = "002";
#define sht_dataPin 6
#define sht_clockPin 2
// CO2 sensor
int RC02_analog_pin = A1;         
int RC02_read_sample_times = 10;     
int read_sample_interval = 50;
// Timer period
int timer_period = 10000;
/*-----( Declare Objects and Variables )----------*/
// Data to transmit
char rf_dataTransmitted[60];
// SHT sensor
SHT1x sht1x(sht_dataPin, sht_clockPin);
// I2C address BH1750
int BH1750address = 0x23;
byte buff[2];
// CO2 concentration
int RC02_ppmCO2 = 0;
// Soil moisture and temperature 5TM
float soil_dielctric = 0, soil_temp = 0;
// Data(t, h, li, ppmCO2, soil_dielctric, soil_temp) in char-array
static char dtostrfbuffer1[4];
static char dtostrfbuffer2[3];
static char dtostrfbuffer3[6];
static char dtostrfbuffer4[5];
static char dtostrfbuffer5[5];
static char dtostrfbuffer6[4];
/*----- SETUP: RUNS ONCE -------------------------*/
void setup()
{
  // Initialize the serial communications. Set speed to 9600 kbps.
  Serial.begin(9600);
  // Initialize the I2C communications.
  Wire.begin();
  // Delay for 2 second.
  delay(2000);
  // Run rf_TransmitDataFunc every 'period' 10 seconds.
  timer.setInterval(timer_period, rf_TransmitDataFunc);                                              // Run read_Data() fuction every 10 seconds
}
/*----- End SETUP ---------------------------------*/
/*----- LOOP: RUNS CONSTANTLY ---------------------*/
void loop()
{ 
  timer.run();
}
/*----- End LOOP ----------------------------------*/
/*-----( Declare User-written Functions )----------*/
void rf_TransmitDataFunc()  
{
  // Read humidity
  float h = sht1x.readHumidity();
  // Read temperature as Celsius (the default)
  float t = sht1x.readTemperatureC();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = sht1x.readTemperatureF();
  
  // Read light intensity
  uint16_t li=0;
  BH1750_Init(BH1750address);
  if(2==BH1750_Read(BH1750address))
  {
    li=((buff[0]<<8)|buff[1])/1.2;
  } else li = 65535;

  // Read CO2 concentration
  float adcVolts = RC02_Read(RC02_analog_pin, RC02_read_sample_times);
  RC02_ppmCO2 = RC02_GetPercentage(adcVolts);

  // Convert all data into char-array
  dtostrf((int)(t*10),3,0,dtostrfbuffer1);
  dtostrf((int)h,2,0,dtostrfbuffer2);
  dtostrf(li,5,0,dtostrfbuffer3);
  dtostrf(RC02_ppmCO2,4,0,dtostrfbuffer4);
  dtostrf(soil_dielctric * 100,4,0,dtostrfbuffer5);
  dtostrf(soil_temp * 10,3,0,dtostrfbuffer6);

  // Connect all char-array into one
  sprintf(rf_dataTransmitted,"S%sT%sH%sL%sC%sD%sP%sE", node_id, dtostrfbuffer1, dtostrfbuffer2, dtostrfbuffer3, dtostrfbuffer4, dtostrfbuffer5, dtostrfbuffer6);  
  // Print all sensor values. Data will be transmited to Gateway by RF CC1101 module.
  // End with "\n\r"
  Serial.print(rf_dataTransmitted); Serial.print("\n\r");

  // Clear all variables
  t = 0;
  h = 0;
  li = 0;
  RC02_ppmCO2 = 0;
  soil_dielctric = 0;
  soil_temp = 0;                                                
}
/*-------Sub-fuction---------------------------------*/
// Read light intensity
int BH1750_Read(int address) 
{
  int i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()) 
  {
    buff[i] = Wire.read();// receive one byte
    i++;
  }
  Wire.endTransmission();  
  return i;
}
// Initialize light intensity sensor
void BH1750_Init(int address) 
{
  Wire.beginTransmission(address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();
}
// Read CO2 concentration
float RC02_Read(int i_rcPin, int read_sample_times) 
{
  int i;
  float v = 0;
 
  for (i = 0; i < read_sample_times; i++) {
    v += analogRead(i_rcPin);
    delay(read_sample_interval);
  }
  v = (v / read_sample_times) * 5 / 1023;
  return v;
}
// Convert into ppm, the range: 0 - 2000 ppm.
int RC02_GetPercentage(float volts) 
{
  float ppmCO2 = (float)(volts / 5) * 2000; 
  if (ppmCO2 < 0) ppmCO2 = 0;
  if (ppmCO2 > 2000) ppmCO2 = 2000;
  return (int)ppmCO2;
}
/*-----( THE END )------------------------------------*/
