/*
 * Simple code for wsn id 01 which is out-side of mushroom growing room. 
 * Reads sensors and sends data to gateway by RF wireless transceiver CC1101. 
 * Timer period: 10 seconds.
 * 
 * Atmega328p-au MCU with arduino bootloader.
 * Data transceiver: rfic CC1101 - UART serial interface, frequency: 433MHz, default baud-rate: 9600 kbps.
 * Sensors: temperature & relative humidity DHT21, light intensity BH1750.
 * 
 * The circuit:
 * D2: DHT21 data pin.
 * A4: BH1750 SDA pin.
 * A5: BH1750 SCL pin.
 * RXD(D0): TXD pin (RF transceiver CC1101).
 * TXD(D1): RXD pin (RF transceiver CC1101).
 * 
 * Created 18 Mar 2017 by AGCT.
 */
// I2C lib for light intensity sensor. Comes with IDE.
#include <Wire.h>
// Timer lib
#include "SimpleTimer.h"
// DHT lib for T&RH sensor
#include "DHT.h"

SimpleTimer timer;
/*-----( Declare Constants and Pin Numbers )------*/
// WSN id
char node_id[] = "001";
// DHT sensor data pin
#define DHTPIN  2
// Timer period
int timer_period = 10000;
/*-----( Declare Objects and Variables )----------*/
// data to transmit
char rf_dataTransmitted[60];
// DHT sensor
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);
// I2C address BH1750
int BH1750address = 0x23;
byte buff[2];
// CO2 concentration
int ppmCO2 = 0;
// Soil moisture and temperature 5TM
float soil_dielctric, soil_temp;
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
  // Delay for 1 second.
  delay(1000);
  // Run rf_TransmitDataFunc every 'period' 10 seconds.
  timer.setInterval(timer_period, rf_TransmitDataFunc);
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
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) 
  {
    //Serial.println("Failed to read DHT sensor!");
    return;
  }
  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)                                     
  float hic = dht.computeHeatIndex(t, h, false);                                

  // No light intensity sensor
  uint16_t li=0;
  // No CO2 sensor
  ppmCO2 = 0;
  // No soil sensor
  soil_dielctric = 0;
  soil_temp = 0;
  
  // Convert all data into char-array
  dtostrf((int)(t*10),3,0,dtostrfbuffer1);
  dtostrf((int)h,2,0,dtostrfbuffer2);
  dtostrf(li,5,0,dtostrfbuffer3);
  dtostrf(ppmCO2,4,0,dtostrfbuffer4);
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
  ppmCO2 = 0;
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
    // receive one byte
    buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
  return i;
}
// Initialize light intensity sensor
void BH1750_Init(int address) 
{
  Wire.beginTransmission(address);
  //1lx reolution 120ms
  Wire.write(0x10);
  Wire.endTransmission();
}
/*-----( THE END )------------------------------------*/

