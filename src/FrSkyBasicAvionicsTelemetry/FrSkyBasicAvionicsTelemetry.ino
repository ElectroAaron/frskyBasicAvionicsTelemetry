/*

                         FRSKY BASIC AVIONICS TELEMETRY

   Open source Arduino based FrSky basic avionics telemetry system for RC aircrafts.
   Arduino uses a GB-1803 GPS module, a BMP280 digital barometer and a resistor divider
   filtered with a capacitor; to obtain navigation data, altitude data and battery
   voltage data. Using FrSkySportTelemetry library, Arduino emulates FrSky sensors and
   display all data in the transmitter LCD. Adafruit_BMP280_Library and TinyGPS libraries
   need to be modified in order to work properly with BMP280 running through I2C and
   GLONASS GNSS modules.



   Aaron G.
   Apr 2020
*/

#include <TimerOne.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include "FrSkySportSensor.h"
#include "FrSkySportSensorFcs.h"
#include "FrSkySportSensorVario.h"
#include "FrSkySportSingleWireSerial.h"
#include "FrSkySportSensorGps.h"
#include "FrSkySportSensorRpm.h"
#include "FrSkySportTelemetry.h"
#include "SoftwareSerial.h"



#define BATT_PER_CELL             //Show battery voltage per cell instead of total voltage.
#define VOLTAGE_PIN       A6        //Analog pin where voltage sensor is connected.
#define MIN_BAT           3.50      //V per cell when consider battery is empty.
#define MAX_BAT           4.35      //V per cell at full charge.
#define VOLTAGE_SAMPLES   100       //Number of samples to filter ADC values.
#define ADC_RATE          1.8026462498587626450679469109248     //AGH Ion Pro
//1.6285492767215723196160421580917     //M. CAS II       //ADC multiplier to get volts from ADC level (0 - 1023).
//1.6899302001269946828808887836638     //M. CAS I



#define GPS_SERIAL        Serial
#define GPS_DELAY         100       //ms

//#define BARO_SEA_ALT              //Show barometer absolute altitude (from sea level) instead of relative altitude (from ground). Default is relative.
#define VSPD_SAMPLES      40
#define VSPD_MAX_SAMPLES  50

#define UPDATE_DELAY      1000      //FrSky SmartPort update period (us).
#define LED_PIN           13        //Status LED, will go on after start up when system is ready to go.

#ifdef BARO_SEA_ALT
#define SEA_PRESSURE      1013.25   //Default sea pressure (hPa)
#endif



Adafruit_BMP280         baroSensor;
TinyGPSPlus             gpsSensor;
FrSkySportSensorFcs     fcsFrSky;
FrSkySportSensorGps     gpsFrSky;
FrSkySportSensorVario   varioFrSky;
FrSkySportSensorRpm     rpmFrSky;
FrSkySportTelemetry     telemetry;



int cellNum;
double mainVolts, cellVolts, batteryPercent;

int numSats;
double gpsLatitude, gpsLongitude, gpsSpeed, gpsCourse, altitudeGPS;
unsigned long age, startTime, endTime;

double bmpP0;
double baroAltitude = 0.0, verticalSpeed;
double tempo = millis();
double N1 = 0, N2 = 0, N3 = 0, D1 = 0, D2 = 0;
double alt[51];
double tim[51];



float readVoltage(void) {
  long voltageSum = 0;
  for (int i = 0; i < VOLTAGE_SAMPLES; i++) {
    voltageSum += analogRead(VOLTAGE_PIN);
    delayMicroseconds(20);
  }
  int computeAverage = (int)(voltageSum / (float)VOLTAGE_SAMPLES);
  mainVolts = (computeAverage * ADC_RATE) / 100.0;
  cellVolts = mainVolts / (float)cellNum;
  if ( (cellVolts - MIN_BAT) >= (MAX_BAT - MIN_BAT) ) batteryPercent = 100.0;
  else if ((cellVolts - MIN_BAT) <= 0.0 ) batteryPercent = 0.0;
  else batteryPercent = ((cellVolts - MIN_BAT) * 100.0) / (MAX_BAT - MIN_BAT);
}



void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  delay(100);
  analogReference(INTERNAL);        //Set analog reference to ATMega328P internal 1V1. This way, voltage readings will not be affected by different Vcc voltages from different systems.
  GPS_SERIAL.begin(57600);
  telemetry.begin(FrSkySportSingleWireSerial::SOFT_SERIAL_PIN_3, &fcsFrSky, &gpsFrSky, &varioFrSky, &rpmFrSky);
  baroSensor.begin();
  delay(2500);
  bmpP0 = baroSensor.readPressure() / 100.0;    //Set actual default ground pressure to calculate relative altitude, this way it will be 0m at this point (ground).
  Timer1.initialize(UPDATE_DELAY);              //Interruption used to send data to FrSky SmartPort.
  Timer1.attachInterrupt(sendData2SPort);
  interrupts();
  readVoltage();
  if (mainVolts <= 17.50)
    cellNum = 4;          //Get number of cells from total voltage.
  if (mainVolts <= 12.70)
    cellNum = 3;
  if (mainVolts <= 8.50)
    cellNum = 2;
  digitalWrite(LED_PIN, HIGH);                 //Turn on LED when ready to go.
}



void loop() {
  //Voltage
  readVoltage();            //Read voltage and battery percent

  //GPS
  startTime = millis();
  do {
    while (GPS_SERIAL.available()) gpsSensor.encode(GPS_SERIAL.read());
    endTime = millis();
  } while ( (endTime - startTime) < GPS_DELAY);

  if (gpsSensor.location.isValid()) {
    gpsLatitude = gpsSensor.location.lat();
    gpsLongitude = gpsSensor.location.lng();
  } else {
    gpsLatitude = 0.0000000;
    gpsLongitude = 0.0000000;
  }


  if (gpsSensor.speed.isValid()) {
    gpsSpeed = gpsSensor.speed.mps();
  } else {
    gpsSpeed = 0.0;
  }

  if (gpsSensor.altitude.isValid()) {
    altitudeGPS = gpsSensor.altitude.meters();
  } else {
    altitudeGPS = 0.0;
  }

  if (gpsSensor.course.isValid()) {
    gpsCourse = gpsSensor.course.deg();
  } else {
    gpsCourse = 0.0;
  }

/*
  if (gpsSensor.satellites() > 50) numSats = 0;
  else numSats = gpsSensor.satellites();

*/


  //Barometer
#ifdef BARO_SEA_ALT
  baroAltitude = baroSensor.readAltitude(SEA_PRESSURE);
#else
  baroAltitude = baroSensor.readAltitude(bmpP0);
#endif
  tempo = millis();
  N1 = 0;
  N2 = 0;
  N3 = 0;
  D1 = 0;
  D2 = 0;
  verticalSpeed = 0.0;
  for (int cc = 1; cc <= VSPD_MAX_SAMPLES; cc++) {
    alt[(cc - 1)] = alt[cc];
    tim[(cc - 1)] = tim[cc];
  }
  alt[VSPD_MAX_SAMPLES] = baroAltitude;
  tim[VSPD_MAX_SAMPLES] = tempo;
  float stime = tim[VSPD_MAX_SAMPLES - VSPD_SAMPLES];
  for (int cc = (VSPD_MAX_SAMPLES - VSPD_SAMPLES); cc < VSPD_MAX_SAMPLES; cc++) {
    N1 += (tim[cc] - stime) * alt[cc];
    N2 += (tim[cc] - stime);
    N3 += (alt[cc]);
    D1 += (tim[cc] - stime) * (tim[cc] - stime);
    D2 += (tim[cc] - stime);
  }
  verticalSpeed = 1000 * ((VSPD_SAMPLES * N1) - N2 * N3) / (VSPD_SAMPLES * D1 - D2 * D2);


#ifdef BATT_PER_CELL
  fcsFrSky.setData(0, cellVolts);
#else
  fcsFrSky.setData(0, mainVolts);
#endif
  gpsFrSky.setData(gpsLatitude, gpsLongitude, altitudeGPS, gpsSpeed, gpsCourse, 0, 0, 0, 0, 0, 0);
  varioFrSky.setData(baroAltitude, verticalSpeed);
  rpmFrSky.setData(numSats, baroSensor.readTemperature(), batteryPercent);
}



//Send data to RX SmartPort
void sendData2SPort(void) {
  telemetry.send();
}
