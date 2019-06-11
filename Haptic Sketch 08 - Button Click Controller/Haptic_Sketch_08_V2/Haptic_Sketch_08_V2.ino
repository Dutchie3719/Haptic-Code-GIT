#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_DRV2605.h"
// Variables
boolean CLICK = true;

int accelval = 0;
int basex = 0;
int basey = 0;
int basez = 0;

// BNO055 Setup
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();
#define BNOADDR 0x28

// DRV2605L Setup
Adafruit_DRV2605 drv;
#define DRVADDR 0x5A
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Haptic Sketch No 8");
  Serial.println("by: TU Delft Pi-Touch Group");
  Serial.println("Version No: 0.1");
  Serial.println("Sensor Required: BNO055");
  Serial.println("Board Required: Arduino Uno");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  drv.begin();
  drv.selectLibrary(6);
  if (!drv.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No DRV detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

baseline();
}
void baseline()
{
  Serial.println("Calibrating...");
  
  delay (1000);
  
  imu::Vector < 3 > accelr = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print(",Baseline X: ,");
  Serial.print(accelr.x());
  Serial.print(",Baseline Y: ,");
  Serial.print(accelr.y());
  Serial.print(",Baseline Z: ,");
  Serial.print(accelr.z());
  Serial.print("\t\t");

  basex = accelr.x();
  basey = accelr.y();
  basez = accelr.z();

  Serial.println("Calibration Complete");
}

void loop()
{
  imu::Vector < 3 > accelr = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//  Serial.print(",X: ,");
//  Serial.print(accelr.x());
//  Serial.print(",Y: ,");
//  Serial.print(accelr.y());
//  Serial.print(",Z: ,");
//  Serial.print(accelr.z());
//  Serial.print("\t\t");
//  Serial.println();
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(& system, & gyro, & accel, & mag);
//  Serial.print("CALIBRATION: Sys=");
//  Serial.print(system, DEC);
//  Serial.print(" Gyro=");
//  Serial.print(gyro, DEC);
//  Serial.print(" Accel=");
//  Serial.print(accel, DEC);
//  Serial.print(" Mag=");
//  Serial.println(mag, DEC);

  int acclx = accelr.x() - basex;
  int accly = accelr.y() - basey;
  int acclz = accelr.z() - basez;
  int val = 3;
  if (acclx >= val)
  {
    playback();
    
  }
  else if (accly >= val)
  {
    playback();
    
  }
//  else if (acclz >= val)
//  {
//    playback();
//    
//  }

}

void playback(void)
{
Serial.println("PLAYBACK");
  if (CLICK == true)
  {
    CLICK = false;
    drv.selectLibrary(6);
    drv.useLRA();
    Serial.println("CLICK1");
    delay(100);
    drv.setWaveform(0, 1);
    drv.setWaveform(1, 0);
    drv.go();
    
  }
  else if (CLICK == false)
  {
    CLICK = 1;
    drv.selectLibrary(6);
    drv.useLRA();
    Serial.println("click2");
    delay(100);
    drv.setWaveform(0, 3);
    drv.setWaveform(3, 0);
    drv.go();
  }
}
