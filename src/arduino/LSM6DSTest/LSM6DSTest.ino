/******************************************************************************
LowLevelExample.ino

Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

Description:
Example using the LSM6DS3 with ONLY read and write methods.  It's up to you to
read the datasheets and get the sensor to behave as you will.

This sketch saves a significant amount of memory because the settings and complex
math (such as floating point variables) don't exist.  The cost of saved memory is
that the values are in 'counts', or raw data from the register.  The user is
responsible for converting these raw values into something meaningful.

Use the register words from SparkFunLSM6DS3.h to manually configure the IC.

Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation

Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23

Hardware connections:
Connect I2C SDA line to A4
Connect I2C SCL line to A5
Connect GND and 3.3v power to the IMU

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.

Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "SPI.h"

uint16_t errorsAndWarnings = 0;
#define PERIPHERAL_POWER_EN PB5
#define GPS_FORCE_ON PA15

LSM6DS3Core myIMU( I2C_MODE, 0x6A );

void setup() {
  api.system.restoreDefault();
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000); //relax...
  Serial.println("Processor came out of reset.\n");
  
  Serial1.begin(9600);
  // power-on all peripheral
  pinMode(PERIPHERAL_POWER_EN, OUTPUT);
  digitalWrite(PERIPHERAL_POWER_EN, HIGH);
  // open pin to allow GPS to go to sleep (backup mode)
  pinMode(GPS_FORCE_ON, INPUT);
  
  
  //Call .beginCore() to configure the IMU
  if( myIMU.beginCore() != 0 )
  {
    Serial.print("Error at beginCore().\n");
  }
  else
  {
    Serial.print("\nbeginCore() passed.\n");
  }
  
  uint8_t dataToWrite = 0;  //Temporary variable

  //Setup the accelerometer******************************
  dataToWrite = 0; //Start Fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

  //Now, write the patched together data
  errorsAndWarnings += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  //Set the ODR bit
  errorsAndWarnings += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

  // set gps to sleep
  delay(2000);
  setGPSToBackupMode();

}

static void setGPSToBackupMode()
{
  // prepare command
  unsigned char backup_mode_command[15];
  String base_backup_mode_command = ("$PMTK225,4*2F"); 
  const char tr[] = {0xD, 0xA};
  base_backup_mode_command.getBytes(backup_mode_command, sizeof(backup_mode_command));
  memcpy(backup_mode_command+13, tr, 2); 

  // for debug, print actual backup mode command
  Serial.print("Backup Mode Command: ");
  for( int i = 0; i < sizeof(backup_mode_command); i++ ) {  
    Serial.print("0x");
    Serial.print(backup_mode_command[i],HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.println("Set GPS to Backup mode");
  Serial1.write(backup_mode_command, sizeof(backup_mode_command));
}

void loop()
{
  int16_t temp;
  //Get all parameters
  Serial.print("\nAccelerometer Counts:\n");
  Serial.print(" X = ");
  
  //Read a register into the temp variable.
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTX_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }
  Serial.println(temp);
  Serial.print(" Y = ");

  //Read a register into the temp variable.
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTY_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }
  Serial.println(temp);
  Serial.print(" Z = ");

  //Read a register into the temp variable.
  if( myIMU.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTZ_L_XL) != 0 )
  {
    errorsAndWarnings++;
  }
  Serial.println(temp);
  
  Serial.println();
  Serial.print("Total reported Errors and Warnings: ");
  Serial.println(errorsAndWarnings);
  
  delay(1000);
}
