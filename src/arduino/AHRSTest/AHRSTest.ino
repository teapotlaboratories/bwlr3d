#include "Adafruit_LIS3MDL.h"
#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_BME680.h"
#include "Adafruit_VEML7700.h"
#include "Adafruit_AHRS_Mahony.h"
#include "Adafruit_AHRS_Madgwick.h"

#define SEALEVELPRESSURE_HPA (1013.25)

// Teapot BWLR3D pins
#define LED0                PA0
#define LED1                PA1
#define PERIPHERAL_POWER_EN PB5
#define BATT_MEASURE_EN     PB2
#define POWER_STATUS        PB12
#define GPS_FORCE_ON        PA15
#define GPS_TXD             PB7
#define GPS_RXD             PB6
#define I2C_SDA             PA11
#define I2C_SCL             PA12
#define MAG_INT             PB3
#define IMU_INT             PB4   // new rev is PA10
#define BATT_MEASURE        PA10  // new rev is PB4

Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL lis3mdl;
Adafruit_BME680 bme;
Adafruit_VEML7700 veml;

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each baord/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { 2.45F, -4.55F, -26.93F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.961,  -0.001,  0.025 },
                                    {  0.001,  0.886,  0.015 },
                                    {  0.025,  0.015,  1.176 } };

float mag_field_strength        = 44.12F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
// Raw values converted to rad/s based on 250dps sensitiviy (1 lsb = 0.00875 rad/s)
float rawToDPS = 0.00875F;
float dpsToRad = 0.017453293F;
float gyro_zero_offsets[3]      = { 175.0F * rawToDPS * dpsToRad,
                                   -729.0F * rawToDPS * dpsToRad,
                                    101.0F * rawToDPS * dpsToRad };


// Mahony is lighter weight as a filter and should be used
// on slower systems
Adafruit_Mahony filter;
//Adafruit_Madgwick filter;

void setup()
{
  Serial.println("Starting Teapot BWL3RD.");
  /* initialize system pin */
  initializeSystem();
  /* initialize sensor */
  enableSensorCommunication();
  /* configure sensor settings */
  configureSensor();
  
  /* configure GPS */
  // wait for gps to boot
  delay(2000);
  // set gps to sleep
  setGPSToBackupMode();

  // Filter expects 25 samples per second
  // The filter only seems to be responsive with a much smaller value, though!
  // ToDo: Figure out the disparity between actual refresh rate and the value
  // provided here!
  filter.begin(5);
}

void loop(void)
{
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t temp_event;
  
  /* Read Sensor Data */
  lis3mdl.getEvent(&mag_event);
  sox.getEvent(&accel_event, &gyro_event, &temp_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x - gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y - gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z - gyro_zero_offsets[2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  
  // Print the orientation filter output
//  float roll = filter.getRoll();
//  float pitch = filter.getPitch();
//  float heading = filter.getYaw();
//  Serial.print(millis());
//  Serial.print("Orientation: ");
//  Serial.print(pitch);
//  Serial.print(", ");
//  Serial.print(heading);
//  Serial.print(". ");
//  Serial.println(roll);

//
//  // Print the orientation filter output in quaternions.
//  // This avoids the gimbal lock problem with Euler angles when you get
//  // close to 180 degrees (causing the model to rotate or flip, etc.)
  float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);
  Serial.print(millis());
  Serial.print("Quaternion: ");
  Serial.print(qw);
  Serial.print(", ");
  Serial.print(qx);
  Serial.print(", ");
  Serial.print(qy);
  Serial.print(", ");
  Serial.println(qz);

  delay(10);
}


void initializeSystem(){  
  api.system.restoreDefault();
  Serial.begin(115200);

  // initialize power pin
  pinMode(PERIPHERAL_POWER_EN, OUTPUT);
  
  // disable sensor communication
  disableSensorCommunication();
  
  // disable unnecessary pin
  pinMode(MAG_INT, INPUT);
  pinMode(IMU_INT, INPUT);
  pinMode(POWER_STATUS, INPUT);
  pinMode(BATT_MEASURE, INPUT);
  pinMode(BATT_MEASURE_EN, INPUT);
  pinMode(LED0, INPUT);
  pinMode(LED1, INPUT);

  pinMode(GPS_FORCE_ON, INPUT); // open pin to allow GPS to go to sleep (backup mode)
    
  // power-on all peripheral
  digitalWrite(PERIPHERAL_POWER_EN, LOW);
  delay(1000);
  digitalWrite(PERIPHERAL_POWER_EN, HIGH);
  delay(1000);

}

void enableSensorCommunication() {
  // enable GPS serial
  Serial1.begin(9600);
  
  // enable sensor I2C bus
  Wire.begin();
}

void disableSensorCommunication() {
  // disable GPS serial
  Serial1.end();
  pinMode(GPS_TXD, INPUT);
  pinMode(GPS_RXD, INPUT);

  // disable I2C
  Wire.end();
  pinMode(I2C_SDA, INPUT);
  pinMode(I2C_SCL, INPUT);
}

void configureSensor(){

  /* configure LIS3MDL */
  if (! lis3mdl.begin_I2C(0x1E, &Wire)) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid LIS3MDL sensor, check wiring!");
    while (1) {
      delay(1000);
    }
  }
  setupLis3mdl();
  
  /* configure LSM6DS0X */
  if (!sox.begin_I2C()) {
    Serial.println("Could not find a valid LSM6DSOX sensor, check wiring!");
    while (1) {
      delay(1000);
    }
  }
  setupLsm6ds0x();

  /* configure BME68x */
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME68x sensor, check wiring!");
    while (1) {
      delay(1000);
    }
  }
  setupBme68x();
  
  /* configure BME68x */
  if (!veml.begin()) {
    Serial.println("Could not find a valid VEML7700 sensor, check wiring!");
    while (1) {
      delay(1000);
    }
  }
  // no need to setup VEML7700, run in auto mode
}

void setupLis3mdl(){
  
  /* setup LIS3MDL */
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);  
  lis3mdl.setRange(LIS3MDL_RANGE_16_GAUSS);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          false); // disable!
}

void setupLsm6ds0x(){
 
  /* setup LSM6DSOX */
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS );
  sox.setAccelDataRate(LSM6DS_RATE_6_66K_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_6_66K_HZ);
}

void setupBme68x(){  

  /* setup BME68x */
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

static void setGPSToBackupMode()
{
  // prepare command
  unsigned char backup_mode_command[15];
  String base_backup_mode_command = ("$PMTK225,4*2F"); 
  const char tr[] = {0xD, 0xA};
  base_backup_mode_command.getBytes(backup_mode_command, sizeof(backup_mode_command));
  memcpy(backup_mode_command+13, tr, 2); 
  
  // send command to GPS
  Serial1.write(backup_mode_command, sizeof(backup_mode_command));
}
