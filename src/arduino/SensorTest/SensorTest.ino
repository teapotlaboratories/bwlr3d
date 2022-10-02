#include "Adafruit_LIS3MDL.h"
#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_BME680.h"
#include "Adafruit_VEML7700.h"

#define SEALEVELPRESSURE_HPA (1013.25)

uint16_t errorsAndWarnings = 0;

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
Adafruit_BME680 bme; // I2C
Adafruit_VEML7700 veml;
void setup(void) {
  /* initialize system pin */
  initializeSystem();
  /* initialize sensor */
  enableSensorCommunication();
  /* configure sensor settings */
  configureSensor();
  
  /* configure GPS */
  Serial.println("================ Setup GPS ================");
  delay(2000);
  pinMode(GPS_FORCE_ON, INPUT); // open pin to allow GPS to go to sleep (backup mode)
  // set gps to sleep
  setGPSToBackupMode();
  
}

void loop() {  
  /* disable sensor comm and then sleep */
  disableSensorCommunication();
  api.system.sleep.all(10000);

  Serial.println();
  Serial.println("================ Sensor Wakes Up ================");
  enableSensorCommunication();
  /* read sensor data */
  readSensorData();
}

void initializeSystem(){  
  api.system.restoreDefault();
  Serial.begin(115200);
  
  // power-on all peripheral
  pinMode(PERIPHERAL_POWER_EN, OUTPUT);
  digitalWrite(PERIPHERAL_POWER_EN, HIGH);

  // disable unnecessary pin
  pinMode(MAG_INT, INPUT);
  pinMode(IMU_INT, INPUT);
  pinMode(POWER_STATUS, INPUT);
  pinMode(BATT_MEASURE, INPUT);
  pinMode(BATT_MEASURE_EN, INPUT);
  pinMode(LED0, INPUT);
  pinMode(LED1, INPUT);

  pinMode(GPS_FORCE_ON, INPUT); // open pin to allow GPS to go to sleep (backup mode)
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

void readSensorData(){
  /* Get a new normalized sensor event */
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag; 


  /* Read lux sensor */
  Serial.println("================ VEML7700 Sensor ================");
  Serial.print("raw ALS: "); Serial.println(veml.readALS());
  Serial.print("raw white: "); Serial.println(veml.readWhite());
  Serial.print("lux: "); Serial.println(veml.readLux());

  /* Read Mag sensor */
  Serial.println("================ LIS3MDL Sensor ================");
  lis3mdl.getEvent(&mag);
  /* Display the results (magnetic field is measured in uTesla) */
  Serial.print("X: "); Serial.print(mag.magnetic.x);
  Serial.print(" \tY: "); Serial.print(mag.magnetic.y); 
  Serial.print(" \tZ: "); Serial.print(mag.magnetic.z); 
  Serial.println(" uTesla ");

  /* Read IMU sensor */
  Serial.println("================ LSM6DS0X Sensor ================");
  sox.getEvent(&accel, &gyro, &temp);
  Serial.print("Temperature ");
  Serial.print(temp.temperature);
  Serial.println(" deg C");

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("Accel X: ");
  Serial.print(accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("Gyro X: ");
  Serial.print(gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");

  Serial.println("================ BME68x Sensor ================");
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
}

void configureSensor(){

  /* configure LIS3MDL */
  Serial.println("================ Configure LIS3MDL ================");
  Serial.println(F("Initialize LIS3MDL"));
  if (! lis3mdl.begin_I2C(0x1E, &Wire)) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid LIS3MDL sensor, check wiring!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("LIS3MDL Found!");
  setupLis3mdl();
  Serial.println();
  
  /* configure LSM6DS0X */
  Serial.println("================ Configure LSM6DS0X ================");
  Serial.println(F("Initialize LSM6DSOX"));
  if (!sox.begin_I2C()) {
    Serial.println("Could not find a valid LSM6DSOX sensor, check wiring!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("LSM6DSOX Found!");
  setupLsm6ds0x();
  Serial.println();

  /* configure BME68x */
  Serial.println("================ Configure BME68x ================");
  Serial.println(F("Initialize BME68x"));
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME68x sensor, check wiring!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("BME68x Found!");
  setupBme68x();
  Serial.println();

  /* configureconfigure BME68x */
  Serial.println("================ Configure VEML7700 ================");
  Serial.println(F("Initialize VEML7700"));
  if (!veml.begin()) {
    Serial.println("Could not find a valid VEML7700 sensor, check wiring!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("VEML7700 Found!");
  setupVeml7700();
  Serial.println();
}

void setupVeml7700(){
  
  Serial.println("Setup VEML7700");
  Serial.print(F("Gain: "));
  switch (veml.getGain()) {
    case VEML7700_GAIN_1: Serial.println("1"); break;
    case VEML7700_GAIN_2: Serial.println("2"); break;
    case VEML7700_GAIN_1_4: Serial.println("1/4"); break;
    case VEML7700_GAIN_1_8: Serial.println("1/8"); break;
  }

  Serial.print(F("Integration Time (ms): "));
  switch (veml.getIntegrationTime()) {
    case VEML7700_IT_25MS: Serial.println("25"); break;
    case VEML7700_IT_50MS: Serial.println("50"); break;
    case VEML7700_IT_100MS: Serial.println("100"); break;
    case VEML7700_IT_200MS: Serial.println("200"); break;
    case VEML7700_IT_400MS: Serial.println("400"); break;
    case VEML7700_IT_800MS: Serial.println("800"); break;
  }

  veml.setLowThreshold(10000);
  veml.setHighThreshold(20000);
  veml.interruptEnable(false);
}


void setupLis3mdl(){
  
  Serial.println("Setup LIS3MDL");
  lis3mdl.setPerformanceMode(LIS3MDL_LOWPOWERMODE);
  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_POWERDOWNMODE);
  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_0_625_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          false); // disable!
}

void setupLsm6ds0x(){
  
  Serial.println("Setup LSM6DSOX");
  /* setup LSM6DSOX */
  // sox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

//  sox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  // disable accel
  sox.setAccelDataRate(LSM6DS_RATE_SHUTDOWN);
  Serial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

//  sox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  // disable gyro
  sox.setGyroDataRate(LSM6DS_RATE_SHUTDOWN );
  Serial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
}

void setupBme68x(){  
  Serial.println("Setup BME68X");
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

  // for debug, print actual backup mode command
//  Serial.print("Backup Mode Command: ");
//  for( int i = 0; i < sizeof(backup_mode_command); i++ ) {  
//    Serial.print("0x");
//    Serial.print(backup_mode_command[i],HEX);
//    Serial.print(" ");
//  }
//  Serial.println();
  
  Serial.println("Set GPS to Backup mode");
  Serial1.write(backup_mode_command, sizeof(backup_mode_command));
}
