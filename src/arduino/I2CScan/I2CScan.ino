// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.


/* Teapot BWLR3D Base System Control */
/* adc to battery multiplier */
#define ADC_TO_BATTERY  0.00120919472

/* board revision */
#define BOARD_REVISION  0x02 // set this based on the board you have

/* pin definition */
#define RED_LED               PA0
#define GREEN_LED             PA1
#define PERIPHERAL_POWER_EN   PB5
#define BATT_MEASURE_EN       PB2
#define POWER_STATUS          PB12
#define GPS_FORCE_ON          PA15
#define GPS_TXD               PB7
#define GPS_RXD               PB6
#define I2C_SDA               PA11
#define I2C_SCL               PA12
#define MAG_INT               PB3
#if BOARD_REVISION == 0x01
  #define IMU_INT             PB4
  #define BATT_MEASURE        PA10
#elif BOARD_REVISION == 0x02
  #define IMU_INT             PA10
  #define BATT_MEASURE        PB4
#else
  #ifndef BOARD_REVISION
    #error "BOARD_REVISION not set"
  #endif
  #error "invalid BOARD_REVISION value"
#endif

void EnablePeripheral(bool power)
{
  if( power ){
    
    // enable GPS serial
    Serial1.begin(9600);
    
    // enable sensor I2C bus
    Wire.begin();
    
    // power-on 3.3v rail to all peripheral
    pinMode(PERIPHERAL_POWER_EN, OUTPUT);
    digitalWrite(PERIPHERAL_POWER_EN, HIGH);
  } else {
    
    // disable GPS serial
    Serial1.end();
    pinMode(GPS_TXD, INPUT);
    pinMode(GPS_RXD, INPUT);
  
    // disable I2C
    Wire.end();
    pinMode(I2C_SDA, INPUT);
    pinMode(I2C_SCL, INPUT);

    
    // power-off 3.3v rail to all peripheral
    pinMode(PERIPHERAL_POWER_EN, OUTPUT);
    digitalWrite(PERIPHERAL_POWER_EN, LOW); 
  }
}

void EnableGnss( bool enable )
{  
  if( enable )
  {
    // wake gps, incase it's off
    pinMode(GPS_FORCE_ON, OUTPUT);
    digitalWrite(GPS_FORCE_ON, HIGH);
  }
  else 
  {
    // open pin to allow GPS to go to sleep (backup mode)
    pinMode(GPS_FORCE_ON, INPUT);
    
    // prepare command
    unsigned char backup_mode_command[15];
    String base_backup_mode_command = ("$PMTK225,4*2F"); 
    const char tr[] = {0xD, 0xA};
    base_backup_mode_command.getBytes(backup_mode_command, sizeof(backup_mode_command));
    memcpy(backup_mode_command+13, tr, 2); 

    // send sleep command to gps
    Serial1.write(backup_mode_command, sizeof(backup_mode_command));

    // clear incoming uart data
    Serial1.flush();
    while (Serial1.available()) Serial1.read();
  }
}

void Initialize(){    
  // disable unnecessary pin
  pinMode(MAG_INT, INPUT);
  pinMode(IMU_INT, INPUT);
  pinMode(POWER_STATUS, INPUT);
  
  // setup battery read measure
  analogReadResolution(14);
  pinMode(BATT_MEASURE, INPUT);
  pinMode(BATT_MEASURE_EN, OUTPUT);
  digitalWrite(BATT_MEASURE_EN, LOW);
  
  // setup led pin
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, HIGH);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);

  // initialize and sleep GNSS
  EnablePeripheral(true);
  delay( 1000 );
  EnableGnss( false );
  delay( 1000 );
  
  // power-off all peripheral
  EnablePeripheral(false);
  delay( 1000 );
}

int ReadBatteryAdc()
{
  // TODO: investigate. it seems pin always disabled at every loop.
  //       always set to output at every ADC read.
  pinMode(BATT_MEASURE_EN, OUTPUT);
  digitalWrite(BATT_MEASURE_EN, HIGH);
  const int battery_adc = analogRead( BATT_MEASURE );
  digitalWrite(BATT_MEASURE_EN, LOW);
  return battery_adc;
}

float ReadBatteryVoltage()
{
  return ((float) ReadBatteryAdc()) * ADC_TO_BATTERY;
}

bool ReadPowerStatus()
{
  // TODO: investigate. it seems pin always disabled at every loop.
  //       always set to input at every digital read.
  pinMode(POWER_STATUS, INPUT);
  return digitalRead(POWER_STATUS);
}
/* Teapot BWLR3D Base System Control */

void setup() {
  api.system.restoreDefault();
  Serial.begin(115200);

  // initialize system
  Initialize();
  
  // power-on all peripheral
  EnablePeripheral( true );
  delay( 1000 ); 

  Wire.begin();
  Wire.setClock(400000);
}

void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
