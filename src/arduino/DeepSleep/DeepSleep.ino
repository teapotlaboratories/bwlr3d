/* Teapot BWLR3D Base System Control */
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
/* Teapot BWLR3D Base System Control */


void setup()
{
  api.system.restoreDefault();
  Serial.begin(115200);
  
  Initialize();
  Serial.println("RAKwireless System Powersave Example");
  Serial.println("------------------------------------------------------");
}

void loop()
{
  Serial.print("The timestamp before sleeping: ");
  Serial.print(millis());
  Serial.println(" ms");
  Serial.println("(Wait 600 seconds or Press any key to wakeup)");
  api.system.sleep.all(600000);
  Serial.print("The timestamp after sleeping: ");
  Serial.print(millis());
  Serial.println(" ms");
}
