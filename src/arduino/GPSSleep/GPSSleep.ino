#include <TinyGPSPlus.h>

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
#define IMU_INT             PA10   // new rev is PA10
#define BATT_MEASURE        PB4  // new rev is PB4

// The TinyGPSPlus object
TinyGPSPlus gps;

void setup()
{
  // initialize pins
  initializeSystem();
  
  // power-on all peripheral
  powerPeripheral(true);

  enableGPS();
  
  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    RX    RX        Fail"));
  Serial.println(F("------------------------------------------------------------------------------------------------"));
}


uint32_t start_time;
uint32_t stop_at_time = 600000; // disable GPS every 10 minute
uint32_t sleep_time = 5000; // sleep for 5 seconds
void loop()
{
  /* print incoming GPS data */
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));


  /* set gps to sleep every stop_at_time ms */
  if( start_time == 0 ){
    start_time = millis();
  }
  if( millis() - start_time > stop_at_time ){

    // disable gps
    disableGPS();
    
    // power-off all peripheral
    powerPeripheral(  false );

    // set device to sleep
    Serial.println("Set device to sleep");
    api.system.sleep.all(sleep_time);

    // wake gps
    enableGPS();
    
    // poweron all peripheral
    powerPeripheral(  true );


    // reset timer
    start_time = millis();
  }
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

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

void powerPeripheral(bool power){
  if( power ){
    
    enableSensorCommunication();
    pinMode(PERIPHERAL_POWER_EN, OUTPUT);
    digitalWrite(PERIPHERAL_POWER_EN, HIGH);
  } else {
    
    disableSensorCommunication();    
    pinMode(PERIPHERAL_POWER_EN, OUTPUT);
    digitalWrite(PERIPHERAL_POWER_EN, LOW);
    delay( 1000 );
    pinMode(PERIPHERAL_POWER_EN, INPUT);  
  }
  delay( 1000 );
}

void initializeSystem(){  
  api.system.restoreDefault();
  Serial.begin(115200);

  // initialize power pin
  pinMode(PERIPHERAL_POWER_EN, INPUT);
    
  // disable unnecessary pin
  pinMode(MAG_INT, INPUT);
  pinMode(IMU_INT, INPUT);
  pinMode(POWER_STATUS, INPUT);
  pinMode(BATT_MEASURE, INPUT);
  pinMode(BATT_MEASURE_EN, INPUT);
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, HIGH);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);
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

void enableGPS(){
  // wake gps, incase it's off
  pinMode(GPS_FORCE_ON, OUTPUT);
  digitalWrite(GPS_FORCE_ON, HIGH);
}

void disableGPS(){  
    // open pin to allow GPS to go to sleep (backup mode)
    pinMode(GPS_FORCE_ON, INPUT);
    // Set GPS to Backup mode
    setGPSToBackupMode();
    delay(1000);
}
