#include <TinyGPSPlus.h>

#define PERIPHERAL_POWER_EN PB5
#define GPS_FORCE_ON PA15

// The TinyGPSPlus object
TinyGPSPlus gps;

void setup()
{
  
  api.system.restoreDefault();

  // power-on all peripheral
  pinMode(PERIPHERAL_POWER_EN, OUTPUT);
  digitalWrite(PERIPHERAL_POWER_EN, HIGH);

  // wake gps, incase it's off
  pinMode(GPS_FORCE_ON, OUTPUT);
  digitalWrite(GPS_FORCE_ON, HIGH);
  
  Serial.begin(115200);
  Serial1.begin(9600);

  Serial.println();
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    RX    RX        Fail"));
  Serial.println(F("------------------------------------------------------------------------------------------------"));
}


uint32_t start_time;
uint32_t stop_at_time = 60000; // disable GPS every 1 minute
uint32_t sleep_time = 60000; // sleep for 1 minute
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
    
    // open pin to allow GPS to go to sleep (backup mode)
    pinMode(GPS_FORCE_ON, INPUT);
    // Set GPS to Backup mode
    setGPSToBackupMode();
    Serial.println("Set device to sleep");
    api.system.sleep.all(sleep_time);

    // wake gps
    pinMode(GPS_FORCE_ON, OUTPUT);
    digitalWrite(GPS_FORCE_ON, HIGH);

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
