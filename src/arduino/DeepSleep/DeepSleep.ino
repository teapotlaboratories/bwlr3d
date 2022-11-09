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

void setup()
{
  initializeSystem();
  Serial.println("RAKwireless System Powersave Example");
  Serial.println("------------------------------------------------------");
}

void loop()
{
  Serial.print("The timestamp before sleeping: ");
  Serial.print(millis());
  Serial.println(" ms");
  Serial.println("(Wait 10 seconds or Press any key to wakeup)");
  api.system.sleep.all(600000);
  Serial.print("The timestamp after sleeping: ");
  Serial.print(millis());
  Serial.println(" ms");
}

void initializeSystem(){  
  api.system.restoreDefault();
  Serial.begin(115200);

  // power-off all peripheral
  pinMode(PERIPHERAL_POWER_EN, OUTPUT);
  digitalWrite(PERIPHERAL_POWER_EN, LOW);
  
  // disable sensor communication
  disableSensorCommunication();
  
  // disable unnecessary pin
  pinMode(MAG_INT, INPUT);
  pinMode(IMU_INT, INPUT);
  pinMode(POWER_STATUS, INPUT);
  pinMode(BATT_MEASURE, INPUT);
  pinMode(BATT_MEASURE_EN, OUTPUT);
  digitalWrite(BATT_MEASURE_EN, LOW);
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, HIGH);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);

  pinMode(GPS_FORCE_ON, INPUT); // open pin to allow GPS to go to sleep (backup mode)
    
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
