#define BOARD_REVISION (0x02)

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

#if BOARD_REVISION == (0x01)
#define IMU_INT             PB4
#define BATT_MEASURE        PA10
#elif BOARD_REVISION == (0x02)
#define IMU_INT             PA10
#define BATT_MEASURE        PB4
#else
#ifndef BOARD_REVISION
#error "BOARD_REVISION not set"
#endif
#error "invalid BOARD_REVISION value"
#endif

// ADC in 14 Bit mode
#define BATT_MEASURE_TO_BATT  0.00120919472

void setup() {
  api.system.restoreDefault();
  Serial.begin(115200);
  pinMode(POWER_STATUS, INPUT);
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(BATT_MEASURE_EN, OUTPUT);
  digitalWrite(BATT_MEASURE_EN, HIGH);
  pinMode(PERIPHERAL_POWER_EN, OUTPUT);
  digitalWrite(PERIPHERAL_POWER_EN, HIGH);
  analogReadResolution(14);
}

void loop() {
  pinMode(BATT_MEASURE_EN, OUTPUT);
  digitalWrite(BATT_MEASURE_EN, HIGH);
  // set LED pin to output
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);

  // blink led
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, HIGH);
  delay(500);
  digitalWrite(LED0, HIGH);
  digitalWrite(LED1, LOW);
  delay(500);

  // disable LED
  digitalWrite(LED0, HIGH);
  digitalWrite(LED1, HIGH);

  int   batt_measure_adc  = analogRead(BATT_MEASURE);
  float batt_measure_volt = batt_measure_adc * BATT_MEASURE_TO_BATT;
  Serial.printf("Power Status: %d, Battery Voltage ADC: %d, Battery Voltage: %f\n", digitalRead(POWER_STATUS), batt_measure_adc, batt_measure_volt);
  delay(1000);
}
