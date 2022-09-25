#define LED0                PA0
#define LED1                PA1
#define PERIPHERAL_POWER_EN PB5
#define BATT_MEASURE_EN     PB2
#define POWER_STATUS        PB12
#define BATT_MEASURE        PA15

void setup() {
  api.system.restoreDefault();
  Serial.begin(115200);
  pinMode(POWER_STATUS, INPUT);
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(BATT_MEASURE_EN, INPUT);
}

void loop() {
  // power-on all peripheral
  pinMode(PERIPHERAL_POWER_EN, OUTPUT);
  digitalWrite(PERIPHERAL_POWER_EN, HIGH);

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

  // disable all power
  pinMode(LED0, INPUT);
  pinMode(LED1, INPUT);
  pinMode(PERIPHERAL_POWER_EN, INPUT);

  Serial.println("Sleep for 120 seconds");
  api.system.sleep.all(120000);
  
  int batt_measure_adc = analogRead(BATT_MEASURE);
  Serial.printf("Power Status: %d, Battery Voltage: %d\n", digitalRead(POWER_STATUS), analogRead(BATT_MEASURE));
  delay(100);
}
