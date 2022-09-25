#define LED0                PA0
#define LED1                PA1
#define PERIPHERAL_POWER_EN           PB5
#define BATT_MEAS_EN                  PB2

void setup()
{
  api.system.restoreDefault();
  Serial.begin(115200);
  Serial.println("RAKwireless System Powersave Example");
  Serial.println("------------------------------------------------------");

  // power-off all peripheral
  pinMode(PERIPHERAL_POWER_EN, INPUT);

  // do not drive pin to save power
  pinMode(BATT_MEAS_EN, INPUT);
  pinMode(LED0, INPUT);
  pinMode(LED1, INPUT);

}

void loop()
{
  Serial.print("The timestamp before sleeping: ");
  Serial.print(millis());
  Serial.println(" ms");
  Serial.println("(Wait 10 seconds or Press any key to wakeup)");
  api.system.sleep.all(10000);
  Serial.print("The timestamp after sleeping: ");
  Serial.print(millis());
  Serial.println(" ms");
}
