#include "TeapotBWLR3D.h"

teapot::bwlr3d::Application app;
teapot::bwlr3d::SensorData data;

void BlinkLedIndicator()
{  
  /* blink led indicator at start-up */
  app.EnableLed(teapot::bwlr3d::Led::kGreen, true);
  delay(500);
  app.EnableLed(teapot::bwlr3d::Led::kGreen, false);
  app.EnableLed(teapot::bwlr3d::Led::kRed, true);
  delay(500);
  app.EnableLed(teapot::bwlr3d::Led::kRed, false);
} 
  
void setup(void) {
  api.system.restoreDefault();  
  Serial.begin(115200);

  app.Initialize();
  
  // blink led to indicate start-up
  BlinkLedIndicator();
  
  // power-on peripheral
  app.EnablePeripheral( true );
  app.EnableGnss( true );
  app.ResetGnssData();
  delay( 100 ); // wait some time for sensor to boot
  teapot::bwlr3d::ReturnCode ret = app.ConfigureSensor();
  if( ret != teapot::bwlr3d::ReturnCode::kOk )
  {
    Serial.printf("failed to configure sensor. ret: %d\n", ret);
    while( true ) delay( 1000 );
  }
  
  Serial.println("Sensor initialized");
}

void loop() {
  if( app.GetSensorData(data) == teapot::bwlr3d::ReturnCode::kOk )
  {
    PrintSensorData();
    Serial.printf("\n\n\n\n\n");
  }
  else
  {
    Serial.println("failed to read sensor data");
  }
  delay( 500 );
}

void PrintSensorData(){
  Serial.println("================ VEML7700 Sensor ================");
  Serial.print("lux: "); Serial.println(data.lux);

  Serial.println("================ LIS3MDL Sensor ================");
  /* Display the results (magnetic field is measured in uTesla) */
  Serial.print("X: "); Serial.print(data.mag.magnetic.x);
  Serial.print(" \tY: "); Serial.print(data.mag.magnetic.y); 
  Serial.print(" \tZ: "); Serial.print(data.mag.magnetic.z); 
  Serial.println(" uTesla ");

  Serial.println("================ LSM6DS0X Sensor ================");
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("Accel X: ");
  Serial.print(data.accel.acceleration.x);
  Serial.print(" \tY: ");
  Serial.print(data.accel.acceleration.y);
  Serial.print(" \tZ: ");
  Serial.print(data.accel.acceleration.z);
  Serial.println(" m/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("Gyro X: ");
  Serial.print(data.gyro.gyro.x);
  Serial.print(" \tY: ");
  Serial.print(data.gyro.gyro.y);
  Serial.print(" \tZ: ");
  Serial.print(data.gyro.gyro.z);
  Serial.println(" radians/s ");

  Serial.println("================ BME68x Sensor ================");
  Serial.print("Temperature = ");
  Serial.print(data.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(data.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(data.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(data.gas_resistance / 1000.0);
  Serial.println(" KOhms");
}
