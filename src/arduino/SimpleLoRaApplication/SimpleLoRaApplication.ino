#include "TeapotBWLR3D.h"
#include "TeapotLoRaWAN.h"

const uint8_t kOtaaDevEui[8]  = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x08, 0x34, 0xB3};
const uint8_t kOtaaAppEui[8]  = {0x0E, 0x0D, 0x0D, 0x01, 0x0E, 0x01, 0x02, 0x0E};
const uint8_t kOtaaAppKey[16] = {0x3d, 0xbe, 0x8c, 0xc0, 0x87, 0x77, 0x5c, 0x9e, 0x83, 0xba, 0x7c, 0x5e, 0x9a, 0x88, 0x96, 0x98};
const RAK_LORA_BAND kOtaaBand = RAK_REGION_US915;
//const uint32_t kOtaaPeriod      = (600*1000); // 10 minute 
//const uint32_t kOtaaPeriod      = (10*1000); // 10 seconds
const uint32_t kOtaaPeriod      = (30*1000); // 30 seconds

teapot::connection::Lorawan lora( kOtaaDevEui, sizeof(kOtaaDevEui),
                                  kOtaaAppEui, sizeof(kOtaaAppEui),
                                  kOtaaAppKey, sizeof(kOtaaAppKey),
                                  kOtaaBand );
teapot::bwlr3d::Application app;
teapot::bwlr3d::SensorData data;
  
void setup(void) {
  api.system.restoreDefault();  
  Serial.begin(115200);

  app.Initialize();
  
  // blink led to indicate start-up
  BlinkLedIndicator();
  
  // connect to LoRaWAN network
  if( !lora.IsInitialized() ){
    Serial.println("Fail to initialize LoRaWAN connection");
    while(true) delay( 1000 );
  }
  
  teapot::connection::ReturnCode ret = lora.Connect();
  if( ret != teapot::connection::ReturnCode::kOk ){
    Serial.printf("Fail to connect to LoRaWAN gateway. Ret: %d\r\n", ret);
    while(true) delay( 1000 );
  }

  // power-on peripheral
  app.EnablePeripheral( true );
  app.EnableGnss( true );
  app.ResetGnssData();
}
bool first_fix = false;
void loop() {
//   TODO: finalize example code
  /* sleep device */
  Serial.printf("Try sleep %ums..\r\n", kOtaaPeriod);
  app.EnableGnss( false );
  app.EnablePeripheral( false );
  api.system.sleep.all(kOtaaPeriod);
  Serial.println("Wakeup..");
  
  /* power-on all sensor and get environmental sensor data */
  app.EnablePeripheral( true );
  app.EnableGnss( true );
  app.ResetGnssData();
  delay( 500 );
  if( app.ConfigureSensor() != teapot::bwlr3d::ReturnCode::kOk )
  {
    Serial.println("failed to configure sensor");
    return;
  }
  delay( 500 );
  if( app.GetSensorData(data) != teapot::bwlr3d::ReturnCode::kOk )
  {
    Serial.println("failed to read sensor data");
    return; 
  }

  teapot::bwlr3d::Sensor test = teapot::bwlr3d::Sensor::kVeml7700 | teapot::bwlr3d::Sensor::kLis3mdl;

  // Process GNSS Stream
//  app.ProcessGnssStream(5,0,60000);

  /* prepare packet */
  uint8_t payload[128];
  size_t out_payload_size = 0;
  /* send environmental packet */
  Serial.println("sending environmental data");
  teapot::bwlr3d::payload::Environmental env( data.temperature, 
                                              data.pressure, 
                                              data.humidity, 
                                              data.gas_resistance, 
                                              data.lux );
  out_payload_size = env.GetAsBytes(payload, sizeof(payload));
  Serial.printf("sending payload. size: %d\r\n", out_payload_size);
  lora.Send(payload, out_payload_size);

  /* send imu packet */
  Serial.println("sending imu data");
  teapot::bwlr3d::payload::Imu imu( { data.mag.magnetic.x, 
                                      data.mag.magnetic.y, 
                                      data.mag.magnetic.z }, 
                                    { data.accel.acceleration.x, 
                                      data.accel.acceleration.y, 
                                      data.accel.acceleration.z }, 
                                    { data.gyro.gyro.x, 
                                      data.gyro.gyro.y, 
                                      data.gyro.gyro.z } );
  out_payload_size = env.GetAsBytes(payload, sizeof(payload));
  Serial.printf("sending payload. size: %d\r\n", out_payload_size);
  lora.Send(payload, out_payload_size);
  

///* for gps testing */
//  const uint32_t temp_otaa_period = 10000;
//  if( !first_fix )
//  {    
//    app.ResetGnssData();
//    Serial.println("Waiting for First GPS Fix");
//    unsigned long start = millis();
//    switch( app.ProcessGnssStream(5,0,60000) )
//    {
//      case teapot::bwlr3d::ReturnCode::kOk:
//      {
//        first_fix = true;
//        Serial.println("Got GNSS Location");
//        break;
//      }
//      case teapot::bwlr3d::ReturnCode::kTimeout:
//      {
//        Serial.println("GNSS Timeout");
//        break;
//      }
//      default:
//      {
//        Serial.println("Unknown error");
//        break;
//      }
//    }
//    
//    Serial.printf("Time to fix: %d ms\n",millis() - start);
//    Serial.print("Time: "); print_date_time(app.GetGnssData().date, app.GetGnssData().time);
//    Serial.print(" | HDOP: ");print_float(app.GetGnssData().hdop.hdop(), app.GetGnssData().hdop.isValid(), 6, 1);
//    Serial.print(" | Satellites: "); print_int(app.GetGnssData().satellites.value(), app.GetGnssData().satellites.isValid(), 5); Serial.println();
//  
//    Serial.printf("Try sleep %ums..", temp_otaa_period);
//    api.system.sleep.all(temp_otaa_period);
//    Serial.println("Wakeup..");
//    return;
//  }
//  app.EnablePeripheral( true );
//  app.EnableGnss( true );
//  app.ResetGnssData();
//
//  unsigned long start = millis();
//  switch( app.ProcessGnssStream(5,0,60000) )
//  {
//    case teapot::bwlr3d::ReturnCode::kOk:
//    {
//      Serial.println("Got GNSS Location");
//      break;
//    }
//    case teapot::bwlr3d::ReturnCode::kTimeout:
//    {
//      Serial.println("GNSS Timeout");
//      break;
//    }
//    default:
//    {
//      Serial.println("Unknown error");
//      break;
//    }
//  }
//  Serial.printf("Time to fix: %d ms\n",millis() - start);
//  Serial.print("Time: "); print_date_time(app.GetGnssData().date, app.GetGnssData().time);
//  Serial.print(" | HDOP: ");print_float(app.GetGnssData().hdop.hdop(), app.GetGnssData().hdop.isValid(), 6, 1);
//  Serial.print(" | Satellites: "); print_int(app.GetGnssData().satellites.value(), app.GetGnssData().satellites.isValid(), 5); Serial.println();
//
//  Serial.printf("Try sleep %ums..", temp_otaa_period);
//
//  app.EnableGnss( false );
//  app.EnablePeripheral( false );
//  api.system.sleep.all(temp_otaa_period);
//  Serial.println("Wakeup..");
}

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

static void print_float(float val, bool valid, int len, int prec)
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
}

static void print_int(unsigned long val, bool valid, int len)
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
}

static void print_date_time(TinyGPSDate &d, TinyGPSTime &t)
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

  print_int(d.age(), d.isValid(), 5);
}
