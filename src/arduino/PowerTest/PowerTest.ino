#include "TeapotBWLR3D.h"
#include "TeapotLoRaWAN.h"

// if COMBINED_LORA_PAYLOAD defined, data will be sent as 1 payload.
#define COMBINED_LORA_PAYLOAD

/* forward-declaration */
void SendCb( int32_t status );

/* LoRaWAN configuration */
const uint8_t kOtaaDevEui[8]  = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x08, 0x34, 0xB3};
const uint8_t kOtaaAppEui[8]  = {0x0E, 0x0D, 0x0D, 0x01, 0x0E, 0x01, 0x02, 0x0E};
const uint8_t kOtaaAppKey[16] = {0x3d, 0xbe, 0x8c, 0xc0, 0x87, 0x77, 0x5c, 0x9e, 0x83, 0xba, 0x7c, 0x5e, 0x9a, 0x88, 0x96, 0x98};
const RAK_LORA_BAND kOtaaBand = RAK_REGION_US915;

/* main application objects */
teapot::connection::Lorawan lora( kOtaaDevEui, sizeof(kOtaaDevEui),
                                  kOtaaAppEui, sizeof(kOtaaAppEui),
                                  kOtaaAppKey, sizeof(kOtaaAppKey),
                                  kOtaaBand,3, 5, nullptr );
teapot::bwlr3d::Application app;

void setup(void) {
  api.system.restoreDefault();  
  Serial.begin(115200);

  app.Initialize();
  
  // blink led to indicate start-up
  app.BlinkLedIndicator( 500 );
  
  // connect to LoRaWAN network
  if( !lora.IsInitialized() ){
    while(true) {
      Serial.println("fail to initialize connection");
      app.BlinkLedIndicator( teapot::bwlr3d::Led::kRed, 4, 200 );   
      delay( 5000 );
    }
  }
  
  class NotifyLorawanNetworkJoin : public teapot::connection::LorawanCallback {
    public:
      NotifyLorawanNetworkJoin() = default;
      void NotifyNetworkJoin(bool joined){
        teapot::bwlr3d::Led led = joined ? teapot::bwlr3d::Led::kGreen : teapot::bwlr3d::Led::kRed;
        app.BlinkLedIndicator( led, 2, 200 );        
      }
  };
  NotifyLorawanNetworkJoin notify_join;
  teapot::connection::ReturnCode ret = lora.Connect( notify_join );
  if( ret != teapot::connection::ReturnCode::kOk ){
    while(true) {
      Serial.printf("fail to connect to gateway. ret: %d\r\n", ret);
      app.BlinkLedIndicator( teapot::bwlr3d::Led::kRed, 5, 200 );   
      delay( 5000 );
    }
  }

  // power-on peripheral
  app.EnablePeripheral( true );
  app.EnableGnss( true );
  app.ResetGnssData();
}

void TestPowerGnssAcquisition();
void TestPowerSensorReadWithGnssOff();
void TestPowerSensorReadAndGnssAcquisition();

void loop()
{
  TestPowerGnssAcquisition();
//  TestPowerSensorReadWithGnssOff();
//  TestPowerSensorReadAndGnssAcquisition();
}

// TEST: Sensor read with GNSS off
void TestPowerSensorReadWithGnssOff() 
{
  Serial.println("start TestPowerSensorReadWithGnssOff()");
  uint32_t temp_period = 10 * 1000;
  teapot::bwlr3d::SensorData  data;
  float battery;
  
  Serial.println("power-on peripheral");
  // power-on peripheral
  app.EnablePeripheral( true );
  app.EnableGnss( false );
  
  /* read sensor and battery voltage */
  {    
    /* read sensor data */
    delay( 50 );
    if( app.ConfigureSensor() != teapot::bwlr3d::ReturnCode::kOk )
    {
      Serial.println("failed to configure sensor");
      return;
    }
    if( app.GetSensorData(data) != teapot::bwlr3d::ReturnCode::kOk )
    {
      Serial.println("failed to read sensor data");
      return; 
    }

    /* read battery voltage */
    battery = app.ReadBatteryVoltage();
  }
  
  /* power-off all sensor */
  app.EnablePeripheral( false );

  /* sleep device */
  Serial.printf("Try sleep %ums..\r\n", temp_period);
  api.system.sleep.all(temp_period);
  Serial.println("Wakeup..");
}

// TEST: GNSS Acquisition
void TestPowerGnssAcquisition() 
{
  Serial.println("start TestPowerGnssAcquisition()");
  uint32_t temp_period = 10 * 1000;
  teapot::bwlr3d::SensorData  data;
  float battery;
  teapot::bwlr3d::ReturnCode gnss_ret = teapot::bwlr3d::ReturnCode::kError;
  
  Serial.println("power-on peripheral");
  // power-on peripheral
  app.EnablePeripheral( true );
  app.EnableGnss( true );
  app.ResetGnssData();
  
  /* read sensor and gnss data */
  {    
    // configure sensor to avoid sensor running on high performance
    delay( 50 );
    if( app.ConfigureSensor() != teapot::bwlr3d::ReturnCode::kOk )
    {
      Serial.println("failed to configure sensor");
      return;
    }
    
    uint32_t start = millis();
    /* read gnss data */
    // process gnss stream
    gnss_ret = app.ProcessGnssStream(5, 1, 120*1000);
    Serial.printf("process gnss stream done. err: %d\r\n", gnss_ret);
    Serial.printf("gnss acqusition time: %d ms\r\n", millis() - start);
    Serial.printf("hdop: %f, sat: %d, lat: %f, long: %f\r\n", 
                  app.GetGnssData().hdop.hdop(), 
                  app.GetGnssData().satellites.value(),
                  app.GetGnssData().location.lat(),
                  app.GetGnssData().location.lng());
  }
  
  /* power-off all sensor */
  app.EnableGnss( false );
  app.EnablePeripheral( false );

  /* sleep device */
  Serial.printf("Try sleep %ums..\r\n", temp_period);
  api.system.sleep.all(temp_period);
  Serial.println("Wakeup..");
}

// TEST: Sensor Read and GNSS Acquisition
void TestPowerSensorReadAndGnssAcquisition() 
{
  Serial.println("start TestPowerSensorReadAndGnssAcquisition()");
  uint32_t temp_period = 10 * 1000;
  teapot::bwlr3d::SensorData  data;
  float battery;
  teapot::bwlr3d::ReturnCode gnss_ret = teapot::bwlr3d::ReturnCode::kError;
  
  Serial.println("power-on peripheral");
  // power-on peripheral
  app.EnablePeripheral( true );
  app.EnableGnss( true );
  app.ResetGnssData();
  uint32_t start = millis();
  
  /* read sensor and gnss data */
  {    
    delay( 50 );
    if( app.ConfigureSensor() != teapot::bwlr3d::ReturnCode::kOk )
    {
      Serial.println("failed to configure sensor");
      return;
    }
    if( app.GetSensorData(data) != teapot::bwlr3d::ReturnCode::kOk )
    {
      Serial.println("failed to read sensor data");
      return; 
    }

    /* read battery voltage */
    battery = app.ReadBatteryVoltage();

    /* read gnss data */
    gnss_ret = app.ProcessGnssStream(5, 1, 120*1000);
    Serial.printf("process gnss stream done. err: %d\r\n", gnss_ret);
    Serial.printf("gnss acqusition time: %d ms\r\n", millis() - start);
    Serial.printf("hdop: %f, sat: %d, lat: %f, long: %f\r\n", 
                  app.GetGnssData().hdop.hdop(), 
                  app.GetGnssData().satellites.value(),
                  app.GetGnssData().location.lat(),
                  app.GetGnssData().location.lng());
  }
  
  /* power-off all sensor */
  app.EnableGnss( false );
  app.EnablePeripheral( false );

  /* sleep device */
  Serial.printf("Try sleep %ums..\r\n", temp_period);
  api.system.sleep.all(temp_period);
  Serial.println("Wakeup..");
}
