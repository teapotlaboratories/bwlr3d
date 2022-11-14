#include "TeapotBWLR3D.h"
#include "TeapotLoRaWAN.h"

// if COMBINED_LORA_PAYLOAD defined, 
// data will be sent as 1 payload.
#define COMBINED_LORA_PAYLOAD

/* forward-declaration */
void SendCb( int32_t status );

/* LoRaWAN configuration */
const uint8_t kOtaaDevEui[8]  = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x08, 0x34, 0xB3};
const uint8_t kOtaaAppEui[8]  = {0x0E, 0x0D, 0x0D, 0x01, 0x0E, 0x01, 0x02, 0x0E};
const uint8_t kOtaaAppKey[16] = {0x3d, 0xbe, 0x8c, 0xc0, 0x87, 0x77, 0x5c, 0x9e, 0x83, 0xba, 0x7c, 0x5e, 0x9a, 0x88, 0x96, 0x98};
const RAK_LORA_BAND kOtaaBand = RAK_REGION_US915;
//const uint32_t kOtaaPeriodMs      = (10 * 60 *1000); // 10 minute 
//const uint32_t kOtaaPeriodMs      = (10 * 1000); // 10 seconds
//const uint32_t kOtaaPeriodMs      = (30 * 1000); // 30 seconds
const uint32_t kOtaaPeriodMs      = (300 * 1000); // 5 minute
//const uint32_t kGnssPeriodS      = ( 24 * 3600 ); // 24 hour
//const uint32_t kGnssPeriodS      = ( 15 * 60 ); // 15 minute
const uint32_t kGnssPeriodS      = ( 5 * 60 ); // 15 minute

/* main application objects */
teapot::connection::Lorawan lora( kOtaaDevEui, sizeof(kOtaaDevEui),
                                  kOtaaAppEui, sizeof(kOtaaAppEui),
                                  kOtaaAppKey, sizeof(kOtaaAppKey),
                                  kOtaaBand,3, 3, nullptr );
teapot::bwlr3d::Application app;
uint32_t                    gnss_process_timer_s = 0; // timer for getting and sending gnss data
bool                        first_fix = false;
//
///* manage uplink process */
//int32_t   send_status;
//bool      send_done;
//
//void ResetSendWait()
//{
//  send_done = false;
//}
//
///** Upstream Transmit Callback
// *
// * A callback once an uplink process is done
// *
// * @param status status of the uplink process.
// */
//void SendCb( int32_t status )
//{
//  send_status = status;
//  send_done = true;
//}
//
///** Busy wait with timeout for Uplink process
// *
// * Callback once an uplink process is done
// *
// * @param timeout_ms timeout of the busy wait
// * @param ms delay for the busy wait loop
// * @return `kOk` if send is successful, `kTimeout` if timeout, 
// *         `kFailSendPayload` if fail to send
// * 
// */
//teapot::connection::ReturnCode WaitForUplinkProcess(uint32_t timeout_ms, uint32_t ms = 100)
//{
//  uint32_t start = millis();
//  bool is_timeout = false;
//  while( !(send_done) && !is_timeout )
//  {
//    delay(ms);
//    is_timeout = timeout_ms < (millis()-start);
//  }
//
//  if( send_status )
//  {
//    return teapot::connection::ReturnCode::kOk;      
//  }
//  else if( is_timeout )
//  {
//    return teapot::connection::ReturnCode::kTimeout;
//  }
//
//  return teapot::connection::ReturnCode::kFailSendPayload;
//}

void setup(void) {
  api.system.restoreDefault();  
  Serial.begin(115200);

  app.Initialize();
  
  // blink led to indicate start-up
  app.BlinkLedIndicator( 500 );
  
  // connect to LoRaWAN network
  if( !lora.IsInitialized() ){
//    Serial.println("fail to initialize LoRaWAN connection");
    while(true) {
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
  teapot::connection::ReturnCode ret = lora.Connect( &notify_join );
  if( ret != teapot::connection::ReturnCode::kOk ){
//    Serial.printf("fail to connect to LoRaWAN gateway. Ret: %d\r\n", ret);
    while(true) {
      app.BlinkLedIndicator( teapot::bwlr3d::Led::kRed, 5, 200 );   
      delay( 5000 );
    }
  }

  // power-on peripheral
  app.EnablePeripheral( true );
  app.EnableGnss( true );
  app.ResetGnssData();
}

void loop() {  
  /* prepare packet */
  uint8_t payload[128];
  size_t  out_payload_size = 0;  
  float                       battery;
  teapot::bwlr3d::SensorData  data;

  /* gnss processing timer variables */
  uint32_t start = millis();
  teapot::bwlr3d::ReturnCode gnss_ret = teapot::bwlr3d::ReturnCode::kError;
  
  if( !first_fix )
  {
    app.ResetGnssData();
    Serial.println("waiting for first gnss fix");

    // Wait until hdop <= 5 and sat >= 1 or timeout in 30 seconds
    teapot::bwlr3d::ReturnCode ret = app.ProcessGnssStream(5, 1, 30*1000);
    if( ret == teapot::bwlr3d::ReturnCode::kOk )
    {
        first_fix = true;
    }
    Serial.printf("process gnss stream done. err: %d\r\n", ret);
    
    /* blink to indicate success or fail of first fix */
    // blink 3 times with 500ms delay
    teapot::bwlr3d::Led led = first_fix ? teapot::bwlr3d::Led::kGreen : teapot::bwlr3d::Led::kRed;
    app.BlinkLedIndicator( led, 3, 200 );

    // force to send gnss data on next loop
    gnss_process_timer_s = kGnssPeriodS;
        
    return;
  }

  /* read sensor and gnss data */
  {    
    /* read sensor data */
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
    battery = app.ReadBatteryVoltage();
    
    /* read gnss data */
    // simple inaccurate timer for processing gnss data
    if( gnss_process_timer_s >= kGnssPeriodS )
    { 
      // reset timer
      gnss_process_timer_s = 0;
      
      // process gnss stream
      gnss_ret = app.ProcessGnssStream(5, 1, 30*1000);
      Serial.printf("process gnss stream done. err: %d\r\n", gnss_ret);
    }
  }
  
  /* power-off all sensor */
  app.EnableGnss( false );
  app.EnablePeripheral( false );

#ifdef COMBINED_LORA_PAYLOAD
  uint32_t payload_total_size = 0;
  /* copy environmental packet */
  teapot::bwlr3d::payload::Environmental env( battery,
                                              data.temperature, 
                                              data.pressure, 
                                              data.humidity, 
                                              data.gas_resistance, 
                                              data.lux );
  out_payload_size = env.GetAsBytes(payload, sizeof(payload));
  payload_total_size += out_payload_size;

  /* copy imu packet */
  teapot::bwlr3d::payload::Imu imu( { data.mag.magnetic.x, 
                                      data.mag.magnetic.y, 
                                      data.mag.magnetic.z }, 
                                    { data.accel.acceleration.x, 
                                      data.accel.acceleration.y, 
                                      data.accel.acceleration.z }, 
                                    { data.gyro.gyro.x, 
                                      data.gyro.gyro.y, 
                                      data.gyro.gyro.z } );
  out_payload_size = env.GetAsBytes(payload + payload_total_size, sizeof(payload) - payload_total_size);
  payload_total_size += out_payload_size;

  /* send gnss packet */
  if( gnss_ret == teapot::bwlr3d::ReturnCode::kOk )
  {
    teapot::bwlr3d::payload::Gnss gnss( app.GetGnssData().satellites.value(),
                                        app.GetGnssData().hdop.hdop(), 
                                        app.GetGnssData().location.lat(),
                                        app.GetGnssData().location.lng(),
                                        app.GetGnssData().altitude.meters() );
    out_payload_size = gnss.GetAsBytes(payload + payload_total_size, sizeof(payload) - payload_total_size);
    payload_total_size += out_payload_size;

    // reset value
    gnss_ret = teapot::bwlr3d::ReturnCode::kError;
  }
  out_payload_size = payload_total_size;

  /* send payload and wait for process to complete */
  Serial.printf("sending environmental, imu, and gnss data. size: %d\r\n", out_payload_size);
//  ResetSendWait();
  lora.Send(payload, out_payload_size);
//  teapot::connection::ReturnCode wait_ret = WaitForUplinkProcess(60 * 1000);
//  Serial.printf("uplink status: %d\r\n", wait_ret);
//  delay(5000);
  
#else
  /* send environmental packet */
  Serial.println("sending environmental data");
  ResetSendWait();
  teapot::bwlr3d::payload::Environmental env( data.temperature, 
                                              data.pressure, 
                                              data.humidity, 
                                              data.gas_resistance, 
                                              data.lux );
  out_payload_size = env.GetAsBytes(payload, sizeof(payload));
  lora.Send(payload, out_payload_size);
  WaitForUplinkProcess(60 * 1000);

  /* send imu packet */
  Serial.println("sending imu data");
  ResetSendWait();
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
  lora.Send(payload, out_payload_size); 
  WaitForUplinkProcess(60 * 1000);

  /* send gnss packet */
  if( gnss_ret == teapot::bwlr3d::ReturnCode::kOk )
  {
    Serial.println("sending gnss data");
    ResetSendWait();
    teapot::bwlr3d::payload::Gnss gnss( app.GetGnssData().satellites.value(),
                                        app.GetGnssData().hdop.hdop(), 
                                        app.GetGnssData().location.lat(),
                                        app.GetGnssData().location.lng(),
                                        app.GetGnssData().altitude.meters() );
    out_payload_size = gnss.GetAsBytes(payload, sizeof(payload));
    lora.Send(payload, out_payload_size);
    WaitForUplinkProcess(60 * 1000);

    // reset value
    gnss_ret = teapot::bwlr3d::ReturnCode::kError;
  }
#endif

  /* sleep device */
  Serial.printf("Try sleep %ums..\r\n", kOtaaPeriodMs);
  api.system.sleep.all(kOtaaPeriodMs);
  Serial.println("Wakeup..");

  /* power-on all sensor */
  app.EnablePeripheral( true );
  app.EnableGnss( true );
  app.ResetGnssData();

  // update gnss_process_timer
  gnss_process_timer_s += abs( (millis()/1000) - (start/1000) );
}
