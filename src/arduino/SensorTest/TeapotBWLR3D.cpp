#include "TeapotBWLR3D.h"
#include <CRC.h>
#include <CRC16.h>

/* application implementation */
namespace teapot {
namespace bwlr3d {

  uint32_t operator|( const Sensor m, const Sensor n ){
    return ( static_cast<uint32_t>(m) | static_cast<uint32_t>(n) ) ;
  }
  uint32_t operator&( const uint32_t m, const Sensor n ){
    return ( m & static_cast<uint32_t>(n) ) ;
  }
  
  Application::Application(){}
  void Application::Initialize()
  {    
    // disable unnecessary pin
    pinMode(MAG_INT, INPUT);
    pinMode(IMU_INT, INPUT);
    pinMode(POWER_STATUS, INPUT);

    // setup battery read measure
    analogReadResolution(14);
    pinMode(BATT_MEASURE, INPUT);
    pinMode(BATT_MEASURE_EN, OUTPUT);
    digitalWrite(BATT_MEASURE_EN, LOW);
    
    // setup led pin
    pinMode( RED_LED, OUTPUT );
    EnableLed( Led::kRed, false );
    pinMode( GREEN_LED, OUTPUT );
    EnableLed( Led::kGreen, false );
    
    // initialize GPS
    EnablePeripheral(true);
    delay( 1000 );
    EnableGnss( false );
    delay( 1000 );
    
    // power-off all peripheral
    EnablePeripheral(false);
    delay( 1000 );
  }

  int Application::ReadBatteryAdc()
  {
    // TODO: investigate. it seems pin always disabled at every loop.
    //       always set to output at every ADC read.
    pinMode(BATT_MEASURE_EN, OUTPUT);
    digitalWrite(BATT_MEASURE_EN, HIGH);
    const int battery_adc = analogRead( BATT_MEASURE );
    digitalWrite(BATT_MEASURE_EN, LOW);
    return battery_adc;
  }

  float Application::ReadBatteryVoltage()
  {
    return ((float) ReadBatteryAdc()) * ADC_TO_BATTERY;
  }

  bool Application::ReadPowerStatus()
  {
    // TODO: investigate. it seems pin always disabled at every loop.
    //       always set to input at every digital read.
    pinMode(POWER_STATUS, INPUT);
    return digitalRead(POWER_STATUS);
  }
  
  void Application::ResetGnssData()
  {
    this->l86 = TinyGPSPlus();
  }
  
  ReturnCode Application::EnableGnss( bool enable )
  {
    if( !this->enable_peripheral )
    {
      return ReturnCode::kPeripheralOff;
    }
    
    if( enable )
    {
      // wake gps, incase it's off
      pinMode(GPS_FORCE_ON, OUTPUT);
      digitalWrite(GPS_FORCE_ON, HIGH);
    }
    else 
    {
      // open pin to allow GPS to go to sleep (backup mode)
      pinMode(GPS_FORCE_ON, INPUT);
      
      // prepare command
      unsigned char backup_mode_command[15];
      String base_backup_mode_command = ("$PMTK225,4*2F"); 
      const char tr[] = {0xD, 0xA};
      base_backup_mode_command.getBytes(backup_mode_command, sizeof(backup_mode_command));
      memcpy(backup_mode_command+13, tr, 2); 

      // send sleep command to gps
      Serial1.write(backup_mode_command, sizeof(backup_mode_command));

      // clear incoming uart data
      Serial1.flush();
      while (Serial1.available()) Serial1.read();
    }
    
    return ReturnCode::kOk;
  }
  
  ReturnCode Application::EnableLed( Led led, bool enable )
  {
    switch( led ){
      case Led::kRed:
      {
        digitalWrite( RED_LED, (enable ? LOW : HIGH) );
      }
      case Led::kGreen:
      {
        digitalWrite( GREEN_LED, (enable ? LOW : HIGH) );
      }
      default:
      {
        return ReturnCode::kInvalidLed;
      }
    }
    return ReturnCode::kOk;
  }
  
  void Application::EnablePeripheral(bool enable)
  {
    this->enable_peripheral = enable;
    if( enable )
    {  
      // enable GPS serial
      Serial1.begin(9600);
      
      // enable sensor I2C bus
      Wire.begin();

      // power-on 3.3v rail to all peripheral
      pinMode(PERIPHERAL_POWER_EN, OUTPUT);
      digitalWrite(PERIPHERAL_POWER_EN, HIGH);
    } 
    else 
    {
      // disable GPS serial
      Serial1.end();
      pinMode(GPS_TXD, INPUT);
      pinMode(GPS_RXD, INPUT);
    
      // disable I2C
      Wire.end();
      pinMode(I2C_SDA, INPUT);
      pinMode(I2C_SCL, INPUT);

      // power-off 3.3v rail to all peripheral
      pinMode(PERIPHERAL_POWER_EN, OUTPUT);
      digitalWrite(PERIPHERAL_POWER_EN, LOW);
    }
  }
  
  void Application::SetupLis3mdl()
  {
    this->lis3mdl.setPerformanceMode(LIS3MDL_LOWPOWERMODE);
    this->lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    this->lis3mdl.setDataRate(LIS3MDL_DATARATE_1_25_HZ);    
    this->lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  }
  
  void Application::SetupLsm6dsox()
  {
    this->lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    this->lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
    this->lsm6dsox.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    this->lsm6dsox.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  }
  
  void Application::SetupBme68x()
  {  
    // Set up oversampling and filter initialization
    this->bme68x.setOversampling(TemperatureSensor, Oversample16);  // Use enumerated type values
    this->bme68x.setOversampling(HumiditySensor, Oversample16);     // Use enumerated type values
    this->bme68x.setOversampling(PressureSensor, Oversample16);     // Use enumerated type values
    this->bme68x.setIIRFilter(IIR4);  // Use enumerated type values
    this->bme68x.setGas(320, 150);  // 320c for 150 milliseconds
  }
 
  ReturnCode Application::ConfigureSensor()
  {
    
    /* configure LIS3MDL */
    if (! this->lis3mdl.begin_I2C(0x1E, &Wire)) {
      return ReturnCode::kLis3mdlNotFound;
    }
    SetupLis3mdl();
    
    /* configure LSM6DS0X */
    if (!this->lsm6dsox.begin_I2C()) {
      return ReturnCode::kLsm6dsoxNotFound;
    }
    SetupLsm6dsox();
  
    /* configure BME68x */
    if ( !(this->bme68x.begin()) ) {
      return ReturnCode::kBme68xNotFound;
    }
    SetupBme68x();
  
    /* configure VEML7700 */
    if (!this->veml7700.begin()) {
      return ReturnCode::kVeml7700NotFound;
    }
    return ReturnCode::kOk;
  }
  
  ReturnCode Application::GetSensorData( SensorData& data, uint32_t sensor )
  {    
    if( !this->enable_peripheral )
    {
      return ReturnCode::kPeripheralOff;
    }

    /* Read LIS3MDL sensor */
    if( sensor & Sensor::kLis3mdl )
    {   
      this->lis3mdl.getEvent(&data.mag);
    }
    
    /* Read LSM6DS0X sensor */
    if( sensor & Sensor::kLsm6dsox )
    {   
      sensors_event_t unused_temp;
      this->lsm6dsox.getEvent(&data.accel, &data.gyro, &unused_temp);
    }
    
    /* Read VEML7700 sensor */
    if( sensor & Sensor::kVeml7700 )
    {
      data.lux = this->veml7700.readLux(VEML_LUX_AUTO);
    }
    
    /* Read BME68x sensor */
    if( sensor & Sensor::kBme68x )
    {
      int32_t temp = 0, humidity = 0, pressure = 0, gas = 0;
      
      // TODO: improve BME68x reading. Currently first read always fails.;
      for( int i = 0; i < 2; i++)
      {
        this->bme68x.getSensorData(temp, humidity, pressure, gas, true);
      }
      
      data.temperature      = static_cast<float>(temp) / 100;
      data.pressure         = static_cast<float>(pressure) / 100;
      data.humidity         = static_cast<float>(humidity) / 1000;
      data.gas_resistance   = static_cast<float>(gas) / 100;  
    }
    return ReturnCode::kOk;
  }

  ReturnCode Application::GetSensorData( SensorData& data, Sensor sensor )
  {
    return GetSensorData( data, static_cast<uint32_t>( sensor ) );
  }
  
  ReturnCode Application::ProcessGnssStream(float hdop, uint32_t sat, unsigned long ms)
  {
    unsigned long start = millis();
    do 
    {
      while (Serial1.available()) this->l86.encode(Serial1.read());
      if( l86.hdop.hdop() != 0 && l86.hdop.hdop() <= hdop && l86.hdop.isValid() &&
          l86.satellites.value() != 0 && l86.satellites.value() >= sat && l86.satellites.isValid() )
      {
        return ReturnCode::kOk; 
      }
    } while (millis() - start < ms);
    return ReturnCode::kTimeout;
  }

  TinyGPSPlus& Application::GetGnssData()
  {
    return this->l86; 
  }

  void Application::BlinkLedIndicator( uint32_t ms )
  {  
    EnableLed(Led::kGreen, true);
    EnableLed(Led::kRed, true);
    delay(ms);
    EnableLed(Led::kGreen, false);
    EnableLed(Led::kRed, false);
  } 
  
  void Application::BlinkLedIndicator(Led led, uint32_t ms)
  {  
    EnableLed( led, true );
    delay(ms);
    EnableLed( led, false );
  } 
  
  void Application::BlinkLedIndicator(Led led, int total_blink, uint32_t ms)
  {
    for( int i = 0; i < total_blink; i++ )
    {      
      EnableLed( led, true );
      delay(ms);
      EnableLed( led, false );
      delay(ms);
    }
  }
  
} // namespace bwlr3d 
} // namespace teapot

/* lora payload implementation */
namespace teapot{
namespace bwlr3d {
namespace payload {
  /* environmental payload implementation*/
  Environmental::Environmental( float battery,
                                float temperature,
                                float pressure,
                                float humidity,
                                float gas_resistance,
                                float lux )
  {
    // set data
    this->data.header.version = kVersion;
    this->data.header.type = static_cast<uint32_t>(this->type);
    this->data.battery = battery;
    this->data.temperature = temperature;
    this->data.pressure = pressure;
    this->data.humidity = humidity;
    this->data.gas_resistance = gas_resistance;
    this->data.lux = lux;

    // calculate CRC16
    size_t frame_size = sizeof(this->data);
    size_t payload_wo_crc16_size = frame_size - sizeof(this->data.trailer.checksum);
    this->data.trailer.checksum = crc16_CCITT(reinterpret_cast<uint8_t*>( &(this->data) ), payload_wo_crc16_size);  
  }
  
  size_t Environmental::GetAsBytes(uint8_t* data, size_t size)
  {
    if( data == nullptr || size < sizeof(this->data) )
    {
      return 0;
    }
    
    memcpy( data, &(this->data), sizeof(this->data) );
    return( sizeof(this->data) );
  }

  /* imu payload implementation */
  Imu::Imu( const Vector& mag, const Vector& accel, const Vector& gyro  )
  {
    // set data
    this->data.header.version = kVersion;
    this->data.header.type = static_cast<uint32_t>(this->type);
    this->data.magnetic = mag;
    this->data.acceleration = accel;
    this->data.gyro = gyro;

    // calculate CRC16
    size_t frame_size = sizeof(this->data);
    size_t payload_wo_crc16_size = frame_size - sizeof(this->data.trailer.checksum);
    this->data.trailer.checksum = crc16_CCITT(reinterpret_cast<uint8_t*>( &(this->data) ), payload_wo_crc16_size);
  }
  
  size_t Imu::GetAsBytes(uint8_t* data, size_t size)
  {
    if( data == nullptr || size < sizeof(this->data) )
    {
      return 0;
    }
    
    memcpy( data, &(this->data), sizeof(this->data) );
    return( sizeof(this->data) );
  }

  /* gnss payload implementation */
  Gnss::Gnss( 
        uint16_t satellite,
        float hdop,
        double latitude,
        double longitude,
        double altitude
  )
  {
    // set data
    this->data.header.version = kVersion;
    this->data.header.type = static_cast<uint32_t>(this->type);
    this->data.satellite = satellite;
    this->data.hdop = hdop;
    this->data.latitude = latitude;
    this->data.longitude = longitude;
    this->data.altitude = altitude;

    // calculate CRC16
    size_t frame_size = sizeof(this->data);
    size_t payload_wo_crc16_size = frame_size - sizeof(this->data.trailer.checksum);
    this->data.trailer.checksum = crc16_CCITT(reinterpret_cast<uint8_t*>( &(this->data) ), payload_wo_crc16_size);  
  }
  
  size_t Gnss::GetAsBytes(uint8_t* data, size_t size)
  {
    if( data == nullptr || size < sizeof(this->data) )
    {
      return 0;
    }
    
    memcpy( data, &(this->data), sizeof(this->data) );
    return( sizeof(this->data) );
  }

} // namespace payload 
} // namespace bwlr3d 
} // namespace teapot
