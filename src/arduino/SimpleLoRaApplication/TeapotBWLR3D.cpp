#include "TeapotBWLR3D.h"
#include <CRC32.h>

/* application implementation */
namespace teapot {
namespace bwlr3d {
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
    digitalWrite(BATT_MEASURE_EN, HIGH);
    const int battery_adc = analogRead( BATT_MEASURE );
    digitalWrite(BATT_MEASURE_EN, LOW);
    return battery_adc;
  }

  float Application::ReadBatteryVoltage()
  {
    return ((float) ReadBatteryAdc()) * ADC_TO_BATTERY;
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
    this->lis3mdl.setOperationMode(LIS3MDL_SINGLEMODE);
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
    this->bme68x.setTemperatureOversampling(BME680_OS_8X);
    this->bme68x.setHumidityOversampling(BME680_OS_2X);
    this->bme68x.setPressureOversampling(BME680_OS_4X);
    this->bme68x.setIIRFilterSize(BME680_FILTER_SIZE_3);
    this->bme68x.setGasHeater(320, 150); // 320*C for 150 ms
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
    if (!this->bme68x.begin()) {
      return ReturnCode::kBme68xNotFound;
    }
    SetupBme68x();
  
    /* configure VEML7700 */
    if (!this->veml7700.begin()) {
      return ReturnCode::kVeml7700NotFound;
    }
    return ReturnCode::kOk;
  }
  
  ReturnCode Application::GetSensorData( SensorData& data )
  {    
    if( !this->enable_peripheral )
    {
      return ReturnCode::kPeripheralOff;
    }
      
    /* Read LIS3MDL sensor */
    this->lis3mdl.getEvent(&data.mag);
  
    /* Read LSM6DS0X sensor */
    sensors_event_t unused_temp;
    this->lsm6dsox.getEvent(&data.accel, &data.gyro, &unused_temp);
  
    /* Read VEML7700 sensor */
    data.lux = this->veml7700.readLux(VEML_LUX_AUTO);
    
    /* Read BME68x sensor */
    if ( !this->bme68x.performReading()) {
      return ReturnCode::kBme68xReadFail;
    }
    data.temperature      = bme68x.temperature;
    data.pressure         = bme68x.pressure;
    data.humidity         = bme68x.humidity;
    data.gas_resistance   = bme68x.gas_resistance;
    
    return ReturnCode::kOk;
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
  
} // namespace bwlr3d 
} // namespace teapot

/* lora payload implementation */
namespace teapot{
namespace bwlr3d {
namespace payload {
  
  Environmental::Environmental( float temperature,
                                uint32_t pressure,
                                float humidity,
                                uint32_t gas_resistance,
                                float lux )
  {
    // set data
    this->data.header.version = kVersion;
    this->data.header.type = static_cast<uint32_t>(this->type);
    this->data.temperature = temperature;
    this->data.pressure = pressure;
    this->data.humidity = humidity;
    this->data.gas_resistance = gas_resistance;
    this->data.lux = lux;

    // calculate CRC32
    size_t frame_size = sizeof(this->data);
    size_t payload_wo_crc32_size = frame_size - sizeof(this->data.trailer.checksum);
    this->data.trailer.checksum = CRC32::calculate(&(this->data), payload_wo_crc32_size);    
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

  
  Imu::Imu( const Vector& mag, const Vector& accel, const Vector& gyro  )
  {
    // set data
    this->data.header.version = kVersion;
    this->data.header.type = static_cast<uint32_t>(this->type);
    this->data.magnetic = mag;
    this->data.acceleration = accel;
    this->data.gyro = gyro;

    // calculate CRC32
    size_t frame_size = sizeof(this->data);
    size_t payload_wo_crc32_size = frame_size - sizeof(this->data.trailer.checksum);
    this->data.trailer.checksum = CRC32::calculate(&(this->data), payload_wo_crc32_size);    
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

  // TODO: implement lora payload for gnss

} // namespace payload 
} // namespace bwlr3d 
} // namespace teapot
