#ifndef TEAPOT_BWLR3D_H
#define TEAPOT_BWLR3D_H

#include "Arduino.h"
#include "Adafruit_LIS3MDL.h"
#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_BME680.h"
#include "Adafruit_VEML7700.h"
#include "TinyGPSPlus.h"

// configuration for teapot bwlr3d application
#include "TeapotBWLR3DConfig.h"

/* pin definition */
#define RED_LED               PA0
#define GREEN_LED             PA1
#define PERIPHERAL_POWER_EN   PB5
#define BATT_MEASURE_EN       PB2
#define POWER_STATUS          PB12
#define GPS_FORCE_ON          PA15
#define GPS_TXD               PB7
#define GPS_RXD               PB6
#define I2C_SDA               PA11
#define I2C_SCL               PA12
#define MAG_INT               PB3
#if BOARD_REVISION == 0x01
  #define IMU_INT             PB4
  #define BATT_MEASURE        PA10
#elif BOARD_REVISION == 0x02
  #define IMU_INT             PA10
  #define BATT_MEASURE        PB4
#else
  #ifndef BOARD_REVISION
    #error "BOARD_REVISION not set"
  #endif
  #error "invalid BOARD_REVISION value"
#endif

#ifndef ADC_TO_BATTERY
  #error "ADC_TO_BATTERY not set"
#endif

/* application definition */
namespace teapot{
namespace bwlr3d {
  enum class ReturnCode {
    kOk = 0,
    kError,
    kLis3mdlNotFound,
    kLsm6dsoxNotFound,
    kBme68xNotFound,
    kVeml7700NotFound,
    kInvalidLed,
    kPeripheralOff,
    kBme68xReadFail,
    kTimeout,
  };
  
  enum class Led {
    kGreen = 0,
    kRed,
  };

  enum class Sensor : uint32_t {
    kNone     = 0b0,
    kAll      = 0b1111,
    kLis3mdl  = 0b1,
    kLsm6dsox = 0b10,
    kVeml7700 = 0b100,
    kBme68x   = 0b1000
  };
  uint32_t operator|( const Sensor m, const Sensor n );  
  uint32_t operator&( const uint32_t m, const Sensor n );  
  
  struct SensorData {
    // Temperature (Celsius)
    float temperature;
    // Pressure (Pascals)
    uint32_t pressure;
    // Humidity (RH %)
    float humidity;
    // Gas resistor (ohms)
    uint32_t gas_resistance;
    // Lux
    float lux;
    // Magnetic (uT)
    sensors_event_t mag;
    // Accel (m/s^2) 
    sensors_event_t accel;
    // Gyro (rad/s),
    sensors_event_t gyro;
  };

  class Application {
    private:
      Adafruit_LSM6DSOX lsm6dsox;
      Adafruit_LIS3MDL  lis3mdl;
      Adafruit_BME680   bme68x;
      Adafruit_VEML7700 veml7700;
      TinyGPSPlus       l86;
      bool              enable_peripheral;
      
      void  SetupLis3mdl();
      void  SetupLsm6dsox();
      void  SetupBme68x();
    public:
      Application();
      void          Initialize();
      ReturnCode    ConfigureSensor();
      void          EnablePeripheral( bool enable );
      ReturnCode    EnableLed( Led led, bool enable );
      ReturnCode    EnableGnss( bool enable );
      int           ReadBatteryAdc();
      float         ReadBatteryVoltage();
      bool          ReadPowerStatus();
      ReturnCode    GetSensorData( SensorData& data, uint32_t sensor );
      ReturnCode    GetSensorData( SensorData& data, Sensor sensor = Sensor::kAll );
      void          ResetGnssData();
      
      /**
       * Process incoming GNSS NMEA stream from Serial1.
       *
       * Process and busy wait until receive hdop value at or below target
       * and number of satellite at or above target.
       * Process returns kTimeout if the process is not able to achieve target
       * for `ms` time
       *
       * @param hdop Target HDOP value
       * @param sat Target number of satellite
       * @param ms Timeout in milliseconds to reach the target value
       * @return `kOk` if target reached, or `kTimeout` if timeout reached
       */
      ReturnCode    ProcessGnssStream(float hdop, uint32_t sat, unsigned long ms);
      TinyGPSPlus&  GetGnssData();
      void          BlinkLedIndicator( uint32_t ms );
      void          BlinkLedIndicator(Led led, uint32_t ms);
      void          BlinkLedIndicator(Led led, int total_blink, uint32_t ms);
  };
  
} // namespace bwlr3d 
} // namespace teapot


/* lora payload definition */
namespace teapot{
namespace bwlr3d {
namespace payload {
  /* Payload Component Definition */
  const uint8_t kVersion = 0x01;
  enum class Type {
    kEnvironmental = 0x01,
    kImu = 0x02,
    kGnss = 0x03
  };
  
  struct Header {
    uint8_t version;
    uint8_t type;
  } __attribute__((packed));

  struct Trailer {
    uint32_t checksum; // CRC32
  } __attribute__((packed));
    
  struct Vector {
    float x;
    float y;
    float z;
  } __attribute__((packed));

  /* Payload Definition */  
  /* environmental payload */
  class Environmental {
    struct Frame {
      Header header;
      float battery;
      float temperature;  // Celsius
      uint32_t pressure;  // Pascals
      float humidity; // RH %
      uint32_t gas_resistance;  //  ohms
      float lux;  /// lux
      Trailer trailer;
    } __attribute__((packed));
    
    const Type type = teapot::bwlr3d::payload::Type::kEnvironmental;
    Frame data;
    public:
      Environmental( 
        float battery,
        float temperature,
        uint32_t pressure,
        float humidity,
        uint32_t gas_resistance,
        float lux 
      );
      size_t GetAsBytes(uint8_t* data, size_t size);
  };

  /* imu payload */
  class Imu {
    struct Frame {
      Header header;
      Vector magnetic;  // micro-Tesla (uT)
      Vector acceleration;  // meter per second (m/s^2
      Vector gyro;  // rad/s
      Trailer trailer;
    } __attribute__((packed));
    const Type type = teapot::bwlr3d::payload::Type::kImu;
    Frame data;
    public:
      Imu( const Vector& mag, const Vector& accel, const Vector& gyro );
      size_t GetAsBytes(uint8_t* data, size_t size);    
  };
  
  /* gnss payload */
  class Gnss {
    struct Frame {
      Header header;
      uint16_t satellite; 
      float hdop;
      double latitude;
      double longitude;
      double altitude;
      Trailer trailer;    
    } __attribute__((packed));
    const Type type = teapot::bwlr3d::payload::Type::kGnss;
    Frame data;
    public:
      Gnss( 
        uint16_t satellite,
        float hdop,
        double latitude,
        double longitude,
        double altitude
      );
      size_t GetAsBytes(uint8_t* data, size_t size);
  };  

} // namespace payload 
} // namespace bwlr3d 
} // namespace teapot

#endif
