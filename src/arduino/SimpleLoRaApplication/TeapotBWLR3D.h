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
      ReturnCode    GetSensorData( SensorData& data );
      void          ResetGnssData();
      ReturnCode    ProcessGnssStream(float hdop, uint32_t sat, unsigned long ms);
      TinyGPSPlus&  GetGnssData();
  };
  
} // namespace bwlr3d 
} // namespace teapot


/* lora payload definition */
namespace teapot{
namespace bwlr3d {
namespace payload {
  /* Payload Component Definition */
  const uint32_t kVersion = 0x01;
  enum class Type {
    kEnvironmental = 0x01,
    kImu = 0x02,
    kGnss = 0x03
  };
  
  struct Header {
    uint32_t version;
    uint32_t type;
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
  class Environmental {
    struct Frame {
      Header header;
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
      Environmental( float temperature,
                     uint32_t pressure,
                     float humidity,
                     uint32_t gas_resistance,
                     float lux );
      size_t GetAsBytes(uint8_t* data, size_t size);
  };

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
  
  struct Gnss {
    Header header;
    uint32_t timestamp;
    uint16_t satellite; 
    float hdop;
    double latitude;
    double longitude;
    double altitude;
    Trailer trailer;    
  } __attribute__((packed));

} // namespace payload 
} // namespace bwlr3d 
} // namespace teapot

#endif
