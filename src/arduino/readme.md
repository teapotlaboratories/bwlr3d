# Source Code
 Here are the examples code for the Teapot BWLR3D device. The provided code here are not intended for production and only use to understand on how to interact with the components on the device.
- **I2CScan**: Test the communication to all I2C sensors.
	- Power-on all on-board sensor and ping all available I2C address
- **BoardTest**: Test LED indicator and read battery voltage.
- **DeepSleep**: Test deep-sleep capability of the device
- **GNSSSleep**: Test setting the L86-M33 module to sleep
- **PowerTest**: Test and measure power consumption for multiple case
- **SensorTest**: Test reading all on-board sensor
- **SimpleLoRaApplication**: Simple LoRa application
	- Connect to a LoRaWAN Gateway at boot
	- Get GNSS first fix ( loop indefintely until get satellite fix )
	- Periodically send sensor data to gateway
	- Periodically send gnss data to gateway
	- Interval for read plus transmit sensor data and gnss data can be different
