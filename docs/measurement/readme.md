# Measurement
Power consumption and solar charging current are measured using [Nordic PPK2](https://www.nordicsemi.com/Products/Development-hardware/Power-Profiler-Kit-2)  and [CurrentRanger](https://lowpowerlab.com/shop/product/152).
The following are the summary of the measurement:
- Transmit 102 bytes @ 14dBm:  225ms @ 48mA
- Deep-Sleep : 12 uA
- GNSS Fix - Cold Start: 15s @ 38mA
- GNSS Fix - Hot Start: 5s @ 35mA
- Sensor Read: 1.36s @ 9mA
- Direct Sunlight Solar Charge: 9mA
- Indirect Sunlight Solar Charge: 300uA

Notes: 
- GNSS time to fix depends on the conditions of the sky

## LoRa Transmit
<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/simple_lora_application-lora_transmit.png" alt="lora_transmit"  width="90%" height="90%"/></p>

## Deep-Sleep
<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/deep_sleep.png" alt="deep_sleep"  width="90%" height="90%"/></p>

## GNSS Fix - Cold Start
<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/gnss_fix_cold_start.png" alt="gnss_fix-cold_start"  width="90%" height="90%"/></p>

## GNSS Fix - Hot Start
<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/gnss_fix_hot.png" alt="gnss_fix-hot_start"  width="90%" height="90%"/></p>

## Sensor Read without GNSS Fix
<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/sensor_read_without_gnss_fix.png" alt="sensor_read"  width="90%" height="90%"/></p>

## Sensor Read with GNSS Fix
<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/sensor_read_and_gnss_fix_hot_start.png" alt="sensor_read"  width="90%" height="90%"/></p>

## Simple LoRa Application - Sensor Read and GNSS Fix then Transmit
Power consumption for the **SimpleLoraApplication** when it read all sensor data, wait for GNSS fix and LoRa transmit
<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/simple_lora_application-sensor_and_gnss_data_transmit.png" alt="sensor_read"  width="90%" height="90%"/></p>

## Simple LoRa Application - Sensor Read Fix then Transmit
Power consumption for the **SimpleLoraApplication** when it read all sensor data and LoRa transmit
<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/simple_lora_application-sensor_data_transmit.png" alt="sensor_read"  width="90%" height="90%"/></p>
