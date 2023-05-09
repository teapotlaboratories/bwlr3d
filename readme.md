
# Teapot BWLR3D
 <p align="center"> <img src="https://raw.githubusercontent.com/teapotlaboratories/bwlr3d/main/docs/images/device.jpg" alt="drawing"  width="50%" height="50%"/></p>
 
Teapot BWLR3D is an Asset Tracker and Environmental Sensor with Solar Energy Harvesting. 
The device is capable of sensing temperature, humidity, air pressure, air quality, light intensity using the on-board **BME688** plus **VEM7700**, and calculate device's AHRS using the **LSM6DSOX** as accelerometer and gyrometer with **LIS3MDL** as magnetometer. 
Equiped with low-power **L86-M33** GNSS module, the device is capable to locate itself anywhere in the world.
With **STM32WLE** MCU as it's core and **AEM10941** for solar charging, the device is capable ultra-low power operation with the possibility of indefinite battery-life by utilizing the solar charging capability.

Teapot BWLR3D is part of  [Teapot open-hardware project](https://github.com/teapotlaboratories). 
  
## Disclaimer
- The 1KM range is based on [AERQ - Air Quality Monitoring](https://github.com/Mircerson/AERQ/) design, but have not been tested on this device yet
- The position of the **BME688** sensor on the board might not be the most efficient
- The position of the **LIS3MDL** sensor might get interference from the surrounding components and the provided magnetic case does interfere with the sensor.

## Future Works
- Develop code to read sensor and send data when movement is detected. **LSM6DSOX** can be set to ultra-low power mode and set to wake MCU when movement is detected, all other sensors needs to be in power-down mode.
- Change SW4 wiring from GND to 3V3

## Specification

- [RAK3172](https://docs.rakwireless.com/Product-Categories/WisDuo/RAK3172-Module/Overview/): An STM32WLE5CC module
- [AEM10941](https://e-peas.com/product/aem10941): Solar energy harvesting chip
- On-board sensors:
	- [L86-M33](https://www.quectel.com/product/gnss-l86): GNSS module
	- [LSM6DSOX](https://www.st.com/en/mems-and-sensors/lsm6dsox.html): Accelerometer and Gyrometer
	- [LIS3MDL](https://www.st.com/en/mems-and-sensors/lis3mdl.html): Magnetometer
	- [VEML7700](https://www.vishay.com/docs/84286/veml7700.pdf): Ambient Light Intensity meter
	- [BME688](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme688): Environmental sensor
- 3.3V only power/pin. 
- 12uA Deep-Sleep
- Switchable TX Power. 14 dBm(50mA) or 22 dBm(140mA) ( on 915MHz frequency )
- Supports LoRaWAN 1.0.3
- 1KM+ Range
- UART2 breakout for **Arduino** progamming
- SWD breakout for **Mbed OS/STM32Cube** programming
- IPEX antenna connector 
- 3.7 Volts LiPo Battery

## Schematics

<p align="center"> <img src="https://raw.githubusercontent.com/teapotlaboratories/bwlr3d/main/hardware/schematic.png" alt="schematic"/></p>

Schematic revisions:
-   Revision 1: Initial design
-   Revision 2: Swap BATT_MEAS and IMU_INT pin, and add 0.1uF to voltage divider 

## Boards
 <p align="center">  <img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/images/pcb_render.gif" alt="pcb_render"  width="50%" height="50%"/><br><b>PCB Render</b></p>

Built using KiCAD, the board is design to be as small as possible with all components placed on both side of the PCB.
The following design are based on the latest revision.
| Top Board | Bottom Board |
|--|--|
| <p align="center"> <img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/images/assembled_front.jpg" alt="assembled_front"  width="63%" height="63%"/></p> | <p align="center"> <img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/images/assembled_back.jpg" alt="assembled_back"  width="70%" height="70%"/></p> |
| <p align="center"> <img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/images/pcb_front.png" alt="pcb_front"  width="68%" height="68%"/></p> | <p align="center"> <img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/images/pcb_back.png" alt="pcb_bottom"  width="77%" height="77%"/></p> |

 <p align="center"> <img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/hardware/pcb.png" alt="pcb"  width="50%" height="50%"/><br><b>PCB Top and Bottom Layout</b></p> 
  
### Case
<p align="center">  <img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/images/case_render.gif" alt="case_render"  width="50%" height="50%"/></p>

Built using [TinkerCAD](https://www.tinkercad.com), the cases are available with 2 variant, with or without the programming port. The cases are 3D printable with any generic 3D printer with/without suppport (depends on the orientation). The STL files are available [here](https://github.com/teapotlaboratories/bwlr3d/tree/main/hardware/case)
 <p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/images/case_open.jpg" alt="drawing"  width="50%" height="50%"/><br><b>Case Open</b></p>

The case is design to be as small as possible with an additional magnets in the back to ease the placement of the sensor. The following are the list of material used at the time of testing:
- 3.7v LiPo Battery, 500 mAh 50mm x 22mm x 48mm
- 4 piece of 8mm x 2mm neodymium magnet

<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/images/placement_showcase.gif" alt="placement_showcase"  width="50%" height="50%"/><br><b>Sensor Placement with Magnet</b></p>

### Measurement
Power consumption and solar charging current are measured using [Nordic PPK2](https://www.nordicsemi.com/Products/Development-hardware/Power-Profiler-Kit-2)  and [CurrentRanger](https://lowpowerlab.com/shop/product/152).
The following are the summary of the measurement:
- Transmit 102 bytes @ 14dBm:  225ms @ 48mA
- Deep-Sleep : 12 uA
- GNSS Fix - Cold Start: 15s @ 38mA
- GNSS Fix - Hot Start: 5s @ 35mA
- Sensor Read: 1.36s @ 9mA
- Direct Sunlight Solar Charge: 12mA
- Indirect Sunlight Solar Charge: 454uA

Notes: 
- GNSS time to fix depends on the conditions of the sky

<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/simple_lora_application-lora_transmit.png" alt="lora_transmit"  width="90%" height="90%"/><br><b>LoRa Transmit</b></p>

<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/deep_sleep.png" alt="deep_sleep"  width="90%" height="90%"/><br><b>Deep-Sleep</b></p>

<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/gnss_fix_cold_start.png" alt="gnss_fix-cold_start"  width="90%" height="90%"/><br><b>GNSS Fix - Cold Start</b></p>

<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/gnss_fix_hot.png" alt="gnss_fix-hot_start"  width="90%" height="90%"/><br><b>GNSS Fix - Hot Start</b></p>

<p align="center"><img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/sensor_read_without_gnss_fix.png" alt="sensor_read_without_gnss_fix"  width="90%" height="90%"/><br><b>Sensor Read</b></p>

| Solar Charge - Direct Sunlight | Solar Charge - Indirect Sunlight |
|--|--|
| <p align="center"> <img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/solar_charge_direct_sunlight.jpg" alt="solar_charge_direct_sunlight"  width="80%" height="80%"/></p> | <p align="center"> <img src="https://github.com/teapotlaboratories/bwlr3d/raw/main/docs/measurement/solar_charge_indirect_sunlight.jpg" alt="solar_charge_indirect_sunlight"  width="70%" height="70%"/></p> |

More measurement can be found [here](https://github.com/teapotlaboratories/bwlr3d/tree/main/docs/measurement)

### Bill Of Materials
Most of the components are generic and can be bought from any electornics/semi-conductor distributor. RAK3172 is the only component available in [RAKwireless store](https://store.rakwireless.com/products/wisduo-lpwan-module-rak3172?variant=40014759493830). The bill of materials can be downloaded [here](https://github.com/teapotlaboratories/bwlr3d/raw/main/hardware/bill_of_materials.csv)

> :warning: **Be sure to buy the RAK3172 variant without IPEX to use the On-Board Antenna** 

|Id |Designator                                           |Package                                |Quantity|Designation   |
|---|-----------------------------------------------------|---------------------------------------|--------|--------------|
|1  |BT1                                                  |JST_PH_S2B-PH-K_1x02_P2.00mm_Horizontal|1       |3.7v          |
|2  |C14,C23,C29                                          |C_0603_1608Metric                      |3       |1uF           |
|3  |C17,C31,C32,C1,C9,C22,C8,C10,C34,C13                 |C_0603_1608Metric                      |12      |10uF          |
|4  |C2,C4                                                |CP_EIA-3528-15_AVX-H_Pad1.50x2.35mm    |2       |330uF         |
|5  |C21                                                  |C_0603_1608Metric                      |1       |4.7uF         |
|6  |C27                                                  |C_0603_1608Metric                      |1       |2.2uF         |
|7  |C5,C20,C26,C12,C28,C19,C30,C11,C25,C3,C33,C24,C18,C35|C_0603_1608Metric                      |14      |0.1uF         |
|8  |C6                                                   |C_1210_3225Metric                      |1       |150uF         |
|9  |C7                                                   |C_0603_1608Metric                      |1       |22uF          |
|10 |D1                                                   |LED_0603_1608Metric                    |1       |RED           |
|11 |D2                                                   |LED_0603_1608Metric                    |1       |GREEN         |
|12 |E1                                                   |ANT-915-USP410                         |1       |ANT-915-USP410|
|13 |L2                                                   |IND_LPS4012-103MRB                     |1       |10uH          |
|14 |L3                                                   |L_0603_1608Metric                      |1       |10uH          |
|15 |Q1                                                   |SOT-23                                 |1       |PJA3407       |
|16 |Q2,Q4                                                |SOT-323_SC-70                          |2       |DMG1012UW     |
|17 |Q3                                                   |SOT-23                                 |1       |PMV27UPER     |
|18 |R1                                                   |R_0603_1608Metric                      |1       |4.7K          |
|19 |R11                                                  |R_0603_1608Metric                      |1       |100K          |
|20 |R12,R13,R5                                           |R_0603_1608Metric                      |3       |1K            |
|21 |R14                                                  |R_0603_1608Metric                      |1       |400K          |
|22 |R16                                                  |R_0603_1608Metric                      |6       |0             |
|23 |R7,R2,R3,R6,R21,R4                                   |R_0603_1608Metric                      |6       |10K           |
|24 |R9,R8,R19,R15,R18,R10,R17,R20                        |R_0603_1608Metric                      |8       |1M            |
|25 |SC4,SC1,SC3,SC2                                      |KXOB25-05X3F                           |4       |KXOB25-05X3F  |
|26 |SW3                                                  |SW_SPST_B3U-1000P                      |1       |RESET         |
|27 |SW4                                                  |SW_SPST_B3U-1000P                      |1       |BOOT          |
|28 |U1                                                   |BME688                                 |1       |BME688        |
|29 |U2                                                   |RAK3172                                |1       |RAK3172       |
|30 |U3                                                   |QFN-28-1EP_5x5mm_P0.5mm_EP3.35x3.35mm  |1       |AEM10941-QFN  |
|31 |U5                                                   |L86-M33                                |1       |L86-M33       |
|32 |U6,U4                                                |SOT-23-5                               |2       |XC6215        |
|33 |U7                                                   |PQFN50P250X300X97-14N_0.59x0.30        |1       |LSM6DSOXTR    |
|34 |U8                                                   |LGA12_ST                               |1       |LIS3MDL       |
|35 |U9                                                   |VEML7700-TT                            |1       |VEML7700-TT   |

## Programming

> :warning: **Board can only be powered using the LiPo Battery** 

Programming the device can be done over the **UART2** or **SWD**, available on the right side of the board.
Out of the factory, the RAK3172 chip ships with an **AT firmware** that can be tested by connecting a USB-to-UART bridge to the **UART2** port.

The following are some very good tutorial to start developing with the device:

 - [Communicating with the AT firmware](https://docs.rakwireless.com/Product-Categories/WisDuo/RAK3172-Module/Quickstart/#rak3172-as-a-lora-lorawan-modem-via-at-command)
 - [Programming with Arduino](https://docs.rakwireless.com/Product-Categories/WisDuo/RAK3172-Module/Quickstart/#rak3172-as-a-stand-alone-device-using-rui3)
 - [Programming with STM32Cube](https://docs.rakwireless.com/Product-Categories/WisDuo/RAK3172-Module/Low-Level-Development/#rak3172-on-stm32cubeide-with-stm32wl-sdk-v1-0-0)
 - [Programming with MbedOS](https://github.com/hallard/LoRa-E5-Tiny/blob/main/README.md#compile-and-flash-firmware)

For connecting to the **UART2** port, use any USB-to-UART bridge module. In testing, the [Sparkfun](https://www.sparkfun.com/products/14050) board is used for communication with AT firmware and programming over **Arduino**.
 <p align="center"> <img src="https://raw.githubusercontent.com/teapotlaboratories/bwlr3d/master/docs/images/sparkfun_ftdi.jpeg" width="30%" height="30%"><br>Sparkfun USB-to-UART Bridge</p>

> :warning: **Be sure to only use 3.3V module. Do not 5V module** 

For connecting to the **SWD** port, use ST-Link v2  in-circuit debugger and programmer from STM. In testing, ST-Link v2 clone will not work. The ST-Link v2 should atleast be reconizeable by the [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html).
A cheap and alternative way to get an authorized ST-Link is to buy a Nucleo board, cut the top part which contain the ST-Link and use it as an external programmer.
 <p align="center"> <img src="https://raw.githubusercontent.com/teapotlaboratories/bwlr3d/master/docs/images/nucleo_st-linkv2.jpeg" width="70%" height="70%"><br>ST-Link v2 from a Nucleo Development Board</p>
Here are some good tutorial to convert a Nucleo to and external ST-Link v2:

 - https://www.radioshuttle.de/en/turtle-en/nucleo-st-link-interface-en/
 - https://jeelabs.org/book/1547a/index.html

## Notes
There are some issue, notes, and behavior that was discovered at the time of testing and development. The following are those discovery:
- Soldering the solar cell is better to be done manually using a soldering iron. Without proper reflow oven, it may damage the solar cell and reduces it's efficiency
- PRIMIN is available to use as the input for AEM10941 Primary Battery input. See schematic for more detail	

## Reference
The project won't be possible without the amazing work from people across the globe. The following are the reference to those awesome projects:

 - [LoRa e5 Tiny](https://github.com/hallard/LoRa-E5-Tiny)
 - [AERQ - Air Quality Monitoring](https://github.com/Mircerson/AERQ/)
 - [TSEM](https://hackaday.io/project/159139-tiny-solar-energy-module-tsem)

## License
The product is open-source! However, some part of library used under **src**, might have it's own license.
Please reach out or create a ticket to report any license violation.

![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
