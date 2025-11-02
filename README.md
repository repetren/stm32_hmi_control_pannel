# STM32 CAN Dashboard Panel

Control unit for generating CAN signals based on an STM32. Inputs from magnetic encoders, buttons, and a potentiometer.

My personal project with goal to expant my knowlegde about intdustry standards communiation protocols.

Created to provide interactive demo CAN inputs for my [Motorcycle Instrument Cluster project](https://github.com/repetren/moto_instrument_cluster).

![Image](https://github.com/user-attachments/assets/b68cb69c-caa8-4185-b78f-092eaa4e9a21)

[![Watch the video](https://img.youtube.com/vi/6McVTvd34ow/maxresdefault.jpg)](https://youtu.be/6McVTvd34ow)

### [Watch demo and work in progress video on YouTube](https://youtu.be/6McVTvd34ow)

<img width="1080" height="634" alt="Image" src="https://github.com/user-attachments/assets/994d8275-6f5f-48e5-96df-f77f57af5563" />

## Features

### Power and ESD
- 5 V input, 3.3 V logic. Conversion through LD1117A regulator
- TVS diode for CAN bus and power input protection

### Features:
- Timers for CAN frame transmission schedule  
- Buttons via EXTI
- Software button debounce
- LED indicators for power and mode indicators
- Two potentiometer modes for two CAN frames
- ADC with DMA for potentiometer sampling

## Project Structure

I'm using App.c to avoid deal with USER CODE BLOCKs inside STM32 generated code.

!!ADD FOLDER STRUCTURE!!

## Hardware
- STM32F446RET6 as main MCU
- 6 push buttons via EXTI interrupts
- 2 rotary encoders (AS5048B) over I2C
- 1 potentiometer with analog input
- CAN transceiver: SN65HVD230DR
- UART for debugging and serial output
- SWD interface for firmware flashing and debugging

## Software Architecture

![Image](https://github.com/user-attachments/assets/4e9286e8-073c-4ac3-a5f3-334426e7a35e)

#### Inputs
The panel collects input data from physical controls and sensors, processes them periodically in the main loop, and sends formatted messages over CAN and UART.

| Input                     | Interface       | Processed in            | Purpose                                    |
| :-----------------------: | --------------- | ----------------------- | ------------------------------------------ |
| Buttons (6 pcs)           | EXTI interrupts | EXTI + Debounce.        | Gear, turn signals, potentiometer modes    |
| Magnetic encoders (2 pcs) | I2C             | `encoder_remap.c`       | Angle to telemetry parameter               |
| Potentiometer             | ADC + DMA       | `fuel_and_temp_utils.c` | Fuel/temperature input (depending on mode) |

#### Periodic task handling (app.c)
Tasks are executed based on timing from the TIM14 timer.

|  Frequency | Task responsibilities                                                                                 |
| :--------: | --------------------------------------------------------------------------------                      |
| **100 Hz** | Update telemetry, monitor temperature, send telemetry                                                 |
|  **50 Hz** | Update lights and send over CAN light frame                                                           |
|  **25 Hz** | Fuel conversion, range estimation, vehicle info broadcast, low fuel notification, CAN fuel frame send |

#### Output communication

| Output  | Used for                                                                                  |
| ------- | --------------------------------------------------------------------                      |
| UART    | Debug messages                                                                            |
| CAN bus | Telemetry, light control, fuel, and notification frames sent to Raspberry Pi with CAN HAT |

## CAN Frame Map
![Image](https://github.com/user-attachments/assets/5da04f03-c850-4726-8469-995679e1c428)



