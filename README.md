# STM32 CAN Dashboard Panel

Control unit for generating CAN signals based on an STM32. Inputs from magnetic encoders, buttons, and a potentiometer.

My personal project with goal to expand my knowledge about industry standards communication protocols.

Created to provide interactive demo CAN inputs for my [Motorcycle Instrument Cluster project](https://github.com/repetren/moto_instrument_cluster).

![Image](https://github.com/user-attachments/assets/b68cb69c-caa8-4185-b78f-092eaa4e9a21)

[![Watch the video](https://img.youtube.com/vi/6McVTvd34ow/maxresdefault.jpg)](https://youtu.be/6McVTvd34ow)

### [Watch demo and work in progress video on YouTube](https://youtu.be/6McVTvd34ow)

<img width="1080" height="634" alt="Image" src="https://github.com/user-attachments/assets/994d8275-6f5f-48e5-96df-f77f57af5563" />

### Features:
- Timers for CAN frame transmission schedule  
- Buttons via EXTI
- Software button debounce
- LED indicators for power and mode indicators
- Two potentiometer modes for two CAN frames
- ADC with DMA for potentiometer sampling

### Power and ESD
- 5 V input, 3.3 V logic. Conversion through LD1117A regulator
- TVS diode for CAN bus and power input protection

## Project Structure

I'm using **[app.c](App/Src/app.c)** to avoid deal with USER CODE BLOCKs inside STM32 generated code.

**CAN_Control_Panel**

App:
- Inc - Header
- Src - Source - **main [app.c](App/Src/app.c)**

Core: Startup + HAL init (generated)
- Inc
- Src - **main.c** contains **app.c**

Docs:
- CAN message map

Hardware:
- PCB - Gerbers, BOM, circuit diagram
- 3DPrints - Knobs, buttons, frame .stl files

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

The panel collects input data from physical controls and sensors, processes them periodically in the main loop, and sends formatted messages over CAN and UART.

| Input                     | Interface       | Processed in            | Purpose                                    |
| :-----------------------: | --------------- | ----------------------- | ------------------------------------------ |
| Buttons (6 pcs)           | EXTI interrupts | EXTI + Debounce.        | Gear, turn signals, potentiometer modes    |
| Magnetic encoders (2 pcs) | I2C             | `[encoder_remap](App/Src/encoder_utils.c)`       | Angle to telemetry parameter               |
| Potentiometer             | ADC + DMA       | `[fuel_and_temp_utils.c](App/Src/fuel_and_temp_utils.c)` | Fuel/temperature input (depending on mode) |

#### Periodic task handling ([app.c](App/Src/app.c))
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

## Build and run

### Requirements
 - VS Code
 - Installed [STM32CubeIDE for Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension)
 - [STLINK-V3MINIE](https://www.digikey.de/de/products/detail/stmicroelectronics/STLINK-V3MINIE/16284301)

### VS Code
1. Open repository folder
2. Configure project. CMD (CTRL) + Shift + P
```
Setup STM32Cube project(s)
```
<img width="634" height="86" alt="Image" src="https://github.com/user-attachments/assets/908955fa-ae71-4bb9-970b-bc4206f19bd2" />

3. Select device

<img width="775" height="348" alt="Image" src="https://github.com/user-attachments/assets/e120b31a-1882-4ca3-9a6c-c2c42c96c56a" />

4. Click "Save and close"

5. Run and debug

<img width="228" height="86" alt="Image" src="https://github.com/user-attachments/assets/43cf27ef-3385-42bd-b94c-251a5485301b" />

6. Select ST-LINK server

7. Select project .elf

<img width="754" height="107" alt="Image" src="https://github.com/user-attachments/assets/92c26fbf-a4ea-44e0-b478-a664b0a62e8b" />

8. Click "Continue" (F5)

<img width="528" height="112" alt="Image" src="https://github.com/user-attachments/assets/e2813105-840a-4921-8ae6-edc6c6eea660" />

## Future improvements
- Replacing big SWD 14-pin connector to 6-pin version