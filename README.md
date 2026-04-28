# T5-E1 Smart Power Monitor
preview.png.png
Real-time power monitor using Tuya T5-E1 AMOLED display and INA226 sensor.

## Current Version (R100 Shunt)

This version is configured for INA226 module with onboard shunt:

- Shunt: R100 = 0.1Ω
- Voltage: 12V system
- Max safe current: ~0.8A
- Interface: I2C
- Display: AMOLED 1.75" (466x466)

## Hardware Used

- Tuya T5-E1 AMOLED board
- INA226 current sensor (R100 module)
- 12V power source

## Wiring

INA226 → T5-E1:

- SDA → GPIO17  
- SCL → GPIO16  
- VCC → 3.3V  
- GND → GND  

## Features

- Real-time voltage measurement
- Real-time current measurement
- Real-time power calculation
- Custom circular LVGL UI
- Overcurrent warning (0.8A limit)

## Build & Flash

```bash
cd C:\Users\maher\tuyaopen
.\export.bat
cd C:\Users\maher\tuyaopen\my_ina219_ui
python ..\tos.py clean
python ..\tos.py build
python ..\tos.py flash --port COM4
