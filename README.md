# INA226 High-Accuracy Power Monitor (R100 Shunt, 12V System)

![Preview](preview.png.png)

**Real-time high-accuracy power monitoring system using INA226 + R100 shunt (0.1Ω)**
Tested on real 12V loads with live current, voltage, and power tracking.

---

## Why this project matters

Most INA226 projects online are **incorrectly calibrated**, leading to misleading measurements.

This project is different:

* Uses **real R100 shunt (0.1Ω)** — not fake assumptions
* Calibrated for **actual 12V systems**
* Shows **true current limits (~0.8A safe range)**
* Includes **live filtering + noise reduction**
* Built and tested on **real hardware (not simulation)**

This is not a demo — this is a **real measurement tool**.

---

## Key Features

* Real-time current measurement (A)
* Real-time voltage measurement (V)
* Real-time power calculation (W)
* Smooth UI with LVGL (circular gauge)
* Digital segmented display
* Overrange detection (~0.8A limit)
* Signal filtering (low-pass + deadband)
* Debug mode (raw register monitoring)

---

## Hardware Used

* Tuya T5-E1 AMOLED development board
* INA226 current sensor module (R100 shunt)
* 12V DC power source
* Real load (tested around 5W)

---

## System Specifications

* **Shunt resistor:** 0.1Ω (R100)
* **Voltage range:** up to 36V (INA226 limit)
* **Tested voltage:** 12V system
* **Max accurate current:** ~0.8A
* **Interface:** I2C
* **Display:** AMOLED 1.75" (466×466)

---

## Real Test

Test conditions:

* Supply: 12V
* Load: ~5W
* Measured current: ~0.3A

System displays:

* Current (A)
* Power (W)
* Voltage (V)
* Warning when nearing measurement limit

---

## Wiring

INA226 → Tuya T5-E1:

* SDA → GPIO17
* SCL → GPIO16
* VCC → 3.3V
* GND → GND

---

## Calibration Notes

* Bus voltage corrected using real measurement factor
* Shunt measurement based on **0.1Ω resistor**
* Designed to avoid:

  * False high power readings
  * Unrealistic current values
  * Noise instability

---

## Build & Flash

```bash
cd C:\Users\maher\tuyaopen
.\export.bat

cd C:\Users\maher\tuyaopen\my_ina219_ui
python ..\tos.py clean
python ..\tos.py build
python ..\tos.py flash --port COM4
```

---

## Project Structure

* `/src` → main application (UI + INA226 logic)
* `/config` → board configurations
* `CMakeLists.txt` → build system
* `app_default.config` → default settings

---

## Video Demonstration

Full build and real testing:

https://youtube.com/@inventorkr1

---

## About Me

**Inventor KR**
Electronics Developer & YouTube Creator

* Power systems
* Batteries & inverters
* Real-world testing
* Arduino & embedded projects

---

## License

MIT License
