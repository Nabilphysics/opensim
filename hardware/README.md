# Hardware Notes

## Main components

| Component | Purpose |
|---|---|
| Seeed Studio XIAO ESP32-C6 | Main embedded controller |
| BNO055 IMU sensors | Quaternion orientation output |
| TCA9548A I2C multiplexer | Supports up to 8 IMUs with the same I2C address |
| microSD card module | Local `.sto` file storage |
| Push button | Calibration, start, stop, and long-press configuration trigger |
| Status LED | Recording/status feedback |
| 3.7 V Li-ion battery | Portable power source |

## Pin mapping used in firmware

| Signal | ESP32-C6 pin |
|---|---:|
| Button | D1 |
| LED | D2 |
| I2C SDA | GPIO22 |
| I2C SCL | GPIO23 |
| SD CS | GPIO21 |
| SD SCK | GPIO19 |
| SD MISO | GPIO20 |
| SD MOSI | GPIO18 |

## TCA9548A channel convention

The selected body segments are mapped in the order selected from the web interface:

- First selected segment -> TCA channel 0
- Second selected segment -> TCA channel 1
- Third selected segment -> TCA channel 2
- ...
- Eighth selected segment -> TCA channel 7

Make sure the physical IMU wiring follows the same channel order selected in the browser.
