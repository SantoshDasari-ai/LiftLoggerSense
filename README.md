# IMU Sensor Display with Touch Interface

This project combines an LCD display with touch functionality and a QMI8658C IMU sensor to create an interactive sensor data visualization application for ESP32.

## Features

- Real-time display of accelerometer and gyroscope data from QMI8658C IMU sensor
- Touch-enabled interface using CST816S touch controller
- Visual representation of sensor data with charts
- Temperature monitoring
- Modern UI with color-coded panels

## Hardware Requirements

- ESP32 development board
- ST7789 LCD display (240x280 resolution)
- CST816S touch controller
- QMI8658C IMU sensor

## Pin Configuration

### LCD Display (SPI)

- SCLK: GPIO 6
- MOSI: GPIO 7
- RST: GPIO 8
- DC: GPIO 4
- CS: GPIO 5
- BL: GPIO 15

### Touch and IMU (I2C)

- SCL: GPIO 10
- SDA: GPIO 11
- Touch RST: GPIO 13
- Touch INT: GPIO 14

## Building and Flashing

This project is built using ESP-IDF. To build and flash:

```bash
idf.py build
idf.py -p [PORT] flash monitor
```

## Project Structure

- `main/main.cpp`: Main application code combining LCD, touch, and IMU functionality
- `main/lv_conf.h`: LVGL configuration
- `components/SensorLib`: Sensor library for QMI8658C IMU

## Dependencies

- ESP-IDF
- LVGL (Light and Versatile Graphics Library)
- ESP LCD drivers
- SensorLib component

## License

This project is open source and available under the [MIT License](LICENSE).
