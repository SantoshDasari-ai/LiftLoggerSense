# IMU Sensor Display with Touch Interface and Workout Tracking

This project combines an LCD display with touch functionality and a QMI8658C IMU sensor to create both an interactive sensor data visualization application and a workout tracking system for ESP32.

## Features

- Real-time display of accelerometer and gyroscope data from QMI8658C IMU sensor
- Touch-enabled interface using CST816S touch controller
- Visual representation of sensor data with charts
- Temperature monitoring
- Modern UI with color-coded panels
- Workout tracking system with:
  - Exercise selection (currently supporting bicep curls)
  - Automatic rep counting using IMU motion detection
  - Set tracking and history
  - Workout summary statistics
  - Multi-screen workout flow

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
- `main/workout_app.h` & `main/workout_app.cpp`: Workout tracking application with rep counting
- `main/lv_conf.h`: LVGL configuration
- `components/SensorLib`: Sensor library for QMI8658C IMU

## Dependencies

- ESP-IDF
- LVGL (Light and Versatile Graphics Library)
- ESP LCD drivers
- SensorLib component
- Node.js packages (see package.json)

## Workout Application

The workout application features a multi-screen UI that allows users to:

1. Start a new workout from the home screen
2. Select exercise type (current implementation focuses on bicep curls)
3. Track sets and reps with automatic rep detection using IMU sensor data
4. View set summaries and overall workout statistics
5. Navigate through the workout flow using an intuitive touch interface

## License

This project is open source and available under the [MIT License](LICENSE).
