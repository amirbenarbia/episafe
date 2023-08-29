# Episafe Sensor Monitoring System

## Overview

The Episafe Sensor Monitoring System is an embedded IoT system aimed at providing real-time sensor data collection and Bluetooth-based transmission to a host device. Built on the ESP32 microcontroller, the system captures various physiological data points including heart rate, body temperature, acceleration, conductance, and more. With a built-in battery monitoring system, it offers a highly portable and efficient platform for health and safety applications.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Hardware Setup](#hardware-setup)
- [Usage](#usage)
- [Data Points](#data-points)
- [Error Handling](#error-handling)
- [Contributing](#contributing)
- [License](#license)

## Prerequisites

- ESP32 Development Board
- Arduino IDE (version >= 1.8.10)
- Pulse Sensor
- Adafruit MPU6050 Accelerometer and Gyroscope
- Adafruit MLX90614 Temperature Sensor
- Additional components (LEDs, buttons, etc.)

## Installation

1. **Clone the repository**

   ```
   git clone https://github.com/yourusername/Episafe.git
   ```

2. **Open Arduino IDE**

3. **Install Required Libraries**

   Navigate to `Sketch > Include Library > Manage Libraries...`, then search and install the following libraries:
   - OneButton
   - Adafruit_MLX90614
   - Adafruit_MPU6050
   - BluetoothSerial
   - ArduinoJson

4. **Select Board and Port**

   Choose `ESP32` from `Tools > Board` and select the COM Port the board is connected to.

5. **Compile and Upload**

   Compile the code by clicking the checkmark button, and then upload it to your ESP32 by clicking the right-arrow button.

## Hardware Setup

Follow the hardware schematic [here](link_to_schematic) to connect all the components.

## Usage

The system starts in the "Data Capture" state, where it begins to read the sensor data. The data is stored in an internal buffer until it reaches the specified limit, at which point it moves to the "Data Transmission" state.

### Buttons

- **Button 1**: 
  - Single Click: Manually trigger sensor reading
  - Long Press: Trigger "Crisis" state

- **Button 2**: 
  - Double Click: Report false alarm
  - Long Press: Reset the system

## Data Points

- Heart Rate (BPM)
- Body Temperature (°C)
- Acceleration (m/s^2)
- Conductance (µS)
- Battery Level (%)

## Error Handling

The system has built-in error handling mechanisms to deal with common issues such as Bluetooth disconnection, sensor failures, and more. Errors are indicated by specific LED patterns.

## Contributing

If you wish to contribute, please fork the repository and submit a pull request. For major changes, please open an issue to discuss the proposed changes.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

Feel free to clone, modify, and use the code as per your requirements. If you find any issues or have feature requests, please open an issue or submit a pull request.
