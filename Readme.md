ESP32 Sensor Data Collection and Bluetooth Transmission
Overview:

This project integrates an ESP32 board to collect data from a range of sensors, such as temperature, acceleration, pulse rate, and electrodermal activity (EDA). Once collected, the data is then transmitted via Bluetooth. This README provides a structured walkthrough for setting up, understanding, and executing the project.

Features:

Collects temperature readings from MLX90614 sensor.
Retrieves accelerometer, gyroscope, and temperature data from MPU6050.
Measures heart rate via a Pulse Sensor.
Reads EDA values.
Transmits consolidated sensor data over Bluetooth every 10 minutes.
Setup:

Hardware Requirements:
ESP32 development board
Adafruit_MLX90614 (Infrared Thermometer)
Adafruit_MPU6050 (Accelerometer & Gyroscope)
Pulse Sensor
Software Requirements:
Arduino IDE
Necessary Arduino libraries listed in the requirements.txt file.
Usage:

Wire up all the sensors to the ESP32 as per their respective datasheets.
Ensure all required libraries are installed. You can find the list in requirements.txt.
Open the given Arduino code in the Arduino IDE.
Upload the code to the ESP32.
Open the Serial Monitor for real-time sensor readings.
Pair a Bluetooth device with the ESP32 to receive sensor data at 10-minute intervals.
Code Structure:
Initialization: Sets up Serial communications, Bluetooth, and initializes the sensors.
Main Loop: Reads data from all sensors, prints results to Serial, and sends data over Bluetooth at regular intervals.
Future Work & Contributions:
Contributors are encouraged to enhance the project by:

Implementing the MySensors library for a cleaner structure.
Adding more sensors or functionalities.
Improving data presentation or analysis.

