ESP32 Sensor Data Collection and Bluetooth Transmission

/Overview:

This project integrates an ESP32 board to collect data from a range of sensors, such as temperature, acceleration, pulse rate, and electrodermal activity (EDA). Once collected, the data is then transmitted via Bluetooth. This README provides a structured walkthrough for setting up, understanding, and executing the project.

/Features:

- Collects temperature readings from an MLX90614 infrared thermometer.
- Retrieves accelerometer, gyroscope, and temperature data from an MPU6050 sensor.
- Measures heart rate using a Pulse Sensor.
- Reads EDA values through analog pins.
- Transmits the consolidated sensor data over Bluetooth every 10 seconds.

/Hardware Requirements:

ESP32 development board
Adafruit_MLX90614 (Infrared Thermometer)
Adafruit_MPU6050 (Accelerometer & Gyroscope)
Pulse Sensor
Electrodes and resistors for EDA measurement

/Software Requirements:

Arduino IDE
Necessary Arduino libraries listed in the requirements.txt file.

/Usage:

- Wire up all the sensors to the ESP32 as per their respective datasheets.
- Ensure all required libraries are installed. You can find the list in the requirements.txt file.
- Open the provided Arduino code in the Arduino IDE.
- Upload the code to the ESP32.
- Open the Serial Monitor to view real-time sensor readings.
- Pair a Bluetooth device with the ESP32 to receive the sensor data at 10-second intervals.

/Code Structure:

- Initialization: Sets up Serial communications, Bluetooth, and initializes the sensors.
- Sensor Task: Periodically reads data from all sensors.
- Bluetooth Task: Transmits the sensor data over Bluetooth.
- Main Loop: Executes other tasks, in this case, it's empty and delays for 1 second.

/Enhancements & Contributions:

Contributors are welcome to further enhance the project by:

- Implementing the MySensors library for a more streamlined code structure.
- Adding more sensors or functionalities.
- Improving data visualization or analysis.
- Optimizing the code for energy efficiency or speed.
- Implementing a mobile or web-based interface for real-time data visualization and analysis.

