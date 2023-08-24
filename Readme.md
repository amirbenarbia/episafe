ESP32 Sensor Data Collection and Bluetooth Transmission

/Overview:

This code is designed to work with the ESP32 board to collect and transmit various types of sensor data. It captures data such as heart rate, skin conductance, object temperature, and accelerometer-gyro data. It also has a Bluetooth functionality that allows the ESP32 to send data to a connected Bluetooth client.

/Features:

-Heart rate monitoring
-Object temperature measurement
-3-axis acceleration capture
-3-axis rotation (gyro) capture
-Ambient temperature reading
-Skin conductance (EDA)
-Data transmission over Bluetooth

/Hardware Requirements:

-ESP32 board
-MLX90614 Temperature Sensor
-MPU6050 Accelerometer-Gyro Sensor
-Pulse Sensor (Analog)
-LEDs for status indication

/Software Requirements:

-Arduino IDE
-Adafruit MLX90614 library
-Adafruit MPU6050 library
-Adafruit Unified Sensor library
-BluetoothSerial library
-Wire library

/Installation and Setup:

-Install the Arduino IDE: Arduino IDE Download
-Open the Arduino IDE and go to Sketch -> Include Library -> Manage Libraries.
-Install the following libraries:
  Adafruit MLX90614
  Adafruit MPU6050
  Adafruit Unified Sensor
-Connect the hardware as per the pin configuration mentioned in the code.
-Upload the code to the ESP32 board using the Arduino IDE.
-Open the Serial Monitor for debugging information

/How to Run:

-Power up the ESP32 board.
-Make sure the Bluetooth client is connected.
-The sensor data will be sent to the connected Bluetooth client at intervals.

/Contributing:
Contributions, issues, and feature requests are welcome! and the process for submitting pull requests to us.

