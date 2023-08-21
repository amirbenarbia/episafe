#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PIN_VO 12
#define PIN_VBIAS 14
#define VDD 5.0
#define VREF 5.0
#define MAX_READ 1024

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
const int PulseSensorPurplePin = 36;

int Signal;
int Threshold = 2000;
unsigned long lastBeatTime = 0;
unsigned long currentBeatTime;
int beatsPerMinute;

unsigned long startTime;
const unsigned long recordingDuration = 10 * 60 * 1000;

struct SensorData {
  int pulseSensorValue;
  float objectTempC;
  float conductance;
  float accelerationX;
  float accelerationY;
  float accelerationZ;
  float rotationX;
  float rotationY;
  float rotationZ;
  float temperature;
};


void readSensors(SensorData& data) {
  // Read pulse sensor
  data.pulseSensorValue = analogRead(PulseSensorPurplePin);
  Signal = data.pulseSensorValue;
  currentBeatTime = millis();
  if (Signal > Threshold && (currentBeatTime - lastBeatTime) > 200) {
    int beatInterval = currentBeatTime - lastBeatTime;
    beatsPerMinute = 60000 / beatInterval;
    lastBeatTime = currentBeatTime;
    Serial.print("Heart Rate: ");
    Serial.print(beatsPerMinute);
    Serial.println(" BPM");
  }

  // Read temperature sensor
  data.objectTempC = mlx.readObjectTempC();
  Serial.print("Temperature = ");
  Serial.print(data.objectTempC);
  Serial.println("°C");

  // Read accelerometer_gyro_temp
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  data.accelerationX = a.acceleration.x;
  data.accelerationY = a.acceleration.y;
  data.accelerationZ = a.acceleration.z;
  data.rotationX = g.gyro.x;
  data.rotationY = g.gyro.y;
  data.rotationZ = g.gyro.z;
  data.temperature = temp.temperature;
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  // Read EDA
  float vo = analogRead(PIN_VO) * VREF / MAX_READ;
  float vbias = analogRead(PIN_VBIAS) * VREF / MAX_READ;
  data.conductance = ((vbias - vo) / (VDD - vbias)) * 1000;
  Serial.print(" EDA: ");
  Serial.println(data.conductance);
}

void setup() {
  pinMode(PIN_VO, INPUT);
  pinMode(PIN_VBIAS, INPUT);
  Serial.begin(115200);
  SerialBT.begin("ESP32-Bluetooth");
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  }
}

void loop() {
  SensorData data;
  readSensors(data);

  // Send data via Bluetooth every 10 minutes
  if (millis() - startTime >= recordingDuration) {
    // Reset the start time
    startTime = millis();

    // Convert SensorData to a string
    String dataString = "Heart Rate: " + String(data.pulseSensorValue) + " BPM\n";
    dataString += "Temperature: " + String(data.objectTempC) + "°C\n";
    dataString += "Acceleration X: " + String(data.accelerationX) + " m/s^2\n";
    dataString += "Acceleration Y: " + String(data.accelerationY) + " m/s^2\n";
    dataString += "Acceleration Z: " + String(data.accelerationZ) + " m/s^2\n";
    dataString += "Rotation X: " + String(data.rotationX) + " rad/s\n";
    dataString += "Rotation Y: " + String(data.rotationY) + " rad/s\n";
    dataString += "Rotation Z: " + String(data.rotationZ) + " rad/s\n";
    dataString += "Temperature: " + String(data.temperature) + " degC\n";
    dataString += "EDA: " + String(data.conductance) + "\n";

    // Send data via Bluetooth
    SerialBT.println(dataString);
  }

  delay(1000); // Adjust delay as needed
}
