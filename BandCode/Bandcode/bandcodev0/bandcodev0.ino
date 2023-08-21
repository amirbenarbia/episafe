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

void readPulseSensor(SensorData& data) {
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
}

void readTemperatureSensor(SensorData& data) {
    data.objectTempC = mlx.readObjectTempC();
    Serial.print("Temperature = ");
    Serial.print(data.objectTempC);
    Serial.println("°C");
}

void readAccelGyro(SensorData& data) {
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
}

void readEDA(SensorData& data) {
    float vo = analogRead(PIN_VO) * VREF / MAX_READ;
    float vbias = analogRead(PIN_VBIAS) * VREF / MAX_READ;
    data.conductance = ((vbias - vo) / (VDD - vbias)) * 1000;
    
    Serial.print("EDA: ");
    Serial.println(data.conductance);
}

void readSensors(SensorData& data) {
    readPulseSensor(data);
    readTemperatureSensor(data);
    readAccelGyro(data);
    readEDA(data);
}

String createDataString(const SensorData& data) {
    return 
        "+---------------------+------------+\n" +
        "| Parameter           | Value      |\n" +
        "+---------------------+------------+\n" +
        "| Heart Rate          | " + String(data.pulseSensorValue) + " BPM       |\n" +
        "| Temperature         | " + String(data.objectTempC) + "°C         |\n" +
        "| Acceleration X      | " + String(data.accelerationX) + " m/s^2 |\n" +
        "| Acceleration Y      | " + String(data.accelerationY) + " m/s^2 |\n" +
        "| Acceleration Z      | " + String(data.accelerationZ) + " m/s^2 |\n" +
        "| Rotation X          | " + String(data.rotationX) + " rad/s  |\n" +
        "| Rotation Y          | " + String(data.rotationY) + " rad/s  |\n" +
        "| Rotation Z          | " + String(data.rotationZ) + " rad/s  |\n" +
        "| Temp from MPU6050   | " + String(data.temperature) + "°C         |\n" +
        "| EDA                 | " + String(data.conductance) + "          |\n" +
        "+---------------------+------------+";
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
        startTime = millis();  // Reset the start time
        String dataString = createDataString(data);
        SerialBT.println(dataString);
    }

    delay(500);  // Adjust delay as needed
}
