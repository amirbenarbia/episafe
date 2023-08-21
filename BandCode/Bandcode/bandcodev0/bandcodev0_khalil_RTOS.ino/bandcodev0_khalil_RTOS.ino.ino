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
  data.pulseSensorValue = analogRead(PulseSensorPurplePin);
  Signal = data.pulseSensorValue;
  currentBeatTime = millis();
  if (Signal > Threshold && (currentBeatTime - lastBeatTime) > 200) {
    int beatInterval = currentBeatTime - lastBeatTime;
    beatsPerMinute = 60000 / beatInterval;
    lastBeatTime = currentBeatTime;
  }

  data.objectTempC = mlx.readObjectTempC();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  data.accelerationX = a.acceleration.x;
  data.accelerationY = a.acceleration.y;
  data.accelerationZ = a.acceleration.z;
  data.rotationX = g.gyro.x;
  data.rotationY = g.gyro.y;
  data.rotationZ = g.gyro.z;
  data.temperature = temp.temperature;

  float vo = analogRead(PIN_VO) * VREF / MAX_READ;
  float vbias = analogRead(PIN_VBIAS) * VREF / MAX_READ;
  data.conductance = ((vbias - vo) / (VDD - vbias)) * 1000;
}

QueueHandle_t dataQueue;

void sensorTask(void * parameter) {
  while(1) {
    SensorData data;
    readSensors(data);
    xQueueSend(dataQueue, &data, portMAX_DELAY);
    delay(1000);
  }
}

void bluetoothTask(void * parameter) {
  SensorData data;
  while(1) {
    if(xQueueReceive(dataQueue, &data, portMAX_DELAY)) {
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
      SerialBT.println(dataString);
      delay(10000);
    }
  }
}

void setup() {
  pinMode(PIN_VO, INPUT);
  pinMode(PIN_VBIAS, INPUT);
  Serial.begin(115200);
  SerialBT.begin("ESP32-Bluetooth");
  Wire.begin();
  
  if (!mpu.begin()) {
    while (1) {
      delay(10);
    }
  }
  
  if (!mlx.begin()) {
    while (1);
  }
  
  dataQueue = xQueueCreate(100, sizeof(SensorData));
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 20000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(bluetoothTask, "BluetoothTask", 20000, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
