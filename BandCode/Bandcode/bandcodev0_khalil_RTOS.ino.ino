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
int bluetoothDelayTime = 10000 ; 
unsigned long lastBeatTime = 0;
unsigned long currentBeatTime;
int beatsPerMinute;

unsigned long startTime;
const unsigned long recordingDuration = 5 * 60 * 1000; // 10 minutes in milliseconds
const unsigned long dataInterval = 500; // Collect data every 1 second
const int maxDataPoints = recordingDuration / dataInterval;


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

void readSensors(SensorData& data)  {
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
  Serial.print("Temperature corp = ");
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

  if (dataCount < maxDataPoints) {
    dataBuffer[dataCount] = data;
    dataCount++;
  }
}


QueueHandle_t dataQueue;

void sensorTask(void * parameter) {
  while(1) {
    SensorData data;
    Serial.printf("--------------capture Data Task running on core %d----------------\n", xPortGetCoreID());
    readSensors(data);

    xQueueSend(dataQueue, &data, portMAX_DELAY);
    delay(1000);
  }
}

void bluetoothTask(void * parameter) {
  SensorData data;
  while(1) {
    if(xQueueReceive(dataQueue, &data, portMAX_DELAY)) {// Check if it's time to send data
  if (millis() - startTime >= recordingDuration) {
    // Reset the start time
    startTime = millis();
  
dataString = ""; // Clear the dataString
        // Send collected data via Bluetooth
        Serial.printf("--------------Send Data Task running on core %d----------------\n", xPortGetCoreID());
    for (int i = 0; i < dataCount; i++) {

       dataString += "Heart Rate: " + String(dataBuffer[i].pulseSensorValue) + " BPM ,";
       dataString += "Temp Corp: " + String(dataBuffer[i].objectTempC) + "°C ,";
       dataString += "Acceleration X: " + String(dataBuffer[i].accelerationX) + " m/s^2 ,";
       dataString += "Acceleration Y: " + String(dataBuffer[i].accelerationY) + " m/s^2 ,";
       dataString += "Acceleration Z: " + String(dataBuffer[i].accelerationZ) + " m/s^2 ,";
       dataString += "Rotation X: " + String(dataBuffer[i].rotationX) + " rad/s ,";
       dataString += "Rotation Y: " + String(dataBuffer[i].rotationY) + " rad/s ,";
       dataString += "Rotation Z: " + String(dataBuffer[i].rotationZ) + " rad/s ,";
       dataString += "Temperature: " + String(dataBuffer[i].temperature) + "  degC ,";
       dataString += "EDA: " + String(dataBuffer[i].conductance) + " \n";
      SerialBT.print(dataString);
      delay(50);

      //Serial.println(sizeof(dataString));
    }
  
 // Reset data count and buffer
     dataCount = 0;
  }
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
