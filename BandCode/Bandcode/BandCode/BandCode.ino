
#include <OneButton.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>

extern "C" {
#include "esp_bt_device.h"
}

#define vib 19
#define pb1 2
#define pb2 4
#define PulseSensorPurplePin 32
#define EDA_pin 25
#define LED_ERROR_PIN 5
#define LED_Sent 5
#define LED_collect 18
#define EDA_TH_PER_EPOCH 5
#define MAX_READ 1024
#define Threshold 2000
#define DURATION_COLLECT 250
#define DURATION_SENDING 1000
#define RECORDING_DURATION 20 * 1000
#define DATA_INTERVAL 250
#define MAX_DATA_POINTS RECORDING_DURATION / DATA_INTERVAL

int enableDebugOutput = 0;
TaskHandle_t BluetoothTaskHandle = NULL;
TaskHandle_t ButtonTaskHandle = NULL;

OneButton b1 = OneButton(
  pb1,    // Input pin for the button
  false,  // Button is active high
  false   // Disable internal pull-up resistor
);
OneButton b2 = OneButton(
  pb2,    // Input pin for the button
  false,  // Button is active high
  false   // Disable internal pull-up resistor
);
enum ErrorType { BT_DISCONNECTED,
                 BT_DISCONNECTED_WHILE_SENDING,
                 SENSOR_INIT_FAIL,
                 EDA_NEG };
struct SensorData {
  int pulseSensorValue;
  float objectTempC;
  float conductance;
  float accelerationX;
  float accelerationY;
  float accelerationZ;
  float temperature;
  float timeStamp;
  float batteryLevel;
};



void showError(ErrorType error);
float read_eda(int pin);
String Query(SensorData dt);
void readSensors(SensorData& data);
void sensorTask(void* parameter);
void bluetoothTask(void* parameter);
void blinkLED(int ledPin, int blinkDuration, int);
float readBatteryLevel(int pin);
float roundToOneDecimal(float num);
BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int Signal, beatsPerMinute;
unsigned long lastBeatTime = 0, currentBeatTime, startTime;
int dataCount = 0;
SensorData dataBuffer[MAX_DATA_POINTS];
QueueHandle_t dataQueue;
String dataString = "";
int BATTERY_PIN = 3;
uint8_t mac[6];

void setup() {
  Serial.begin(115200);
  pinMode(EDA_pin, INPUT);
  pinMode(vib, OUTPUT);
  setupButtons(); //push buttons
  mlx.begin();
  mpu.begin();
  SerialBT.begin("Episafe");
  dataQueue = xQueueCreate(5, sizeof(SensorData));
  esp_read_mac(mac, ESP_MAC_BT);
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 2000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(
    buttonTask,         // Task function
    "ButtonTask",       // Name of task
    2000,               // Stack size of task
    NULL,               // Parameter of the task
    2,                  // Priority of the task (High priority)
    &ButtonTaskHandle,  // Task handle to keep track of created task
    1                   // Core where the task should run
  );
}
void loop() {
  b1.tick();
  b2.tick();
  vTaskDelay(pdMS_TO_TICKS(500));
}

void showError(ErrorType error) {
  switch (error) {
    case BT_DISCONNECTED:
        Serial.println("Bluetooth Disconnected!");
      blinkLED(LED_ERROR_PIN, 300, 3);
      break;
    case SENSOR_INIT_FAIL:
        Serial.println("Sensor Initialization Failed!");
      blinkLED(LED_ERROR_PIN, 300, 3);
      break;
    case EDA_NEG:
        Serial.println("EDA-conductance negative");
      blinkLED(LED_ERROR_PIN, 150, 2);
      break;
    case BT_DISCONNECTED_WHILE_SENDING:
        Serial.println("FAILED TO SEND..");
      blinkLED(LED_ERROR_PIN, 150, 4);
      break;
  }
}
float read_eda(int pin) {
  int sensorValue = 0;
  int gsr_average = 0;
  int human_resistance = 0;
  int EDA_th_per_epch = 5;

  long sum = 0;
  for (int i = 0; i < EDA_th_per_epch; i++) {
    sensorValue = analogRead(pin);
    sum += sensorValue;
  }
  gsr_average = sum / EDA_th_per_epch;
  human_resistance = ((4096 + 2 * gsr_average) * 10000) / (2048 - gsr_average);

  return ((1.0 / human_resistance) * 1000000);
}
String Query(SensorData dt) {
  StaticJsonDocument<200> doc;
  doc["ID"] = "EPISAFE/" + String(mac[0], HEX) + ":" + String(mac[1], HEX) + ":" + String(mac[2], HEX) + ":" + String(mac[3], HEX) + ":" + String(mac[4], HEX) + ":" + String(mac[5], HEX);
  doc["timeStamp"] = roundToOneDecimal(dt.timeStamp);
  doc["pulseSensorValue"] = roundToOneDecimal(dt.pulseSensorValue);
  doc["objectTempC"] = roundToOneDecimal(dt.objectTempC);
  doc["accelerationX"] = roundToOneDecimal(dt.accelerationX);
  doc["accelerationY"] = roundToOneDecimal(dt.accelerationY);
  doc["accelerationZ"] = roundToOneDecimal(dt.accelerationZ);
  doc["temperature"] = roundToOneDecimal(dt.temperature);
  doc["conductance"] = roundToOneDecimal(dt.conductance);
  doc["batteryLevel"] = roundToOneDecimal(dt.batteryLevel);
  String json;
  serializeJson(doc, json);
  return json;
}
void readSensors(SensorData& data) {
  //HR
  data.pulseSensorValue = analogRead(PulseSensorPurplePin);
  Signal = data.pulseSensorValue;
  currentBeatTime = millis();
  check_sensors(); 
  if (Signal > Threshold && (currentBeatTime - lastBeatTime) > 200) {
    int beatInterval = currentBeatTime - lastBeatTime;
    beatsPerMinute = 60000 / beatInterval;
    lastBeatTime = currentBeatTime;
    if (enableDebugOutput) {
      Serial.print("Heart Rate: ");
      Serial.print(beatsPerMinute);
      Serial.println(" BPM");
      delay(50);
    }
  }

  //TMPC
  data.objectTempC = mlx.readObjectTempC();
  if (enableDebugOutput) {
    Serial.print("Temperature corp = ");
    Serial.print(data.objectTempC);
    Serial.println("°C");
  }

  //ACC
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  data.accelerationX = a.acceleration.x;
  data.accelerationY = a.acceleration.y;
  data.accelerationZ = a.acceleration.z;
  data.temperature = temp.temperature;
  if (enableDebugOutput) {
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");
    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");
  }

  //EDA
  int x = read_eda(EDA_pin);
  if (x >= 0) {
    data.conductance = x;
    if (enableDebugOutput) {
      Serial.print("eda value=");
      Serial.println(x);
    }
  } else
    showError(EDA_NEG);
  //battery
  data.batteryLevel = readBatteryLevel(BATTERY_PIN);
  if (enableDebugOutput) {
    Serial.print("Battery Level: ");
    Serial.print(data.batteryLevel);
    Serial.println("%");
  }

  //---------
  if (dataCount < MAX_DATA_POINTS) {
    dataBuffer[dataCount] = data;
    dataCount++;
  }
  data.timeStamp = (float)(millis() - startTime) ; 


  if (!enableDebugOutput)
    Serial.print(".-");

}

void sensorTask(void* parameter) {
  while (1) {
    SensorData data;
    if (enableDebugOutput)
      Serial.printf("--------------capture Data Task running on core %d----------------\n", xPortGetCoreID());
    readSensors(data);
    blinkLED(LED_collect, DURATION_COLLECT, 1);
    if (millis() - startTime >= RECORDING_DURATION) {
      if (BluetoothTaskHandle == NULL) {
        xTaskCreatePinnedToCore(bluetoothTask,"BluetoothTask",2000,NULL,1,&BluetoothTaskHandle,1);
        startTime = millis();
      }
      xQueueSend(dataQueue, &data, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }
}

void bluetoothTask(void* parameter) {
  SensorData data;
  bool allDataSent = false;  // Flag to track if all data has been sent

  while (1) {
    // Reset the flag when entering the loop
    allDataSent = false;

    if (xQueueReceive(dataQueue, &data, portMAX_DELAY)) {
      dataString = "";
      if (enableDebugOutput)
        Serial.printf("--------------Send Data Task running on core %d----------------\n", xPortGetCoreID());

      if (check_bl_cnx()) {
        for (int i = 0; i < dataCount; i++) {

          String jsonPayload = Query(dataBuffer[i]);
          Serial.println(jsonPayload);
          SerialBT.println(jsonPayload);
          delay(20);

          // Check if this is the last data point to be sent
          if (i == dataCount - 1) {
            allDataSent = true;
          }
        }
        if (allDataSent) { // Exit task only if all data is sent
          blinkLED(LED_Sent, DURATION_SENDING, 1);
          dataCount = 0;
          xQueueReset(dataQueue);
          break; // Exiting the while loop
        }
      }
    }
  }

  BluetoothTaskHandle = NULL;
  vTaskDelete(NULL);
}

float readBatteryLevel(int pin) {
  pinMode(pin, INPUT);
  int raw = analogRead(pin);
  // Convert the raw reading into voltage, assuming 3.3V reference
  float voltage = (float)raw / 4095.0 * 3.3;

  // Convert voltage to percentage, assuming 3.3V is 100% and 0V is 0%
  float percentage = (voltage / 3.3) * 100;
  percentage = 33;
  return percentage;
}
void blinkLED(int ledPin, int blinkDuration = 500, int time = 2) {
  pinMode(ledPin, OUTPUT);
  for (int i = 0; i < time; i++) {
    digitalWrite(ledPin, HIGH);
    delay(blinkDuration / 2);

    digitalWrite(ledPin, LOW);
    delay(blinkDuration / 2);
  }
}
float roundToOneDecimal(float num) {
  return round(num * 10.0) / 10.0;
}
void trig_sensor() {
  Serial.println("trig_senor");
}
void crise() {
  Serial.println("crise");
}
void reset() {
  Serial.println("reset");
  ESP.restart();
}
void debug() {
  Serial.println("debug");
  enableDebugOutput = !enableDebugOutput;
}
void false_alarm() {
  Serial.println("False_alarme");
}
void setupButtons() {

  pinMode(pb1, OUTPUT);
  pinMode(pb2, OUTPUT);

  b1.attachClick(trig_sensor);
  b1.attachDuringLongPress(crise);

  b2.attachDuringLongPress(reset);
  b2.attachDoubleClick(false_alarm);
  b2.attachClick(debug);

  b1.setLongPressIntervalMs(1000);
  b2.setLongPressIntervalMs(3500);

  b2.setClickMs(300);
}
void buttonTask(void* parameter) {
  while (1) {
    b1.tick();
    b2.tick();
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}
void vibr(int dur, int x) {

  for (int i = 0; i < x; i++) {
    digitalWrite(vib, 1);
    delay(dur);
    digitalWrite(vib, 0);
  }
}
int check_bl_cnx() {
  if (SerialBT.hasClient())
    return 1;
  else
    showError(BT_DISCONNECTED);
  delay(100);
  return 0;
}
void check_sensors() {
  if (!mpu.begin()) showError(SENSOR_INIT_FAIL);
  if (!mlx.begin()) showError(SENSOR_INIT_FAIL);
}

