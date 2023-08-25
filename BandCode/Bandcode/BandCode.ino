#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
//

enum ErrorType {
  BT_DISCONNECTED,
  SENSOR_INIT_FAIL,
  EDA_NEG,
};
struct SensorData;
void showError(ErrorType error);


#define PIN_VO 12
#define PIN_VBIAS 14

#define VDD 5.0
#define VREF 5.0
#define MAX_READ 1024
#define LED_ERROR_PIN 15
#define LED_Sent 15
#define LED_collect 18
#define EDA_pin 25



void blinkLED(int ledPin, int blinkDuration = 500) {
  pinMode(ledPin, OUTPUT);  // Set the LED pin as OUTPUT

  digitalWrite(ledPin, HIGH);  // Turn ON the LED
  delay(blinkDuration / 2);    // Wait for the specified duration

  digitalWrite(ledPin, LOW);  // Turn OFF the LED
  delay(blinkDuration / 2);
  digitalWrite(ledPin, HIGH);  // Turn ON the LED
  delay(blinkDuration / 2);    // Wait for the specified duration

  digitalWrite(ledPin, LOW);  // Turn OFF the LED
  delay(blinkDuration / 2);   // Wait for the specified duration
}


float read_eda(int pin) {
  int sensorValue = 0;
  int gsr_average = 0;
  int human_resistance = 0;
  int EDA_th_per_epch = 5;  //10 par defaut

  long sum = 0;
  for (int i = 0; i < EDA_th_per_epch; i++)  //Mittelwertbildung über 10 Werte
  {
    sensorValue = analogRead(pin);  //Auslesen des analogen Pins
    sum += sensorValue;             //Summe der ausgelesenen Werte
  }
  gsr_average = sum / EDA_th_per_epch ;

  //Berechnung Hautwiderstand
  human_resistance = ((4096 + 2 * gsr_average) * 10000) / (2048 - gsr_average);

  return ((1.0 / human_resistance) * 1000000);
}

void showError(ErrorType error) {
  switch (error) {
    case BT_DISCONNECTED:
      //Serial.println("Bluetooth Disconnected!");
      break;
    case SENSOR_INIT_FAIL:
      //Serial.println("Sensor Initialization Failed!");
      break;
    case EDA_NEG:
      //Serial.println("EDA-conductance negative");
      // Add more error messages here
      break ; 
  }
  blinkLED(LED_ERROR_PIN, 500);  // Halt the system (optional)
}

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
  float time;
};

String Query(SensorData dt) {
  String data = "";
  data += String(dt.time) + "/";
  data += String(dt.pulseSensorValue) + ";";
  data += String(dt.objectTempC) + ";";
  data += String(dt.accelerationX) + ",";
  data += String(dt.accelerationY) + ",";
  data += String(dt.accelerationZ) + ";";
  data += String(dt.rotationX) + ",";
  data += String(dt.rotationY) + ",";
  data += String(dt.rotationZ) + ";";
  data += String(dt.temperature) + ";";
  data += String(dt.conductance) + ";\n";
  return data;
}

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
const int PulseSensorPurplePin = 36;
int Signal;
int Threshold = 2000;
int bluetoothDelayTime = 10000;
unsigned long lastBeatTime = 0;
unsigned long currentBeatTime;
int beatsPerMinute;
int duration_collect = 250;
int duration_sending = 1000;
unsigned long startTime;
const unsigned long recordingDuration =  10*60 * 1000;  // 10 minutes in milliseconds
const unsigned long dataInterval = 500;             // Collect data every 1 second
const long maxDataPoints = recordingDuration / dataInterval;
int dataCount = 0;
SensorData dataBuffer[maxDataPoints];
String dataString = "";
QueueHandle_t dataQueue;




void readSensors(SensorData& data) {
  // Read pulse sensor
  data.pulseSensorValue = analogRead(PulseSensorPurplePin);
  Signal = data.pulseSensorValue;
  currentBeatTime = millis();

  if (Signal > Threshold && (currentBeatTime - lastBeatTime) > 200) {
    int beatInterval = currentBeatTime - lastBeatTime;
    beatsPerMinute = 60000 / beatInterval;
    lastBeatTime = currentBeatTime;
    /*Serial.print("Heart Rate: ");
    Serial.print(beatsPerMinute);
    Serial.println(" BPM");*/
  }

  // Read temperature sensor
  data.objectTempC = mlx.readObjectTempC();
  /*Serial.print("Temperature corp = ");
  Serial.print(data.objectTempC);
  Serial.println("°C");*/

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


  // Read EDA
  int x = read_eda(EDA_pin);
  if (x >= 0) {
    data.conductance = x;
  } else
    showError(EDA_NEG);
  /*Serial.print("eda value="); 
  Serial.println(x);*/

  if (dataCount < maxDataPoints) {
    dataBuffer[dataCount] = data;
    dataCount++;
  }
  data.time = (float)(millis() - startTime) / (1000.0 * 60.0);  
  Serial.print(Query(data)); 
}

void sensorTask(void* parameter) {
  while (1) {
    SensorData data;
    //Serial.printf("--------------capture Data Task running on core %d----------------\n", xPortGetCoreID());
    readSensors(data);
    blinkLED(LED_collect, duration_collect);
    xQueueSend(dataQueue, &data, portMAX_DELAY);
    delay(500);
  }
}

void bluetoothTask(void* parameter) {
  SensorData data;
  while (1) {
    if (xQueueReceive(dataQueue, &data, portMAX_DELAY)) {
      if (!SerialBT.hasClient()) {
        showError(BT_DISCONNECTED);  // This will halt the system, modify if you want a different behavior
      }                              // Check if it's time to send data
      if (millis() - startTime >= recordingDuration) {
        // Reset the start time
        startTime = millis();

        dataString = "";  // Clear the dataString
        // Send collected data via Bluetooth
        //Serial.printf("--------------Send Data Task running on core %d----------------\n", xPortGetCoreID());

        for (int i = 0; i < dataCount; i++) {

          dataString = Query(dataBuffer[i]);
          SerialBT.print(dataString);
          delay(50);
          //Serial.println(sizeof(dataString));
        }
        blinkLED(LED_Sent, duration_sending);
        // Reset data count and buffer
        dataCount = 0;
      }
    }
  }
}

void setup() {
  pinMode(PIN_VO, INPUT);
  pinMode(PIN_VBIAS, INPUT);
  pinMode(EDA_pin, INPUT);
  Serial.begin(115200);
  SerialBT.begin("ESP32-Bluetooth");
  if (!SerialBT.hasClient()) {
    showError(BT_DISCONNECTED);
  }
  digitalWrite(15, LOW);
  digitalWrite(2, LOW);
  Wire.begin();

  if (!mpu.begin()) {
    showError(SENSOR_INIT_FAIL);
  }

  // Check if MLX90614 sensor initialized successfully
  if (!mlx.begin()) {
    showError(SENSOR_INIT_FAIL);
  }

  dataQueue = xQueueCreate(50, sizeof(SensorData));
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 2000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(bluetoothTask, "BluetoothTask", 2000, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(500));
}