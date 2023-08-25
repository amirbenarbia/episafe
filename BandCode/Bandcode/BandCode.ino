  #include <Wire.h>
  #include <Adafruit_MLX90614.h>
  #include <BluetoothSerial.h>
  #include <Adafruit_MPU6050.h>
  #include <Adafruit_Sensor.h>

  // Constants and Macros
  #define PIN_VO 12
  #define PIN_VBIAS 14
  #define PulseSensorPurplePin 36
  #define EDA_pin 25
  #define LED_ERROR_PIN 15
  #define LED_Sent 15
  #define LED_collect 18


  #define EDA_TH_PER_EPOCH 5
  #define VDD 5.0
  #define VREF 5.0
  #define MAX_READ 1024
  #define Threshold 2000
  #define DURATION_COLLECT 250
  #define DURATION_SENDING 1000
  #define RECORDING_DURATION 10 * 60 * 1000
  #define DATA_INTERVAL 500
  #define MAX_DATA_POINTS RECORDING_DURATION / DATA_INTERVAL
  #define enableDebugOutput 0

  enum ErrorType { BT_DISCONNECTED,
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
    float time;
  };


  void showError(ErrorType error);
  float read_eda(int pin);
  String Query(SensorData dt);
  void readSensors(SensorData& data);
  void sensorTask(void* parameter);
  void bluetoothTask(void* parameter);
  void blinkLED(int ledPin, int blinkDuration,int);


  BluetoothSerial SerialBT;
  Adafruit_MPU6050 mpu;
  Adafruit_MLX90614 mlx = Adafruit_MLX90614();
  int Signal, beatsPerMinute;
  unsigned long lastBeatTime = 0, currentBeatTime, startTime;
  int dataCount = 0;
  SensorData dataBuffer[MAX_DATA_POINTS];
  QueueHandle_t dataQueue;
  String dataString = "";

  void setup() {
    pinMode(PIN_VO, INPUT);
    pinMode(PIN_VBIAS, INPUT);
    pinMode(EDA_pin, INPUT);
    Serial.begin(115200);
    SerialBT.begin("ESP32-Bluetooth");

    if (!SerialBT.hasClient()) showError(BT_DISCONNECTED);
    if (!mpu.begin()) showError(SENSOR_INIT_FAIL);
    if (!mlx.begin()) showError(SENSOR_INIT_FAIL);

    dataQueue = xQueueCreate(50, sizeof(SensorData));
    xTaskCreatePinnedToCore(sensorTask, "SensorTask", 2000, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(bluetoothTask, "BluetoothTask", 2000, NULL, 1, NULL, 1);
  }

  void loop() {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  void showError(ErrorType error) {
    switch (error) {
      case BT_DISCONNECTED:
        if (enableDebugOutput)
          Serial.println("Bluetooth Disconnected!");
        break;
      case SENSOR_INIT_FAIL:
        if (enableDebugOutput)
          Serial.println("Sensor Initialization Failed!");
        break;
      case EDA_NEG:
        if (enableDebugOutput)
          Serial.println("EDA-conductance negative");
        break;
    }
    blinkLED(LED_ERROR_PIN, 300,3);
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
    String data = "";
    data += String(dt.time) + "/";
    data += String(dt.pulseSensorValue) + ";";
    data += String(dt.objectTempC) + ";";
    data += String(dt.accelerationX) + ",";
    data += String(dt.accelerationY) + ",";
    data += String(dt.accelerationZ) + ";";
    data += String(dt.temperature) + ";";
    data += String(dt.conductance) + ";\n";
    return data;
  }

  void readSensors(SensorData& data) {

    //HR
    data.pulseSensorValue = analogRead(PulseSensorPurplePin);
    Signal = data.pulseSensorValue;
    currentBeatTime = millis();

    if (Signal > Threshold && (currentBeatTime - lastBeatTime) > 200) {
      int beatInterval = currentBeatTime - lastBeatTime;
      beatsPerMinute = 60000 / beatInterval;
      lastBeatTime = currentBeatTime;
      if (enableDebugOutput) {
        Serial.print("Heart Rate: ");
        Serial.print(beatsPerMinute);
        Serial.println(" BPM");
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
    } else
      showError(EDA_NEG);
    if (enableDebugOutput) {
      Serial.print("eda value=");
      Serial.println(x);
    }
    //---------
    if (dataCount < MAX_DATA_POINTS) {
      dataBuffer[dataCount] = data;
      dataCount++;
    }
    data.time = (float)(millis() - startTime) / (1000.0 * 60.0);
    Serial.print(Query(data));
  }


  void sensorTask(void* parameter) {
    while (1) {
      SensorData data;
      if (enableDebugOutput)
        Serial.printf("--------------capture Data Task running on core %d----------------\n", xPortGetCoreID());
      readSensors(data);
      blinkLED(LED_collect, DURATION_COLLECT,1);
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
        if (millis() - startTime >= RECORDING_DURATION) {
          // Reset the start time
          startTime = millis();

          dataString = "";  // Clear the dataString
          // Send collected data via Bluetooth
          if (enableDebugOutput)
            Serial.printf("--------------Send Data Task running on core %d----------------\n", xPortGetCoreID());

          for (int i = 0; i < dataCount; i++) {

            dataString = Query(dataBuffer[i]);
            SerialBT.print(dataString);
            delay(50);
            //Serial.println(sizeof(dataString));
          }
          blinkLED(LED_Sent, DURATION_SENDING,1);
          // Reset data count and buffer
          dataCount = 0;
        }
      }
    }
  }

  void blinkLED(int ledPin, int blinkDuration = 500,int time=2) {
    pinMode(ledPin, OUTPUT);
    for (int i = 0; i < time; i++) {
      digitalWrite(ledPin, HIGH);
      delay(blinkDuration / 2);

      digitalWrite(ledPin, LOW);
      delay(blinkDuration / 2);
    }
  }
