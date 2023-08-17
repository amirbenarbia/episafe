#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <BluetoothSerial.h>
#define PIN_VO 12
#define PIN_VBIAS 14
// arduino voltage
#define VDD 5.0
// reference voltage for ADC
#define VREF 5.0
// input resolution
#define MAX_READ 1024

BluetoothSerial SerialBT;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

const int PulseSensorPurplePin = 36;
const int xpin = A0;
const int ypin = A3;
const int zpin = A4;

int Signal; // Holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 2000; // Determine which Signal to "count as a beat" and which to ignore.

unsigned long lastBeatTime = 0; // The time when the last beat occurred
unsigned long currentBeatTime; // The time when the current beat occurs
int beatsPerMinute; // Calculated heart rate in BPM

unsigned long startTime; // To store the start time
const unsigned long recordingDuration = 10 * 60 * 1000; // 10 minutes in milliseconds

struct SensorData {
  int pulseSensorValue;
  float objectTempC;
  float xAxis;
  float yAxis;
  float zAxis;
  float conductance;
};

const int bufferSize = 600; // Number of data points (adjust as needed)
SensorData dataBuffer[bufferSize];
int bufferIndex = 0;


void readPulseSensor(SensorData& data) {

   data.pulseSensorValue = analogRead(PulseSensorPurplePin);
  Signal = data.pulseSensorValue; // Use the stored pulse sensor value
  currentBeatTime = millis(); // Get the current time.
  
  // If the signal is above the threshold and enough time has passed since the last beat
  if (Signal > Threshold && (currentBeatTime - lastBeatTime) > 200) {
    // Calculate the time interval between beats
    int beatInterval = currentBeatTime - lastBeatTime;
  
    // Calculate heart rate in beats per minute (BPM)
    beatsPerMinute = 60000 / beatInterval;
  
    // Store the current time as the last beat time
    lastBeatTime = currentBeatTime;
  
    // Print the heart rate to the Serial Monitor
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

void readAccelerometer(SensorData& data) {
  data.xAxis = analogRead(xpin);
  data.yAxis = analogRead(ypin);
  data.zAxis = analogRead(zpin);
}

void readEDA(SensorData& data) {
   float vo = analogRead(PIN_VO) * VREF / MAX_READ;
  float vbias = analogRead(PIN_VBIAS) * VREF / MAX_READ;
  // simple computation
  float vdiff = abs(vo - vbias);
  // conductance, formula from  Poh et al. A wearable sensor for unobtrusive, long-term assesment of electrodermal activity. IEEE Transactions on Biomedical Engineering
  //float conductance = ((vbias - vo) / (VDD - vbias)) / 1000000;
  // actually don't put the resistance, value too small for us
  // NB: keep raw value to avoid rounding errors?
  data.conductance = ((vbias - vo) / (VDD - vbias))*1000;
  // even boost value for display
  Serial.print(" EDA: ");
  Serial.println(data.conductance);

}


void setup() {

  pinMode(PIN_VO, INPUT);
  pinMode(PIN_VBIAS, INPUT);


  Serial.begin(115200);
   SerialBT.begin("ESP32-Bluetooth"); // Bluetooth device name

  Wire.begin();

  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  }
}

void loop() {
unsigned long currentTime = millis();

  if (currentTime - startTime < recordingDuration) {
    // Collect sensor data and store it in the buffer
    SensorData data;
    readPulseSensor(data);
    readTemperatureSensor(data);
    readAccelerometer(data);
    readEDA(data);

    if (bufferIndex < bufferSize) {
      dataBuffer[bufferIndex] = data;
      bufferIndex++;
    }
    
    delay(1000); // Adjust delay as needed
  } else {
    // Send the stored data over Bluetooth
    for (int i = 0; i < bufferIndex; i++) {
      SerialBT.print("Pulse: ");
      SerialBT.print(dataBuffer[i].pulseSensorValue);
      SerialBT.print(" Temp: ");
      SerialBT.print(dataBuffer[i].objectTempC);
      SerialBT.print(" X: ");
      SerialBT.print(dataBuffer[i].xAxis);
      SerialBT.print(" Y: ");
      SerialBT.print(dataBuffer[i].yAxis);
      SerialBT.print(" Z: ");
      SerialBT.print(dataBuffer[i].zAxis);
      SerialBT.print(" EDA: ");
      SerialBT.println(dataBuffer[i].conductance);
      
      delay(100); // Delay between sending data points over Bluetooth
    }

    SerialBT.end(); // End Bluetooth communication
    bufferIndex = 0; // Reset buffer index to start storing new data
    delay(1000); // Wait for a moment before restarting the recording
    startTime = millis(); // Reset the start time for the new recording
  }
}