#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#define PIN_VO 12
#define PIN_VBIAS 14
// arduino voltage
#define VDD 5.0
// reference voltage for ADC
#define VREF 5.0
// input resolution
#define MAX_READ 1024

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;
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
/* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
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

  Serial.println("");
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


//mpu6050 config
mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");

  
void loop() {
  SensorData data;

  readPulseSensor(data);
  readTemperatureSensor(data);
  readAccelerometer(data);
  readEDA(data);

 
  

   
  delay(1000); // Adjust delay as needed
}
