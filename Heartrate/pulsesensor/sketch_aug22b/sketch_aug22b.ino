#include <PulseSensorPlayground.h>

const int pulsePin = 34;  // Analog pin connected to Pulse Sensor
const int blinkPin = 13;  // LED pin for heartbeat visualization
int thresh = 550;         // Threshold value for detecting a heartbeat

PulseSensorPlayground pulseSensor;

void setup() {
  Serial.begin(115200);
  pulseSensor.analogInput(pulsePin);
  pulseSensor.blinkOnPulse(blinkPin);
  pulseSensor.setThreshold(thresh);
}

void loop() {
  int heartRate = pulseSensor.getBeatsPerMinute();

  if (pulseSensor.sawNewSample()) {
    Serial.println("Heart Rate: " + String(heartRate) + " BPM");
  }
}
