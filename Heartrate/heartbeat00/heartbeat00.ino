#include <PulseSensorPlayground.h>

const int pulsePin = 32;  // Change this to the actual GPIO pin your pulse sensor is connected to
const int ledPin = 13;    // LED indicator pin

PulseSensorPlayground pulseSensor;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  
  pulseSensor.analogInput(pulsePin);
  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(1); // Set output type to BPM
  pulseSensor.setThreshold(550); // Adjust this threshold value based on your sensor and environment
}

void loop() {
  int pulse = pulseSensor.getBeatsPerMinute();
  
  if (pulse > 0) {
    digitalWrite(ledPin, HIGH); // Turn on LED when pulse is detected
    Serial.print("BPM: ");
    Serial.println(pulse);
  } else {
    digitalWrite(ledPin, LOW); // Turn off LED when no pulse is detected
  }
  
  delay(10); // Adjust delay as needed
}
