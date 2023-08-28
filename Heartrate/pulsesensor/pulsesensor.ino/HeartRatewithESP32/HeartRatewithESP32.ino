int PulseSensorPurplePin = 36; // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0


int Signal; // Holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 2000; // Determine which Signal to "count as a beat" and which to ignore.

unsigned long lastBeatTime = 0; // The time when the last beat occurred
unsigned long currentBeatTime; // The time when the current beat occurs
int beatsPerMinute; // Calculated heart rate in BPM

void setup() {

  Serial.begin(9600); // Set up Serial Communication at a certain speed.
}

void loop() {
  Signal = analogRead(PulseSensorPurplePin); // Read the Pulse Sensor's value.
  currentBeatTime = millis(); // Get the current time.

  // If the signal is above the threshold and enough time has passed since the last beat
  if (Signal > Threshold && (currentBeatTime - lastBeatTime) > 500) {
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

  delay(10); // A small delay between readings
}
