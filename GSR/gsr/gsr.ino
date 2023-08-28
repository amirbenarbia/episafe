const int GSR = 25;

int sensorValue = 0;

int gsr_average = 0;

void setup() {

  Serial.begin(9600);

}

void loop() {

  long sum = 0;

  for (int i = 0; i < 10; i++)

  {

    sensorValue = analogRead(GSR);

    sum += sensorValue;

    delay(5);

  }

  gsr_average = sum / 10;

  Serial.print("GSR SENSOR VALUE:");

  Serial.println(gsr_average);

}