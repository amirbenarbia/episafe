#define PIN_VO A0
#define PIN_VBIAS A1
#define VDD 5.0
#define VREF 5.0
#define MAX_READ 1024

const int numSamples = 10;
float samples[numSamples];

void setup() {
  pinMode(PIN_VO, INPUT);
  pinMode(PIN_VBIAS, INPUT);
  Serial.begin(115200);
  
  for (int i = 0; i < numSamples; i++) {
    samples[i] = 0;
  }
}

float calculateMovingAverage(float newValue) {
  static int sampleIndex = 0;
  static float sum = 0;
  
  sum = sum - samples[sampleIndex] + newValue;
  samples[sampleIndex] = newValue;
  
  sampleIndex = (sampleIndex + 1) % numSamples;
  
  return sum / numSamples;
}

void loop() {
  float vo = analogRead(PIN_VO) * VREF / MAX_READ;
  float vbias = analogRead(PIN_VBIAS) * VREF / MAX_READ;
  float vdiff = abs(vo - vbias);
  float conductance = ((vbias - vo) / (VDD - vbias));
  conductance = 1000 * conductance;

  float filteredConductance = calculateMovingAverage(conductance);
  
  // Print data for Serial Plotter
  Serial.print(conductance);
  Serial.print('\t');
  Serial.println(filteredConductance);

  delay(100);
}
