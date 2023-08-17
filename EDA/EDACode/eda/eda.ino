#define PIN_VO A0
#define PIN_VBIAS A1
// arduino voltage
#define VDD 5.0
// reference voltage for ADC
#define VREF 5.0
// input resolution
#define MAX_READ 1024
void setup() {
  pinMode(PIN_VO, INPUT);
  pinMode(PIN_VBIAS, INPUT);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  float vo = analogRead(PIN_VO) * VREF / MAX_READ;
  float vbias = analogRead(PIN_VBIAS) * VREF / MAX_READ;
  // simple computation
  float vdiff = abs(vo - vbias);
  // conductance, formula from  Poh et al. A wearable sensor for unobtrusive, long-term assesment of electrodermal activity. IEEE Transactions on Biomedical Engineering
  //float conductance = ((vbias - vo) / (VDD - vbias)) / 1000000;
  // actually don't put the resistance, value too small for us
  // NB: keep raw value to avoid rounding errors?
  float conductance = ((vbias - vo) / (VDD - vbias));
  // even boost value for display
  conductance = 1000 * conductance;

  Serial.println(conductance);
  delay(100);
}
