#include <OneButton.h>
#define pb1 2
#define pb2 4

OneButton b1 = OneButton(
  pb1,    // Input pin for the button
  false,  // Button is active high
  false   // Disable internal pull-up resistor
);

OneButton b2 = OneButton(
  pb2,    // Input pin for the button
  false,  // Button is active high
  false   // Disable internal pull-up resistor
);

void setup() {
  Serial.begin(115200); 
  setupButtons()  ; 


}
void loop() {
  // put your main code here, to run repeatedly:
  b1.tick();
  b2.tick();
}

void trig_sensor() {
  Serial.println("trig_senor");
  
}
void crise() {
  Serial.println("crise");
  
}
void reset() {
  Serial.println("reset");
  
}
void false_alarm() {
  Serial.println("False_alarme");
  
}

void setupButtons(){

  pinMode(pb1, OUTPUT);
  pinMode(pb2, OUTPUT);

  b1.attachClick(trig_sensor);
  b1.attachDuringLongPress(crise);

  b2.attachDuringLongPress(reset);
  b2.attachDoubleClick(false_alarm);

  b1.setLongPressIntervalMs(1000);
  b2.setLongPressIntervalMs(2000);

  b2.setClickMs(320); 
}
