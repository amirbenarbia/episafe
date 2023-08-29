
#define vib 33

void setup() {
  pinMode(vib,OUTPUT); 
  
}

void loop() {
  
  vibr(150);
  delay(500); 
}

void vibr(int dur){
  digitalWrite(vib,1); 
  delay(dur);  
  digitalWrite(vib,0);
}