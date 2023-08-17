#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!mlx.begin()) {
    Serial.println("Error connecting to MLX sensor. Check wiring.");
    while (1);
  };
}

void loop() {
  
  Serial.print("Temperature = "); 
  Serial.print(mlx.readObjectTempC()); 
  Serial.println("°C");
  
 

  Serial.println("-----------------------------------------------------------------");
  delay(1000);
}