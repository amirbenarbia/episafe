#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
	Serial.begin(9600);
	while (!Serial);

	if (!mlx.begin()) {
		Serial.println("Error connecting to MLX sensor. Check wiring.");
		while (1);
	};
}

void loop() {
	
	Serial.print("T = "); 
  Serial.print(mlx.readObjectTempC()); Serial.println("*C");
	
	delay(500);
}
